#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class CropNavigator(Node):
    """
    Navigates to crops, stops when crop_camera signals, sends 'catch' to arm,
    then navigates to a fixed drop-off point and sends 'release'.
    Avoids obstacles detected by ultrasonic sensors.
    """

    def __init__(self):
        super().__init__('crop_navigator')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher for robot velocity commands (for exploration)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))

        # Publisher for arm commands (String: "catch" or "release")
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)

        # Subscribe to crop positions
        self.create_subscription(
            PointStamped,
            '/nav_crop_positions',
            self.navigate_to_crop,
            10
        )

        # Subscribe to crop centered signal from crop_camera
        self.create_subscription(
            Bool,
            '/crop_centered',
            self.crop_centered_callback,
            10
        )

        # Subscribe to ultrasonic warning
        self.ultrasonic_warning = False
        self.create_subscription(
            Bool,
            '/ultrasonic_warning',
            self.ultrasonic_callback,
            10
        )

        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Exploration state
        self.exploring = True
        self.exploration_speed = 0.2  # m/s
        self.rotation_speed = 0.2  # rad/s

        # Arm and drop-off state
        self.arm_state = None
        self.waiting_for_release = False
        self.dropoff_pose = (1.0, 0.0)  # Example fixed drop-off point (x, y) in map

        # Timer for exploration behavior (10 Hz)
        self.create_timer(0.1, self.explore)

        self.get_logger().info("CropNavigator node initialized - Starting exploration.")

    def ultrasonic_callback(self, msg):
        self.ultrasonic_warning = msg.data
        if self.ultrasonic_warning:
            self.get_logger().warn("Obstacle detected by ultrasonic! Stopping robot.")
            self.stop_robot()

    def explore(self):
        """
        Implements the exploration behavior:
        - Move forward at a constant speed
        - Rotate slowly to scan the environment
        - Stop and avoid if ultrasonic detects obstacle
        """
        if not self.exploring or self.ultrasonic_warning:
            self.stop_robot()
            return

        # Create and publish velocity command
        cmd = Twist()
        cmd.linear.x = self.exploration_speed  # Forward movement
        cmd.angular.z = self.rotation_speed    # Slow rotation
        self.cmd_vel_pub.publish(cmd)

    def navigate_to_crop(self, msg):
        """
        Callback when a crop is detected - stops exploration and navigates to the crop.
        """
        if self.waiting_for_release:
            return  # Don't navigate to new crops while waiting to drop off

        # Transform crop point from nav_camera frame to map frame
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time())
            crop_in_map = tf2_geometry_msgs.do_transform_point(msg, trans)
            self.get_logger().info(f"Crop in map frame: ({crop_in_map.point.x:.2f}, {crop_in_map.point.y:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Stop exploration
        self.exploring = False
        self.stop_robot()

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = crop_in_map.point.x
        goal_msg.pose.pose.position.y = crop_in_map.point.y
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation

        self.get_logger().info(f"Navigating to crop at ({crop_in_map.point.x}, {crop_in_map.point.y})...")

        # Wait for the navigation action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available!")
            self.exploring = True  # Resume exploration
            return

        # Send the navigation goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        """
        Handles the navigation response.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Navigation goal rejected!")
                self.exploring = True  # Resume exploration
                return

            self.get_logger().info("Navigation goal accepted.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in navigation response: {e}")
            self.exploring = True  # Resume exploration

    def navigation_result_callback(self, future):
        """
        Handles the navigation result.
        """
        try:
            result = future.result().result
            if result:
                self.get_logger().info("Successfully reached the crop!")
            else:
                self.get_logger().warn("Failed to reach the crop.")
        except Exception as e:
            self.get_logger().error(f"Error in navigation result: {e}")
        # Do not resume exploration; wait for crop_camera to signal

    def stop_robot(self):
        """
        Stops the robot's movement.
        """
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def crop_centered_callback(self, msg):
        """
        When crop is centered (from crop_camera), send 'catch' to arm, stop robot,
        then after next signal, go to drop-off and send 'release'.
        """
        if msg.data and self.arm_state != "catch":
            self.get_logger().info("Crop centered! Sending 'catch' to robot arm and stopping robot.")
            self.arm_pub.publish(String(data="catch"))
            self.arm_state = "catch"
            self.stop_robot()
            self.waiting_for_release = True
        elif not msg.data and self.arm_state == "catch" and self.waiting_for_release:
            # Fruit is picked, now go to drop-off
            self.get_logger().info("Fruit picked! Navigating to drop-off point.")
            self.navigate_to_dropoff()
            self.arm_state = "release"
            self.waiting_for_release = False

    def navigate_to_dropoff(self):
        """
        Navigates to a fixed drop-off point and sends 'release' to the arm.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.dropoff_pose[0]
        goal_msg.pose.pose.position.y = self.dropoff_pose[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Navigating to drop-off at ({self.dropoff_pose[0]}, {self.dropoff_pose[1]})...")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available for drop-off!")
            self.exploring = True
            return

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.dropoff_navigation_response_callback)

    def dropoff_navigation_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Drop-off navigation goal rejected!")
                self.exploring = True
                return

            self.get_logger().info("Drop-off navigation goal accepted.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.dropoff_navigation_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in drop-off navigation response: {e}")
            self.exploring = True

    def dropoff_navigation_result_callback(self, future):
        try:
            result = future.result().result
            if result:
                self.get_logger().info("Reached drop-off point! Sending 'release' to robot arm.")
                self.arm_pub.publish(String(data="release"))
            else:
                self.get_logger().warn("Failed to reach drop-off point.")
        except Exception as e:
            self.get_logger().error(f"Error in drop-off navigation result: {e}")
        finally:
            self.exploring = True  # Resume exploration

    def __del__(self):
        self.stop_robot()
        self.get_logger().info("Shutting down CropNavigator node.")

def main(args=None):
    rclpy.init(args=args)
    node = CropNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()