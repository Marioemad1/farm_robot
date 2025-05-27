import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DistanceSensor  # For Raspberry Pi GPIO


class UltrasonicNode(Node):
    """
    ROS2 Node for two ultrasonic sensors.
    Publishes True on /ultrasonic_warning if either sensor detects object ≤ 5cm.
    """

    def __init__(self):
        super().__init__('ultrasonic_node')

        # --- Pin assignments ---
        # Ultrasonic 1: TRIG=GPIO17, ECHO=GPIO27
        # Ultrasonic 2: TRIG=GPIO22, ECHO=GPIO23
        self.sensor1 = DistanceSensor(echo=27, trigger=17, max_distance=2.0)
        self.sensor2 = DistanceSensor(echo=23, trigger=22, max_distance=2.0)
        # -----------------------

        self.warning_pub = self.create_publisher(Bool, '/ultrasonic_warning', 10)
        self.timer = self.create_timer(0.1, self.check_distance)
        self.get_logger().info("✅ UltrasonicNode with two sensors initialized.")

    def check_distance(self):
        # Distance in meters
        dist1 = self.sensor1.distance * 100  # convert to cm
        dist2 = self.sensor2.distance * 100  # convert to cm
        warning = dist1 <= 5 or dist2 <= 5
        if warning:
            self.get_logger().warn(f"⚠️ Obstacle detected! dist1={dist1:.1f}cm, dist2={dist2:.1f}cm")
        self.warning_pub.publish(Bool(data=warning))


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()