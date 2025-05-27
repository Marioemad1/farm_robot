#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2


class CropCamera(Node):
    """
    ROS2 Node for the BACK crop camera:
    - Detects crops using YOLOv8.
    - Publishes /crop_centered (Bool) when a crop is at the low center (ready to pick).
    - Publishes /arm_command ("catch") when crop is ready.
    - Stops the robot for 10 seconds when crop is centered.
    - Publishes the camera's TF relative to base_link.
    """

    def __init__(self):
        super().__init__('crop_camera')
        self.bridge = CvBridge()
        self.model = YOLO("/home/mario/farm_bot/src/farm_bot_vision/farm_bot_vision/models/best.pt").to('cpu')
        #self.target_classes = [0]  # Update with your crop class indices

        self.image_pub = self.create_publisher(Image, '/crop_cam/image_raw', 10)
        self.centered_pub = self.create_publisher(Bool, '/crop_centered', 10)
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Camera position relative to base_link ---
        self.camera_x = 0.0     # meters (centered)
        self.camera_y = -0.395  # meters (39.5cm behind base_link)
        self.camera_z = -0.085  # meters (8.5cm below base_link)
        # ---------------------------------------------

        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open back camera.")
            return

        self.timer = self.create_timer(0.2, self.process_frame)
        self.get_logger().info("✅ CropCamera (back) node initialized.")

        self.stopped = False
        self.sent_catch = False

    def broadcast_transform(self):
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "crop_camera_link"
            transform.transform.translation.x = self.camera_x
            transform.transform.translation.y = self.camera_y
            transform.transform.translation.z = self.camera_z
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f"❌ Error broadcasting transform: {e}")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to read frame from back camera.")
            return

        self.broadcast_transform()
        img_h, img_w = frame.shape[:2]
        centered = False

        try:
            results = self.model(frame, verbose=False)
            result = results[0]
            if hasattr(result, 'boxes') and result.boxes.xywh is not None:
                for box, cls in zip(result.boxes.xywh.cpu().numpy(), result.boxes.cls.cpu().numpy()):
                    if int(cls) in self.target_classes:
                        x, y, w, h = box
                        # Center of detected crop in image
                        center_x_img, center_y_img = x, y
                        # --- LOW CENTER LOGIC ---
                        # x near image center, y near bottom (low in image)
                        center_x_ok = abs(center_x_img - img_w/2) < 0.1 * img_w
                        center_y_ok = center_y_img > img_h * 0.8  # bottom 20% of image
                        if center_x_ok and center_y_ok:
                            centered = True
                            self.get_logger().info("Crop is in low center (ready to pick)!")
                            break

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f"❌ Error processing frame: {e}")

        # Publish centered status and stop robot/send catch if ready
        self.centered_pub.publish(Bool(data=centered))

        if centered and not self.stopped:
            self.get_logger().info("Stopping robot and sending 'catch' to arm.")
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.arm_pub.publish(String(data="catch"))
            self.stopped = True
            self.sent_catch = True

        # If not centered anymore, reset state (for next fruit)
        if not centered and self.stopped:
            self.get_logger().info("Fruit no longer detected in low center, ready for next operation.")
            self.stopped = False
            self.sent_catch = False

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CropCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
