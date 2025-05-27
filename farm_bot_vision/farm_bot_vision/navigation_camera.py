#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ultralytics import YOLO
import cv2
import time


class NavigationCamera(Node):
    """
    ROS2 Node for the navigation camera.
    - Camera is on the front face, centered horizontally.
    - Reference: base_link is at the center of the cube, 30cm above ground.
    - navigation_camera_link is at:
        x = 0.0 m (centered, same as base_link)
        y = 0.0 m (centered, front face)
        z = 0.235 m (23.5 cm above base_link, so 53.5 cm from ground)
    """

    def __init__(self):
        super().__init__('navigation_camera')
        self.bridge = CvBridge()
        #self.model = YOLO("yolov8n.pt").to('cpu')
        #self.target_classes = [52, 53, 54, 55, 56]  # Update if using a custom model
        self.model = YOLO("/home/mario/farm_bot/src/farm_bot_vision/farm_bot_vision/models/best.pt").to('cpu')
        self.target_classes = [0]  # Use your crop class indices
        # --- Camera position relative to base_link ---
        self.camera_x = 0.0    # meters (centered)
        self.camera_y = 0.0    # meters (centered, front face)
        self.camera_z = 0.235  # meters (53.5cm from ground - 30cm base_link = 23.5cm)
        # ---------------------------------------------

        self.cap = cv2.VideoCapture('/dev/video1')  # Use correct device index/path for nav camera
        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open navigation camera.")
            return

        self.image_pub = self.create_publisher(Image, '/nav_camera/image_raw', 10)
        self.crop_pub = self.create_publisher(PointStamped, '/nav_crop_positions', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.2, self.process_frame)
        self.last_frame_time = time.time()
        self.get_logger().info("✅ NavigationCamera node initialized and camera opened.")

    def process_frame(self):
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error("❌ Camera is not initialized or has been disconnected.")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to read frame from camera.")
            return

        try:
            # Publish the camera image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            self.last_frame_time = time.time()

            # Broadcast the camera's transform
            self.broadcast_transform()

            # --- Crop detection and 3D point estimation ---
            results = self.model(frame, verbose=False)
            result = results[0]
            img_h, img_w = frame.shape[:2]
            fx = fy = 600  # <-- Set your camera's focal length in pixels (calibrate for your camera!)
            cx = img_w / 2
            cy = img_h / 2
            Z = 0.5  # meters in front of camera (estimate or use depth sensor)
            if hasattr(result, 'boxes') and result.boxes.xywh is not None:
                for box, cls in zip(result.boxes.xywh.cpu().numpy(), result.boxes.cls.cpu().numpy()):
                    if int(cls) in self.target_classes:
                        x, y, w, h = box
                        X = (x - cx) * Z / fx
                        Y = (y - cy) * Z / fy
                        crop_msg = PointStamped()
                        crop_msg.header.frame_id = "navigation_camera_link"
                        crop_msg.header.stamp = self.get_clock().now().to_msg()
                        crop_msg.point.x = X
                        crop_msg.point.y = Y
                        crop_msg.point.z = Z
                        self.crop_pub.publish(crop_msg)
                        self.get_logger().info(
                            f"Detected crop at ({X:.2f}, {Y:.2f}, {Z:.2f}) in navigation_camera_link, class ID {int(cls)}"
                        )

        except Exception as e:
            self.get_logger().error(f"❌ Error processing frame: {e}")

        if time.time() - self.last_frame_time > 2.0:
            self.get_logger().warn("⚠️ No frames published in the last 2 seconds.")

    def broadcast_transform(self):
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "navigation_camera_link"
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

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("Shutting down NavigationCamera node.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()