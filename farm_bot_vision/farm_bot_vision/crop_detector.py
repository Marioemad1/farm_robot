import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO


class CropDetector(Node):
    """
    ROS2 Node for detecting crops using a YOLO model and publishing their positions.
    """

    def __init__(self):
        super().__init__('crop_detector')
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO("yolov8n.pt").to('cpu')  # Use CPU for compatibility

        # Subscribe to the camera image topic
        self.create_subscription(
            Image,
            '/crop_cam/image_raw',
            self.image_callback,
            10
        )

        # Publisher for crop positions
        self.crop_pub = self.create_publisher(PointStamped, '/crop_positions', 10)

        self.get_logger().info("CropDetector node initialized.")

    def image_callback(self, msg):
        """
        Callback to process the camera image and detect crops.
        """
        try:
            # Convert ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO inference
            results = self.model(frame, verbose=False)
            result = results[0]

            # Publish detected crop positions
            if hasattr(result, 'boxes') and result.boxes.xywh is not None:
                coords_list = result.boxes.xywh.cpu().numpy()
                for box in coords_list:
                    x, y, w, h = box
                    crop_msg = PointStamped()
                    crop_msg.header.frame_id = "map"
                    crop_msg.header.stamp = self.get_clock().now().to_msg()
                    crop_msg.point.x = x
                    crop_msg.point.y = y
                    crop_msg.point.z = 0.0  # Assume crops are on the ground
                    self.crop_pub.publish(crop_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    """
    Main function to initialize and run the CropDetector node.
    """
    rclpy.init(args=args)
    node = CropDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()