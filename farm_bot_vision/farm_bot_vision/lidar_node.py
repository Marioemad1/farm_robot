import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import serial
import threading
import time


class LidarNode(Node):
    """
    ROS2 Node for reading custom Arduino LiDAR (VL53L0X on stepper),
    building an occupancy grid, and publishing it.
    """

    def __init__(self):
        super().__init__('lidar_node')

        # Serial port config (edit as needed)
        self.serial_port = '/dev/ttyUSB2'
        self.baudrate = 9600
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # Map parameters
        self.grid_size = 100  # 100x100 grid
        self.resolution = 0.05  # 5 cm per cell
        self.max_range = 0.4  # 400 mm in meters

        # Publisher for the occupancy grid map
        self.map_pub = self.create_publisher(OccupancyGrid, '/lidar_map', 10)

        # Store last scan points
        self.scan_points = []

        # Start serial reading in a thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()

        # Timer to publish map at 2 Hz
        self.create_timer(0.5, self.publish_map)

        self.get_logger().info("✅ Custom LidarNode initialized.")

    def read_serial(self):
        """
        Read angle,distance from Arduino and store as (x, y) points.
        """
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                # Expecting: angle,distance,
                parts = line.split(',')
                if len(parts) < 2:
                    continue
                angle = float(parts[0])
                distance = float(parts[1]) / 1000.0  # mm to meters
                if distance <= 0 or distance > self.max_range:
                    continue
                # Convert polar to cartesian (robot center is at grid center)
                theta = np.deg2rad(angle)
                x = distance * np.cos(theta)
                y = distance * np.sin(theta)
                self.scan_points.append((x, y))
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def publish_map(self):
        """
        Build and publish an occupancy grid from scan points.
        """
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        center = self.grid_size // 2

        # Mark occupied cells
        for x, y in self.scan_points:
            gx = int(x / self.resolution + center)
            gy = int(y / self.resolution + center)
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                grid[gx, gy] = 100  # Occupied

        # Optionally, clear scan_points for next scan
        self.scan_points = []

        # Publish the occupancy grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.frame_id = "base_link"
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.grid_size
        occupancy_grid.info.height = self.grid_size
        occupancy_grid.info.origin.position.x = -self.grid_size / 2 * self.resolution
        occupancy_grid.info.origin.position.y = -self.grid_size / 2 * self.resolution
        occupancy_grid.data = grid.flatten().tolist()

        self.map_pub.publish(occupancy_grid)

    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()