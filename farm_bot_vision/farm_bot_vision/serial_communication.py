import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import os


class SerialCommunication(Node):
    """
    ROS2 Node for serial communication with two Arduinos:
    - 'motors' Arduino receives velocity commands (/cmd_vel).
    - 'robot_arm' Arduino receives arm instructions (/arm_command).
    """

    def __init__(self):
        super().__init__('serial_communication')

        # Define ports for each Arduino (edit as needed)
        ports = {
            'motors': ['/dev/ttyUSB0', '/dev/ttyACM0'],
            'robot_arm': ['/dev/ttyUSB1', '/dev/ttyACM1']
        }
        self.serial_ports = {}

        for name, port_list in ports.items():
            for port in port_list:
                try:
                    if os.path.exists(port):
                        self.serial_ports[name] = serial.Serial(port, 9600, timeout=1)
                        self.get_logger().info(f"Connected to {name} Arduino on {port}")
                        break
                except Exception as e:
                    self.get_logger().warn(f"Failed to connect to {name} on {port}: {e}")
            if name not in self.serial_ports:
                self.get_logger().error(f"❌ No valid serial port found for {name} Arduino.")

        # Subscribe to /cmd_vel for motors Arduino
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Subscribe to /arm_command for robot_arm Arduino
        self.create_subscription(
            String,
            '/arm_command',
            self.arm_callback,
            10
        )

    def cmd_callback(self, msg):
        """
        Send velocity commands to the motors Arduino via serial.
        """
        ser = self.serial_ports.get('motors')
        if ser and ser.is_open:
            try:
                # Example: "VEL:0.2,0.0\n"
                ser.write(f"VEL:{msg.linear.x:.3f},{msg.angular.z:.3f}\n".encode())
            except Exception as e:
                self.get_logger().error(f"Serial write to motors failed: {e}")

    def arm_callback(self, msg):
        """
        Send arm instructions to the robot_arm Arduino via serial.
        """
        ser = self.serial_ports.get('robot_arm')
        if ser and ser.is_open:
            try:
                # Example: "ARM:catch\n" or "ARM:release\n"
                ser.write(f"ARM:{msg.data}\n".encode())
            except Exception as e:
                self.get_logger().error(f"Serial write to robot_arm failed: {e}")


def main(args=None):
    """
    Main function to initialize and run the SerialCommunication node.
    """
    rclpy.init(args=args)
    node = SerialCommunication()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()