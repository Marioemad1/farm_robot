import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import sys
import termios
import tty


class ReceiverNode(Node):
    """
    ROS2 Node to receive commands from a mobile app or keyboard and publish them as Twist messages.
    """

    def __init__(self, mode='keyboard'):
        super().__init__('receiver_node')
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.mode = mode

        if mode == 'keyboard':
            self.setup_keyboard()
        else:
            self.get_logger().error("Invalid mode. Only 'keyboard' mode is supported in this version.")
            raise ValueError("Invalid mode.")

    def setup_keyboard(self):
        """
        Sets up keyboard input for controlling the robot.
        """
        self.get_logger().info("Keyboard control enabled.")
        self.get_logger().info("Use W/A/S/D to control the robot. Press Q to quit.")
        self.read_keyboard()

    def read_keyboard(self):
        """
        Reads keyboard input and publishes it as Twist messages.
        """
        def get_key():
            """
            Reads a single key press from the terminal.
            """
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                key = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return key

        while True:
            key = get_key()
            msg = Twist()

            if key == 'w':  # Move forward
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                self.get_logger().info("Moving forward")
            elif key == 's':  # Move backward
                msg.linear.x = -0.5
                msg.angular.z = 0.0
                self.get_logger().info("Moving backward")
            elif key == 'a':  # Turn left
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                self.get_logger().info("Turning left")
            elif key == 'd':  # Turn right
                msg.linear.x = 0.0
                msg.angular.z = -0.5
                self.get_logger().info("Turning right")
            elif key == 'q':  # Quit
                self.get_logger().info("Exiting keyboard control.")
                break
            else:
                self.get_logger().warn(f"Unknown key: {key}")
                continue

            # Publish the Twist message
            self.cmd_pub.publish(msg)

    def cleanup(self):
        """
        Cleans up resources when the node is destroyed.
        """
        self.get_logger().info("Shutting down ReceiverNode.")


def main(args=None):
    """
    Main function to initialize and run the ReceiverNode.
    """
    rclpy.init(args=args)
    node = ReceiverNode(mode='keyboard')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()