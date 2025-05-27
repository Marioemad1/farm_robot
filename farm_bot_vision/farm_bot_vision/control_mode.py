#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess


class ControlMode(Node):
    """
    ROS2 Node to manage different control modes for the farm bot.
    """

    def __init__(self):
        super().__init__('control_mode_node')

        # Password for access
        self.password = "22156"

        # Prompt for password
        self.authenticate()

        # Display control mode options
        print("\n=== Farm Bot Control ===")
        print("1) Autonomous Mode")
        print("2) Mobile App Control")
        print("3) Keyboard Control")

        # Select and start the desired mode
        self.select_mode()

    def authenticate(self):
        """
        Prompts the user for a password and loops until the correct password is entered.
        """
        while True:
            user_input = input("Enter the password to access the control modes: ").strip()
            if user_input == self.password:
                print("✅ Access granted.")
                break
            else:
                print("❌ Incorrect password. Please try again.")

    def select_mode(self):
        """
        Prompts the user to select a control mode and starts the corresponding nodes.
        """
        mode = input("Select mode (1/2/3): ").strip()

        if mode == '1':
            self.start_autonomous()
        elif mode == '2':
            self.start_mobile_app()
        elif mode == '3':
            self.start_keyboard_control()
        else:
            print("❌ Invalid selection. Please restart and choose a valid mode.")
            return

    def start_autonomous(self):
        """
        Starts the autonomous mode with the required nodes.
        """
        # Launch nodes for autonomous mode
        self.launch_node('crop_camera')
        self.launch_node('crop_navigator')
        self.launch_node('navigation_camera')
        self.launch_node('lidar_node')
        self.launch_node('ultrasonic_node')
        self.launch_node('serial_communication')

        self.get_logger().info("✅ Started Autonomous Mode.")

    def start_mobile_app(self):
        """
        Starts the mobile app control mode with the required nodes.
        """
        # Launch nodes for mobile app control
        self.launch_node('receiver_node', '--mode mobile')
        self.launch_node('serial_communication')

        self.get_logger().info("✅ Started Mobile App Control Mode.")

    def start_keyboard_control(self):
        """
        Starts the keyboard control mode with the required nodes.
        """
        # Launch nodes for keyboard control
        self.launch_node('receiver_node', '--mode keyboard')
        self.launch_node('serial_communication')

        self.get_logger().info("✅ Started Keyboard Control Mode.")

    def launch_node(self, node_name, args=""):
        """
        Launches a ROS2 node in a new terminal.
        """
        try:
            # Use subprocess to open a new terminal and run the node
            subprocess.Popen(
                f"gnome-terminal -- bash -c 'ros2 run farm_bot_vision {node_name} {args}; exec bash'",
                shell=True
            )
            self.get_logger().info(f"✅ Launched {node_name} in a new terminal.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to launch {node_name}: {e}")


def main(args=None):
    """
    Main function to initialize and run the ControlMode node.
    """
    rclpy.init(args=args)
    control = ControlMode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()