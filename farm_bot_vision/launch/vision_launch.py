from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farm_bot_vision',
            executable='control_mode',
            name='control_mode_node',
            output='screen'
        )
    ])
