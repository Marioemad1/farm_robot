from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        Node(
            package='farm_bot_vision',
            executable='crop_camera',
            name='crop_camera_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='farm_bot_vision',
            executable='crop_navigator',
            name='crop_navigator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='farm_bot_vision',
            executable='navigation_camera',
            name='navigation_camera_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='farm_bot_vision',
            executable='serial_communication',
            name='serial_communication_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='farm_bot_vision',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='farm_bot_vision',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap_node',
            output='screen',
            parameters=[
                '/home/mario/farm_bot/src/farm_bot_vision/farm_bot_vision/config/rtabmap_params.yaml'
            ]
        )
    ])