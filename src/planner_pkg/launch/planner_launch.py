from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/giovanni/Desktop/Mobile_Robots-1/src/planner_pkg/config.yaml',
            description='Path to the config file'
        ),
        Node(
            package='planner_pkg',
            executable='planner',
            name='planner_handler',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    ])
