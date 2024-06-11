from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.conditions

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/giovanni/Desktop/Mobile_Robots-1/src/planner_pkg/config.yaml',
            description='Path to the config file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(["/opt/ros/humble/share/turtlebot4_navigation/launch/localization.launch.py"]),
            launch_arguments={'map': 'src/map/diem_map.yaml'}.items()  # Passaggio del parametro "map" al file di lancio localization.launch.py
        ),
        Node(
            package='planner_pkg',
            executable='planner',
            name='planner_handler',
            output='screen',
            parameters=[{'config_file': config_file}],
            # Aggiungi condizioni di avvio se necessario
            # condition=launch.conditions.IfCondition('/mac_server')
        )
    ])
