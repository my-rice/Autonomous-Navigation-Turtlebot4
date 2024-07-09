from launch import LaunchDescription, events
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, TimerAction
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnShutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config_path = PathJoinSubstitution([FindPackageShare('planner_pkg'), 'config', 'config.yaml'])

    Planner_node = Node(
        package='planner_pkg',
        executable='planner',
        name='planner_handler',
        output='screen',
        parameters=[{'config_file': config_path}],
    )
  

    Discovery_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("discovery_pkg"), '/launch', '/discovery_launch.py']),
    )

    return LaunchDescription([
        Planner_node,
    ])
