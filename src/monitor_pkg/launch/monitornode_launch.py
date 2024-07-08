from launch import LaunchDescription, events
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnShutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    Localization_monitor_node = Node(
        package='monitor_pkg',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        parameters=[{'service_name': '/map_server/get_state'}],
    )
    # Navigation_monitor_node = Node(
    #     package='monitor_pkg',
    #     executable='monitor_node',
    #     name='monitor_node',
    #     output='screen',
    #     parameters=[{'service_name': '/controller_server/get_state'}],
    # )


    return LaunchDescription([
        Localization_monitor_node,
        # Navigation_monitor_node
    ])