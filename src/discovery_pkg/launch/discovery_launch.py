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
    config_path = PathJoinSubstitution([FindPackageShare('discovery_pkg'), 'config', 'config.yaml'])

    

    Discovery_node = Node(
        package='discovery_pkg',
        executable='discovery',
        name='discovery_handler',
        output='screen',
        parameters=[{'config_file': config_path}],
    )

    


    return LaunchDescription([
        Discovery_node
        
    ])