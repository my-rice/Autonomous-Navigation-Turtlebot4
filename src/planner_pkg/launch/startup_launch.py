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
    
    map_path = PathJoinSubstitution([FindPackageShare('config_pkg'), 'map', 'diem_map_clean.yaml'])
    
    Localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot4_navigation"), '/launch', '/localization.launch.py']),
        launch_arguments={'map': map_path}.items()
    )

    params_file = PathJoinSubstitution([FindPackageShare('config_pkg'), 'params', 'nav2_params.yaml'])

    Navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot4_navigation"), '/launch', '/nav2.launch.py']),
        launch_arguments={'params_file': params_file}.items()
    )

 

    View_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("planner_pkg"), '/launch', '/robot_launch.py']),
    )

  

    return LaunchDescription([
        Localization_node,
        Navigation_node,
        View_robot_node
    ])
