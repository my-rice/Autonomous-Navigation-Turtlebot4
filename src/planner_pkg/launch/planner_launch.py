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
    config_path = PathJoinSubstitution([FindPackageShare('planner_pkg'), 'config', 'config.yaml'])

    Localization_monitor_node = Node(
        package='monitor_pkg',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        parameters=[{'service_name': '/amcl/change_state'}],
    )

    Planner_node = Node(
        package='planner_pkg',
        executable='planner',
        name='planner_handler',
        output='screen',
        parameters=[{'config_file': config_path}],
    )

    Localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot4_navigation"), '/launch', '/localization.launch.py']),
    )

    Navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot4_navigation"), '/launch', '/nav2.launch.py']),
        launch_arguments={'map': 'src/map/diem_map.yaml'}.items()  # Passaggio del parametro "map" al file di lancio localization.launch.py
    )

    View_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot4_viz"), '/launch', '/view_robot.launch.py']),
    )

    Navigation_monitor_node = Node(
        package='monitor_pkg',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        parameters=[{'service_name': '/planner_server/change_state'}],
    )
    return LaunchDescription([
    #    Localization_node,
    #    Localization_monitor_node,
    #    RegisterEventHandler(
    #         OnProcessExit(
    #             target_action=Localization_monitor_node,
    #             on_exit=[
    #                 LogInfo(msg='Monitor node has finished executing'),
    #                 EmitEvent(event=events.Shutdown(reason='Monitor node completed'))
    #             ],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         OnShutdown(
    #             on_shutdown=[
    #                 Navigation_node
    #             ]
    #         )
    #     ),
    #     Navigation_monitor_node,
    #     RegisterEventHandler(
    #         OnProcessExit(
    #             target_action=Navigation_monitor_node,
    #             on_exit=[
    #                 LogInfo(msg='Monitor node has finished executing'),
    #                 EmitEvent(event=events.Shutdown(reason='Monitor node completed'))
    #             ],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         OnShutdown(
    #             on_shutdown=[
    #                 View_robot_node, Planner_node
    #             ]
    #         )
    #     ),
    Planner_node
        
    ])