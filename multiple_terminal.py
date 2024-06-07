import subprocess
import time

# subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'colcon build --symlink-install;'])
# time.sleep(5)

commands = [
    'source install/setup.bash && ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py',
    'source install/setup.bash && ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map.yaml',
    'source install/setup.bash && ros2 launch turtlebot4_navigation nav2.launch.py',
    'source install/setup.bash && ros2 launch turtlebot4_viz view_robot.launch.py',
    'source install/setup.bash && ros2 run planner_pkg planner'
]

for cmd in commands:
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'echo -e "\033[32mLaunching command: {cmd}\033[0m"; {cmd}; exec bash'])