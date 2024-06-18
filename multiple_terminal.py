import subprocess
import time

subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'colcon build;'])
time.sleep(3)

commands = [ 
    'source install/setup.bash && ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py',# x:=-6.3 y:=0.0 world:=diem_map',
    'source install/setup.bash && ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map.yaml',
    'source install/setup.bash && ros2 launch turtlebot4_navigation nav2.launch.py',
    'source install/setup.bash && ros2 launch turtlebot4_viz view_robot.launch.py',
    'source install/setup.bash && ros2 run discovery_pkg discovery',
    'source install/setup.bash && ros2 launch planner_pkg planner_launch.py'
]

for i,cmd in enumerate(commands):
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'echo -e "\033[32mLaunching command: {cmd}\033[0m"; {cmd}; exec bash'])
    
    if i==0:
        time.sleep(3)
    else:
        time.sleep(1)
