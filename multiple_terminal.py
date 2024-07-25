import subprocess
import time

subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'colcon build;'])
time.sleep(5)

commands = [ 
    'source install/setup.bash && ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py',#x:=-6.3 y:=0.0 world:=diem_map',
    'source install/setup.bash && ros2 launch planner_pkg startup_launch.py',
    'source install/setup.bash && ros2 launch discovery_pkg discovery_launch.py',
    'source install/setup.bash && ros2 launch planner_pkg planner_launch.py',

]

for i,cmd in enumerate(commands):
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'echo -e "\033[32mLaunching command: {cmd}\033[0m"; {cmd}; exec bash'])
    
    if i==0:
        time.sleep(3) 
    else:
        time.sleep(2)
