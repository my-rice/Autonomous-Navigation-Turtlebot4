# import subprocess
# import time

# subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'colcon build;'])
# time.sleep(15)

# commands = [ 
#     #'source install/setup.bash && ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py',# x:=-6.3 y:=0.0 world:=diem_map',
#     'source install/setup.bash && ros2 launch turtlebot4_navigation localization.launch.py map:=src/map/diem_map_clean_noi.yaml',
#     'source install/setup.bash && ros2 launch turtlebot4_navigation nav2.launch.py',# params_file:=src/config_mobile_robots/nav2_params.yaml',
#     'source install/setup.bash && ros2 launch turtlebot4_viz view_robot.launch.py',
#     'source install/setup.bash && ros2 run discovery_pkg discovery',
#     'source install/setup.bash && ros2 launch planner_pkg planner_launch.py',
#     #'source install/setup.bash && ros2 run findtuning_pkg publish_circles',
# ]

# for i,cmd in enumerate(commands):
#     subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'echo -e "\033[32mLaunching command: {cmd}\033[0m"; {cmd}; exec bash'])
    
#     if i==0:
#         time.sleep(14)
#     else:
#         time.sleep(14)


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
        time.sleep(2) 
    else:
        time.sleep(1)
