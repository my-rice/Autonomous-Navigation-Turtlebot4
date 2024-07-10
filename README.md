
# Mobile Robots for Critical Missions Project

Source code for the project work of Mobile Robots for Critical Missions.




## Requirements

Python version: 3.10


## How to launch

First of all the localization, nav2 and rviz node should be bringed up, by using the startup launch file created:

ros2 launch planner_pkg startup_launch.py

And if all the nodes are started correctly, the discovery and planner package are needed to correctly solve the task:

ros2 launch discovery_pkg discovery_launch.py
ros2 launch planner_pkg planner_launch.py

If the task should be performed in a simulation environment, the gazebo simulator will be used, by launching:

????? TODO: Not working map in gazebo