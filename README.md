
# Mobile Robots for Critical Missions Project

Source code for the project work of Mobile Robots for Critical Missions.

## Requirements

Python version: 3.10 <br />
Python libraries for QReader: pyzbar, pytorch, qrdet, numpy <br />
Python software suite for QReader: zbar-tools, to install through 
``` sudo apt-get install zbar-tools ```
command.

## How to launch

First of all the localization, nav2 and rviz node should be brought up, by using the startup launch file created:
``` ros2 launch planner_pkg startup_launch.py ```
And if all the nodes are started correctly, the discovery and planner package are needed to correctly solve the task:
``` ros2 launch discovery_pkg discovery_launch.py ```
``` ros2 launch planner_pkg planner_launch.py ```

If the task should be performed in a simulation environment, the gazebo simulator will be used, by launching:

``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py x:=-6.3 y:=0.0 world:=diem_map ```

When all the nodes are up, use Rviz2 to estimate the initial pose and then the navigation will start.

## Video Example




https://github.com/user-attachments/assets/bb4c342b-8148-4e41-a206-4988f7ee03f8

