# How to run

## Run Empty Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py ```

## Run Square Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square ```

### Run Square Map Rviz
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=square rviz:=true ```

## Run Warehouse Map
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py world:=warehouse ```

## Run Diem Map 
Group 10 created a world which represents the diem map using the map2gazebo tool, more details are in the report. 
1. ``` source install/setup.bash ```
2. ``` ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py x:=-6.3 y:=0.0 world:=diem_map```
Note that x and y are the position of the robot when the world is launched.