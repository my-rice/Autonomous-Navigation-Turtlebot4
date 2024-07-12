## planner_pkg
The planner package takes the role of global planner in our application. The main responsibilities are to define the positions to be reached by the robot according to the detected signs and handle possible situations.
It also contains launch files for startup of the entire localization and navigation stacks.

## How to launch
Using the launch file created, run the node:
``` ros2 launch planner_pkg planner_launch.py ```

For the localization,navigation and Rviz visualization node run:
``` ros2 launch planner_pkg startup_launch.py ```

