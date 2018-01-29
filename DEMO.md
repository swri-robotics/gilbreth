# How to launch demo
## Getting Started
- fetch gilbreth source code
- fetch moveit source code (https://github.com/ros-planning/moveit)
- copy and paste the 'moveit_commander' package in the same /src folder with gilbreth

## Demo
### Environment
- to launch the environment
```
roslaunch gilbreth_gazebo gilbreth.launch
```
Both Gazebo and Rviz will be launched at the same time. 
Use Rviz to test the robot controller by dragging the end-effector to another pose and click 'plan and execute' in moveit panel.
If the robot failed to move to the target pose, then the robot controller is now working. 
Close Gazebo and Rviz and then relaunch the environment.
### Demo script
If the robot controller is working, run the demo script by
```
chmod +x src/gilbreth/demo.sh
./src/gilbreth/demo.sh
```
The script will run object_recognition, tool_planning, trajectory_planning and robot_execution nodes seperately.
### Launch node seperately:
- object segmentation and object recognition:
```
roslaunch gilbreth_perception gilbreth_perception.launch
```
- robot tool planner:
```
rosrun gilbreth_grasp_planning tool_planner.py
```
- robot trajectory planner:
```
rosrun gilbreth_grasp_planning trajectory_planner.py
```
- robot execution:
```
rosrun gilbreth_grasp_planning robot_execution.py
```
