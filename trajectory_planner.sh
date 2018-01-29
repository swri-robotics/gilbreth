#!/bin/bash
source devel/setup.bash
chmod u+x src/gilbreth/gilbreth_grasp_planning/scripts/trajectory_planner.py 
gnome-terminal --tab --name="robot_execution" -x bash -c "source devel/setup.bash;chmod u+x src/gilbreth/gilbreth_grasp_planning/scripts/robot_execution.py;rosrun gilbreth_grasp_planning robot_execution.py;exec bash"
roslaunch gilbreth_grasp_planning trajectory_planning_demo.launch
