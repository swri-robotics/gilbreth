#!/bin/bash
#source devel/setup.bash
#chmod u+x src/gilbreth/gilbreth_grasp_planning/scripts/tool_planner.py 
gnome-terminal --tab --name="trajectory_planner" -x bash -c "exec bash"
roslaunch gilbreth_grasp_planning tool_planning_demo.launch
