#!/bin/bash
source devel/setup.bash
gnome-terminal --tab --name="gilbreth_perception" -x bash -c "./src/gilbreth/tool_planner.sh;exec bash"
roslaunch gilbreth_perception gilbreth_perception.launch
