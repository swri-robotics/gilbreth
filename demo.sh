#!/bin/bash
source devel/setup.bash
chmod +x src/gilbreth/perception.sh
chmod +x src/gilbreth/tool_planner.sh
chmod +x src/gilbreth/trajectory_planner.sh
echo "Start Demo"
echo "Moving Conveyor Belt"
rosservice call /gilbreth/conveyor/control "state:
  power: 100.0" 
echo "Start Spawning Objects"
rosservice call /start_spawn
gnome-terminal --tab --name="gilbreth_perception" -x bash -c "./src/gilbreth/perception.sh;"
echo "Start kinect_publisher"
rosrun gilbreth_perception kinect_publisher
