#!/bin/bash
echo "Start Demo"
echo "Moving Conveyor Belt"
rosservice call /gilbreth/conveyor/control "state:
  power: 20.0" 
echo "Start Spawning Objects"
rosservice call /start_spawn
echo "Start kinect_publisher"
rosrun gilbreth_perception kinect_publisher > /dev/null

