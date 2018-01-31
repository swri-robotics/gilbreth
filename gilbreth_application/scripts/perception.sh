#!/bin/bash
#source devel/setup.bash
gnome-terminal --tab --name="gilbreth_perception" -x bash -c "exec bash"
roslaunch gilbreth_perception gilbreth_perception.launch
