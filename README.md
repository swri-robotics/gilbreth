# Gilbreth
Robotic product handling simulation

[Lillian Moller Gilbreth](https://en.wikipedia.org/wiki/Lillian_Moller_Gilbreth)  was an American psychologist and industrial engineer.

## Getting Started

The project is based on Ubuntu 16.04, ROS Kinect, gazebo 7.0, wstool and catkin tools. Links to the instructions for installing ROS, Gazebo, etc. are provided below
- [Install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Install gazebo 7.0](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [Install wstool](http://wiki.ros.org/wstool#Installation)
- [Install catkin-tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)

### Installation
- Source the ROS environment script
	
	```
	source /opt/ros/kinetic/setup.bash
	```
- Create a "gilbreth_ws" catkin workspace
 
	```
	mkdir -p ~/gilbreth_ws/src
	cd ~/gilbreth_ws/
	catkin init
	```
- Fetch the gilbreth source code:

	```
	cd ~/gilbreth_ws/src
	wstool init .
	wstool merge https://raw.githubusercontent.com/swri-robotics/gilbreth/kinetic-devel/gilbreth.rosinstall
	wstool update
	rosdep install --from-paths . --ignore-src -y
	```
- Build the workspace

	```
	cd ~/gilbreth_ws
	catkin build
	```

## Running Gilbreth

### Setup
- Source the workspace and add the Gazebo model path environment variables

	```
	cd ~/catkin_ws
	source devel/setup.bash
	bash src/gilbreth/gilbreth_gazebo/scripts/export_gazebo_model_path.bash
	```

### View the URDF

	```
	roslaunch urdf_tutorial display.launch model:=`rospack find gilbreth_support`/urdf/gilbreth.xacro
	```

### Run the simulation environment

1. Launch the gazebo environment:

	```
	roslaunch gilbreth_gazebo gilbreth.launch
	```
1. Activate the gripper:

	```
	rosservice call /gilbreth/gripper/control "enable: <true/false>"
	```
1. Activate the conveyor:

	```
	rosservice call /gilbreth/conveyor/control "state: power: <(0.0 - 100.0)>"
	```

1. Control the conveyor part spawner:
	
	```
	rosservice call /[start/stop]_spawn
	```
