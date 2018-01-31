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

## Visualize The Simulation Enviroment

### Setup
- Source the catkin workspace 

	```
	cd ~/catkin_ws
	source devel/setup.bash
	```

### View the URDF
	
	
	
   ```
      roslaunch urdf_tutorial display.launch model:=$(rospack find gilbreth_support)/urdf/gilbreth.xacro
   ```	
    
   It may be necessary to select "world" in the "Fixed Frame" drop down in the rviz window.
    

### Run the Gazebo simulation environment

1. Launch the gazebo simulation environment:

	```
  	roscd gilbreth_gazebo
  	source scripts/env_setup.bash
	roslaunch gilbreth_gazebo gilbreth.launch rviz:=false
	```

  	- The **source scripts/env_setup.bash** command sets up environment variables needed
  	by the gazebo simulator.  
    - Use "rviz:=true" to show rviz
  	
1. Activate the gripper:

	```
	rosservice call /gilbreth/gripper/control "enable: true"
	```
    Use "false" in order to turn off the gripper

1. Activate the conveyor:

	```
	rosservice call /gilbreth/conveyor/control "state: power: 100.0"
	```
    The "power" can range from 0.0 to 100.0.  Use 0.0 to stop the conveyor

1. Start the part spawner:
	
	```
	rosservice call /start_spawn "{}"
	```
    This command will make parts appear on the conveyor at random intervals. The frequency, types of parts and other properties can be configured in this [yaml file](gilbreth_gazebo/config/conveyor_objects.yaml)
    
1. Stop the part spawner
	```
	rosservice call /stop_spawn "{}"

### Run the Main Application
 - [See here](DEMO.md)
