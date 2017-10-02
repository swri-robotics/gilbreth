# Gilbreth
Robotic product handling simulation

[Lillian Moller Gilbreth](https://en.wikipedia.org/wiki/Lillian_Moller_Gilbreth)  was an American psychologist and industrial engineer.

## Getting Started

The project is based on Ubuntu 16.04 and ROS Kinect. To install ROS and Gazebo, the following links might be helpful.
```
http://wiki.ros.org/kinetic/Installation/Ubuntu

http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
```
Be sure to install gazebo_ros_packages.
```
http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
```
### Installation

Create a catkin workspace.
```
http://wiki.ros.org/catkin/Tutorials/create_a_workspace
```
Second, go to src folder in your workspace:
```
cd ~/catkin_ws/src
```
Third,clone gilbreth_gazebo and gilbreth_description folder:
```
git clone https://github.com/swri-robotics/gilbreth.git
```
Make the files
```
cd ~/catkin_ws
catkin_make
```

## Running Gilbreth

### Setup
Source the workspace and add the Gazebo model path environment variables
```
cd ~/catkin_ws
source devel/setup.bash
bash src/gilbreth/gilbreth_support/scripts/export_gazebo_model_path.bash
```

### View the URDF
```
roslaunch urdf_tutorial display.launch model:=`rospack find gilbreth_support`/urdf/gilbreth.xacro
```

### Run the simulation environment

Roslaunch the gazebo environment:
```
roslaunch gilbreth_support gilbreth.launch
```
Activate the gripper:
```
rosservice call /gilbreth/gripper/control "enable: <true/false>"
```
Activate the conveyor:
```
rosservice call /gilbreth/conveyor/control "state: power: <(0.0 - 100.0)>"
```
Control the conveyor part spawner:
```
rosservice call /[start/stop]_spawn
```
