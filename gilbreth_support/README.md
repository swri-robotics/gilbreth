# Gilbreth Support Package

A URDF representation and Gazebo simulation environment for Gilbreth Project.

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
source devel/setup.bash
```
## Running the testing environment

Source the workspace and add the Gazebo model path environment variables
```
cd ~/catkin_ws
source devel/setup.bash
bash src/gilbreth/gilbreth_support/scripts/export_gazebo_model_path.bash
```
Roslaunch the gazebo environment:
```
roslaunch gilbreth_support gilbreth.launch
```
