# Gilbreth Gazebo Package

A Gazebo simulation environment for Gilbreth Project.

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
Setup the Gazebo environment path. Add the following line into .bashrc file
```
export GAZEBO_MODEL_PATH=`Yout Path to gilbreth_gazebo Package`/models
```
## Running the testing environment

Roslaunch the gazebo environment:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun gilbreth_gazebo run_gilbreth_test_sim.bash
```

## Changelog
07-19-2017 Add Gazebo Models, Worlds, Plugins and Ros Launch Files. 
