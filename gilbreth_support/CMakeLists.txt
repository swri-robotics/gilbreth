cmake_minimum_required(VERSION 2.8.3)

project(gilbreth_support)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  ur_description
  gazebo_ros
)

catkin_package()
