cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_grasp_planning)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  gazebo_plugins
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  geometry_msgs
  message_generation
  moveit_commander
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    moveit_commander
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

