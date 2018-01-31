cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_grasp_planning)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  std_msgs
  std_srvs
  tf
  geometry_msgs
  moveit_msgs
  moveit_commander
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    tf
    geometry_msgs
    moveit_msgs
    moveit_commander
)


