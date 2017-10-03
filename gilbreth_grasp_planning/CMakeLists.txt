cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_grasp_planning)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear 
  gazebo_ros
  gazebo_plugins
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    osrf_gear
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
)

include_directories(
  include
  ${osrf_gear_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(start_conveyor src/start_conveyor.cpp)
add_dependencies(start_conveyor ${catkin_EXPORTED_TARGETS})
target_link_libraries(start_conveyor ${catkin_LIBRARIES})

add_executable(kinect_publisher src/kinect_publisher.cpp)
add_dependencies(kinect_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect_publisher ${catkin_LIBRARIES})


