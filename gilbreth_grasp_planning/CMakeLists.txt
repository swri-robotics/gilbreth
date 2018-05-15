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
  gilbreth_msgs
  gilbreth_gazebo
  controller_manager_msgs
  moveit_core
  moveit_ros_planning_interface
  roscpp
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
    gilbreth_msgs
    gilbreth_gazebo
    controller_manager_msgs
    moveit_core
    moveit_ros_planning_interface
    roscpp
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

# Create URDF creator library
add_executable(robot_trajectory_executor
  src/robot_trajectory_executor.cpp
)
target_link_libraries(robot_trajectory_executor
  ${catkin_LIBRARIES}
)