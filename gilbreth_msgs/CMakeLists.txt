cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_msgs)
find_package(catkin REQUIRED roscpp geometry_msgs std_msgs trajectory_msgs message_generation moveit_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObjectDetection.msg
  TargetToolPoses.msg
  RobotTrajectories.msg
  ObjectVoxel.msg
  ObjectType.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS  message_runtime roscpp geometry_msgs std_msgs trajectory_msgs moveit_msgs sensor_msgs
)

###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############

