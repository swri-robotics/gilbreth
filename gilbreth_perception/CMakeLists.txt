cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_perception)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  gilbreth_msgs
  gilbreth_gazebo
  rospy
)

find_package(
     PCL 1.8 REQUIRED
)

catkin_python_setup()
###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(segmentation_node src/segmentation_node.cpp)
target_link_libraries(segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(recognition_node src/recognition_node.cpp)
target_link_libraries(recognition_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kinect_publisher src/kinect_publisher.cpp)
target_link_libraries(kinect_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alignment_node src/alignment_node.cpp)
target_link_libraries(alignment_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(voxelizer_node src/voxelizer_node.cpp)
target_link_libraries(voxelizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Install ##
#############

#############
## Testing ##
#############
