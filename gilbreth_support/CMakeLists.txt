cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_support)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  std_srvs
  tf
  xmlrpcpp
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    std_srvs
    tf
    xmlrpcpp
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(conveyor_spawner_node src/conveyor_spawner_node.cpp)
target_link_libraries(conveyor_spawner_node ${catkin_LIBRARIES})
