cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_support)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_msgs
  std_srvs
  tf
  xmlrpcpp
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    conveyor_spawner
    urdf_creator
  CATKIN_DEPENDS
    gazebo_msgs
    std_srvs
    tf
    xmlrpcpp
  DEPENDS
    TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(urdf_creator
  src/urdf_creator.cpp
)
target_link_libraries(urdf_creator
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_library(conveyor_spawner
  src/conveyor_spawner.cpp
)
target_link_libraries(conveyor_spawner
  ${catkin_LIBRARIES}
  urdf_creator
)

add_executable(conveyor_spawner_node
  src/conveyor_spawner_node.cpp
)
target_link_libraries(conveyor_spawner_node
  ${catkin_LIBRARIES}
  conveyor_spawner
)

add_executable(urdf_creator_test_node
  src/urdf_creator_test.cpp
)
target_link_libraries(urdf_creator_test_node
  ${catkin_LIBRARIES}
  urdf_creator
)
