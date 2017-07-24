cmake_minimum_required(VERSION 2.8.3)
project(gilbreth_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(DIRECTORY msg
  FILES
  ConveyorBeltState.msg
  DetectedObject.msg
  Kit.msg
  KitObject.msg
  KitTray.msg
  LogicalCameraImage.msg
  Model.msg
  Order.msg
  PopulationState.msg
  Proximity.msg
  StorageUnit.msg
  TrayContents.msg
  VacuumGripperState.msg
)

add_service_files(DIRECTORY srv
  FILES
  AGVControl.srv
  ConveyorBeltControl.srv
  GetMaterialLocations.srv
  PopulationControl.srv
  SubmitTray.srv
  VacuumGripperControl.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_srvs
  tf
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gilbreth_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#link_directories(${GAZEBO_LIBRARY_DIRS})
#include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/VacuumGripperPlugin.cc)
target_link_libraries(${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libROSVacuumGripperPlugin.so library.
set(ros_vacuum_gripper_plugin_name ROSVacuumGripperPlugin)
add_library(${ros_vacuum_gripper_plugin_name} src/ROSVacuumGripperPlugin.cc)
target_link_libraries(${ros_vacuum_gripper_plugin_name}
  ${catkin_LIBRARIES}
  ${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_vacuum_gripper_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Create the libSideContactPlugin.so library.
set(side_contact_plugin_name SideContactPlugin)
add_library(${side_contact_plugin_name} src/SideContactPlugin.cc)
target_link_libraries(${side_contact_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${side_contact_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Create the libConveyorBeltPlugin.so library.
set(conveyor_belt_plugin_name ConveyorBeltPlugin)
add_library(${conveyor_belt_plugin_name} src/ConveyorBeltPlugin.cc)
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
install(TARGETS ${conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSConveyorBeltPlugin.so library.
set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)
add_library(${ros_conveyor_belt_plugin_name} src/ROSConveyorBeltPlugin.cc)
target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSAGVPlugin.so library.
set(ros_agv_plugin_name ROSAGVPlugin)
add_library(${ros_agv_plugin_name} src/ROSAGVPlugin.cc)
target_link_libraries(${ros_agv_plugin_name} ${GAZEBO_LIBRARIES})
add_dependencies(${ros_agv_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_agv_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libProximityRayPlugin.so library.
set(proximity_ray_plugin_name ProximityRayPlugin)
add_library(${proximity_ray_plugin_name} src/ProximityRayPlugin.cc)
target_link_libraries(${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSProximityRayPlugin.so library.
set(ros_proximity_ray_plugin_name ROSProximityRayPlugin)
add_library(${ros_proximity_ray_plugin_name} src/ROSProximityRayPlugin.cc)
target_link_libraries(${ros_proximity_ray_plugin_name}
  ${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_proximity_ray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libPopulationPlugin.so library.
set(population_plugin_name PopulationPlugin)
add_library(${population_plugin_name} src/PopulationPlugin.cc)
target_link_libraries(${population_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${population_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSPopulationPlugin.so library.
set(ros_population_plugin_name ROSPopulationPlugin)
add_library(${ros_population_plugin_name} src/ROSPopulationPlugin.cc)
target_link_libraries(${ros_population_plugin_name}
  ${catkin_LIBRARIES}
  ${population_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${ros_population_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_population_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAriacTaskManagerPlugin.so library.
set(task_plugin_name ROSAriacTaskManagerPlugin)
add_library(${task_plugin_name}
  src/ROSAriacTaskManagerPlugin.cc
)
target_link_libraries(${task_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  AriacScorer
)
add_dependencies(${task_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${task_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSLogicalCameraPlugin.so library.
set(ros_logical_camera_plugin_name ROSLogicalCameraPlugin)
add_library(${ros_logical_camera_plugin_name} src/ROSLogicalCameraPlugin.cc)
target_link_libraries(${ros_logical_camera_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${ros_logical_camera_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_logical_camera_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libObjectDisposalPlugin.so library.
set(object_disposal_plugin_name ObjectDisposalPlugin)
add_library(${object_disposal_plugin_name} src/ObjectDisposalPlugin.cc)
target_link_libraries(${object_disposal_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
install(TARGETS ${object_disposal_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSAriacKitTrayPlugin.so library.
set(kit_tray_plugin_name ROSAriacKitTrayPlugin)
add_library(${kit_tray_plugin_name} src/ROSAriacKitTrayPlugin.cc)
target_link_libraries(${kit_tray_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${side_contact_plugin_name}
)
add_dependencies(${kit_tray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${kit_tray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAriacKitTray.so library.
set(target_name AriacKitTray)
add_library(${target_name} src/AriacKitTray.cpp)
target_link_libraries(${target_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${target_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAriacScorer.so library.
set(target_name AriacScorer)
add_library(${target_name} src/AriacScorer.cpp)
target_link_libraries(${target_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  AriacKitTray
)
add_dependencies(${target_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#############
## Install ##
#############



#############
## Testing ##
#############

