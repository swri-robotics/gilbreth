// This node has two function:
// Call service to start moving the conveyor belt
// Call service to start spawning random objects in Gazebo

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <osrf_gear/ConveyorBeltControl.h>

//start the object spawner
void start_spawner(ros::NodeHandle & node) {
  ros::ServiceClient start_client = node.serviceClient<std_srvs::Empty>("/start_spawn");

  if (!start_client.exists()) {
    ROS_INFO("Waiting for the object spawner to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Spawner is ready.");
  }
  ROS_INFO("Request to start spawning objects on conveyor belt.");
  std_srvs::Empty srv;  
  start_client.call(srv);  
}

//start moving conveyor belt 
void move_conveyor(ros::NodeHandle & node){
  ros::ServiceClient conveyor_client = node.serviceClient<osrf_gear::ConveyorBeltControl>("/gilbreth/conveyor/control");

  if (!conveyor_client.exists()) {
    ROS_INFO("Waiting for the conveyor belt controller to be ready...");
    conveyor_client.waitForExistence();
    ROS_INFO("Conveyor belt controller is ready.");
  }
  ROS_INFO("Start moving conveyor belt...");
  osrf_gear::ConveyorBeltControl srv;
  
  srv.request.state.power = 100.0;
  conveyor_client.call(srv);
  if (!srv.response.success) {  
    ROS_ERROR_STREAM("Failed to start moving conveyor belt");
  } else {
    ROS_INFO("CStart moving conveyor belt...");
  }
}

int main(int argc, char ** argv){  	
  ros::init(argc, argv, "gilbreth_start_conveyor_node");

  ros::NodeHandle node;

  start_spawner(node);
  move_conveyor(node);
  
  return 0;
}
