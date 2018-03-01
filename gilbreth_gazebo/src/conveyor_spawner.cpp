#include <boost/filesystem.hpp>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include "gilbreth_gazebo/conveyor_spawner.h"
#include "gilbreth_gazebo/urdf_creator.h"
#include <random>
#include <ros/package.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <boost/format.hpp>

const static std::string GAZEBO_SPAWN_SERVICE = "gazebo/spawn_urdf_model";
const static std::string START_SPAWN_SERVICE = "start_spawn";
const static std::string STOP_SPAWN_SERVICE = "stop_spawn";
const static std::string SPAWNED_PART_TOPIC = "spawned_part";
static const std::string GAZEBO_SET_MODEL_STATE_SERVICE = "gazebo/set_model_state";
static const std::string GAZEBO_DISPOSED_MODELS_TOPIC = "gazebo/disposed_models";
const static double SRV_TIMEOUT = 10.0f;

std::string generateObjectName(int id)
{
  return "object_" + std::to_string(id);
}

namespace gilbreth
{
namespace simulation
{

ConveyorSpawner::ConveyorSpawner(ros::NodeHandle& nh)
  : nh_(nh)
{
}

bool ConveyorSpawner::init(XmlRpc::XmlRpcValue& p)
{
  // Load the parameters
  if(!loadSpawnParameters(p, params_))
  {
    return false;
  }

  // Connect to ROS topics/services/actions/etc.
  if(!connectToROS())
  {
    return false;
  }

  // Initialize the randomization engine
  srand(params_.randomization_seed);

  start_server_ = nh_.advertiseService(START_SPAWN_SERVICE, &ConveyorSpawner::start, this);
  stop_server_ = nh_.advertiseService(STOP_SPAWN_SERVICE, &ConveyorSpawner::stop, this);
  timer_ = nh_.createTimer(ros::Duration(params_.spawn_period), &ConveyorSpawner::spawnObjectTimerCb, this, false, false);
  pub_ = nh_.advertise<std_msgs::Header>(SPAWNED_PART_TOPIC, 10, true);

  return true;
}

void ConveyorSpawner::run()
{
  ros::spin();
}


bool ConveyorSpawner::loadSpawnParameters(const XmlRpc::XmlRpcValue& p,
                                          SpawnParameters& spawn_params) const
{
  XmlRpc::XmlRpcValue params = p;
  try
  {
    // Get the top-level parameters
    // Get the reference frame
    XmlRpc::XmlRpcValue& frame = params["reference_frame"];
    spawn_params.reference_frame = static_cast<std::string>(frame);

    // Get the spawn timing
    XmlRpc::XmlRpcValue& period = params["spawn_period"];
    spawn_params.spawn_period = static_cast<double>(period);

    // Get the randomization seed
    XmlRpc::XmlRpcValue& seed = params["randomization_seed"];
    spawn_params.randomization_seed = static_cast<int>(seed);

    // Get max objects
    XmlRpc::XmlRpcValue& max_objects = params["max_objects"];
    spawn_params.max_objects = static_cast<int>(max_objects);

    // Get the spawned objects
    XmlRpc::XmlRpcValue& objects = params["objects"];
    for(int i = 0; i < objects.size(); ++i)
    {
      XmlRpc::XmlRpcValue& obj = objects[i];
      ObjectParameters object_params;
      if(!loadObjectParameters(obj, object_params))
      {
        return false;
      }

      // Add the object's parameters to the list
      spawn_params.objects.push_back(object_params);
    }
  }
  catch(XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("Exception in loading spawner parameters:\n%s'", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConveyorSpawner::loadObjectParameters(const XmlRpc::XmlRpcValue& object,
                                           ObjectParameters& object_params) const
{
  XmlRpc::XmlRpcValue obj = object;

  try
  {
    // Get the name
    XmlRpc::XmlRpcValue& name = obj["name"];
    object_params.name = static_cast<std::string>(name);

    // Get the relative URDF path file
    XmlRpc::XmlRpcValue& filepath = obj["mesh_resource"];
    object_params.mesh_resource = static_cast<std::string>(filepath);

    // Get the initial pose
    XmlRpc::XmlRpcValue& initial_pose = obj["initial_pose"];

    XmlRpc::XmlRpcValue& position = initial_pose["position"];
    object_params.initial_pose.position.x = static_cast<double>(position[0]);
    object_params.initial_pose.position.y = static_cast<double>(position[1]);
    object_params.initial_pose.position.z = static_cast<double>(position[2]);

    XmlRpc::XmlRpcValue& orientation = initial_pose["orientation"];
    object_params.initial_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(static_cast<double>(orientation[0]),
                                                                               static_cast<double>(orientation[1]),
                                                                               static_cast<double>(orientation[2]));

    // Get the lateral placement variance
    XmlRpc::XmlRpcValue& lpv = obj["lateral_placement_variance"];
    object_params.lateral_placement_variance = static_cast<double>(lpv);

    // Get the yaw placement_variance
    XmlRpc::XmlRpcValue& ypv = obj["yaw_placement_variance"];
    object_params.yaw_placement_variance = static_cast<double>(ypv) * M_PI / 180.0f;

    // Get the spawn timing variance
    XmlRpc::XmlRpcValue& stv = obj["spawn_timing_variance"];
    object_params.spawn_timing_variance = static_cast<double>(stv);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("Exception in loading object parameters:\n%s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConveyorSpawner::connectToROS()
{
  spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SERVICE);
  if(!spawn_client_.waitForExistence(ros::Duration(SRV_TIMEOUT)))
  {
    ROS_ERROR("Timeout waiting for '%s' service", spawn_client_.getService().c_str());
    return false;
  }

  set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(GAZEBO_SET_MODEL_STATE_SERVICE);
  if(!set_state_client_.waitForExistence(ros::Duration(SRV_TIMEOUT)))
  {
    ROS_ERROR("Timeout waiting for '%s' service", set_state_client_.getService().c_str());
    return false;
  }

  disposed_objs_subs_ = nh_.subscribe<gazebo_msgs::ModelStates>(GAZEBO_DISPOSED_MODELS_TOPIC,1,
                                                                   &ConveyorSpawner::disposedObjectsCallback,this);

  return true;
}

bool ConveyorSpawner::start(std_srvs::EmptyRequest& req,
           std_srvs::EmptyResponse& res)
{
  ROS_INFO("Starting conveyor spawner...");
  timer_.start();
  return true;
}

bool ConveyorSpawner::stop(std_srvs::EmptyRequest& req,
          std_srvs::EmptyResponse& res)
{
  ROS_INFO("Stopping conveyor spawner...");
  timer_.stop();
  return true;
}

void ConveyorSpawner::spawnObjectTimerCb(const ros::TimerEvent& e)
{
  if(object_counter_ < params_.max_objects)
  {
    spawnObject();
  }
  else
  {
    recirculateObject();
  }
}

void ConveyorSpawner::spawnObject()
{
  ++object_counter_;

  // Randomize the model to be spawned
  int idx = rand() % params_.objects.size();
  auto obj = params_.objects.begin();
  std::advance(obj, idx);

  // Populate the spawn service request
  gazebo_msgs::SpawnModel srv;
  srv.request.reference_frame = params_.reference_frame;
  srv.request.robot_namespace = "gilbreth";
  srv.request.model_xml = createObjectURDF(obj->name, obj->mesh_resource);
  srv.request.initial_pose = obj->initial_pose;
  srv.request.model_name = generateObjectName(object_counter_);

  // Randomize the object's lateral spawn position
  // Create a new pseudo-random number between 0 and 1
  double r_lpv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& lpv = obj->lateral_placement_variance;
  double lpv_delta = -lpv + 2.0*r_lpv*lpv;
  srv.request.initial_pose.position.y += lpv_delta;

  // Randomize the object's spawn yaw angle
  // Create a new psuedo-random number between 0 and 1
  double r_ypv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& ypv = obj->yaw_placement_variance;
  double ypv_delta = -ypv + 2.0*r_ypv*ypv;
  tf::Quaternion q;
  tf::quaternionMsgToTF(srv.request.initial_pose.orientation, q);
  tf::Quaternion dq = tf::createQuaternionFromRPY(0.0, 0.0, ypv_delta);
  q *= dq;
  tf::quaternionTFToMsg(q, srv.request.initial_pose.orientation);

  // Randomize the delay in spawning the object
  // Create a new psuedo-random number between 0 and 1
  double r_tv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  ros::Duration time_var (r_tv * obj->spawn_timing_variance);
  time_var.sleep();

  // Call the spawn service
  if(!spawn_client_.call(srv))
  {
    ROS_ERROR("Failed to call '%s' service", spawn_client_.getService().c_str());
    --object_counter_;
    return;
  }
  else
  {
    if(!srv.response.success)
    {
      ROS_ERROR("%s", static_cast<std::string>(srv.response.status_message).c_str());
      --object_counter_;
      return;
    }
    else
    {
      // Publish which part was just spawned onto the conveyor
      std_msgs::Header msg;
      msg.frame_id = obj->name;
      msg.stamp = ros::Time::now();
      msg.seq = object_counter_;
      pub_.publish(msg);
    }
  }

  ROS_DEBUG_STREAM(boost::str(boost::format("Spawned new object '%1%' with id '%2%'") % obj->name % srv.request.model_name));

  // storing model name for tracking purposes
  spawned_objects_map_.insert(std::make_pair(srv.request.model_name, obj->name));
}

void ConveyorSpawner::recirculateObject()
{
  if(inactive_objects_ids_.empty())
  {
    return; // no objects to recirculate yet
  }

  // randomize object choice from inactive list
  int idx = rand() % inactive_objects_ids_.size();
  auto obj_id = inactive_objects_ids_.begin();
  std::advance(obj_id,idx);

  // locating object from list
  std::string obj_name = spawned_objects_map_[*obj_id];
  std::vector<ObjectParameters>::iterator obj = std::find_if(params_.objects.begin(),
                                                          params_.objects.end(),
                                                          [&obj_name](const ObjectParameters& obj){
                                                            return obj_name == obj.name; });

  // create the set state request
  gazebo_msgs::SetModelState srv;
  gazebo_msgs::ModelState& ms = srv.request.model_state;
  ms.model_name = *obj_id;
  ms.pose = obj->initial_pose;
  ms.reference_frame = params_.reference_frame;
  tf::vector3TFToMsg(tf::Vector3(0,0,0),ms.twist.linear);
  tf::vector3TFToMsg(tf::Vector3(0,0,0),ms.twist.angular);

  // randomizing the y placement (along conveyor width)
  double r_lpv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  double& lpv = obj->lateral_placement_variance;
  double lpv_delta = -lpv + 2.0*r_lpv*lpv;
  ms.pose.position.y += lpv_delta;

  // Randomize the object's yaw angle
  double r_ypv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  double& ypv = obj->yaw_placement_variance;
  double ypv_delta = -ypv + 2.0*r_ypv*ypv;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ms.pose.orientation, q);
  tf::Quaternion dq = tf::createQuaternionFromRPY(0.0, 0.0, ypv_delta);
  q *= dq;
  tf::quaternionTFToMsg(q, ms.pose.orientation);

  // Randomize the request call delay
  double r_tv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  ros::Duration time_var (r_tv * obj->spawn_timing_variance);
  time_var.sleep();

  // call the service
  if(!set_state_client_.call(srv))
  {
    ROS_ERROR_STREAM(boost::str(boost::format("Failed to call the '%1%' service") % set_state_client_.getService()));
    return;
  }

  if(!srv.response.success)
  {
    ROS_ERROR("The 'set_state' call failed");
    return;
  }

  ROS_DEBUG_STREAM(boost::str(boost::format("Recirculated object '%1%' with id '%2%'") % obj->name % *obj_id));

  // Publish which part was just recycled onto the conveyor
  std_msgs::Header msg;
  msg.frame_id = obj->name;
  msg.stamp = ros::Time::now();
  pub_.publish(msg);

  inactive_objects_ids_.erase(obj_id);
}

void ConveyorSpawner::disposedObjectsCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
  for(const auto& n: msg->name)
  {
    if(spawned_objects_map_.count(n))
    {
      inactive_objects_ids_.push_back(n);
    }
    else
    {
      ROS_WARN_STREAM(boost::str(boost::format("An unknown object '%1%' was received, ignoring ...") % n));
    }
  }
}

} // namespace simulation
} // namespace gilbreth
