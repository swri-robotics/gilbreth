#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <random>

const static std::string GAZEBO_SPAWN_SERVICE = "gazebo/spawn_urdf_model";
const static std::string START_SPAWN_SERVICE = "start_spawn";
const static std::string STOP_SPAWN_SERVICE = "stop_spawn";
const static double SRV_TIMEOUT = 10.0f;

namespace gilbreth
{
namespace simulation
{

struct SpawnParameters
{
  std::vector<std::string> model_topics;
  std::vector<std::string> model_descriptions;
  geometry_msgs::Pose initial_pose;
  std::string reference_frame;
  double lateral_placement_variance;
  double yaw_placement_variance;
  double spawn_timing_variance;
  double spawn_period;
  int randomization_seed;
};

class ConveyorSpawner
{
public:
  ConveyorSpawner(ros::NodeHandle& nh)
    : nh_(nh)
  {
  }

  bool init(XmlRpc::XmlRpcValue& p)
  {
    if(!loadParameters(p))
    {
      return false;
    }

    if(!connectToROS())
    {
      return false;
    }

    srand(params_.randomization_seed);

    start_server_ = nh_.advertiseService(START_SPAWN_SERVICE, &ConveyorSpawner::start, this);
    stop_server_ = nh_.advertiseService(STOP_SPAWN_SERVICE, &ConveyorSpawner::stop, this);
    timer_ = nh_.createTimer(ros::Duration(params_.spawn_period), &ConveyorSpawner::spawnObject, this, false, false);

    return true;
  }

  void run()
  {
    ros::spin();
  }

private:

  bool loadParameters(const XmlRpc::XmlRpcValue& p)
  {
    XmlRpc::XmlRpcValue objects = p;
    try
    {
      // Get the topics
      XmlRpc::XmlRpcValue& topics = objects["topics"];
      for(int i = 0; i < topics.size(); ++i)
      {
        XmlRpc::XmlRpcValue& topic = topics[i];
        params_.model_topics.push_back(static_cast<std::string>(topic));
      }

      // Get the initial pose
      XmlRpc::XmlRpcValue& initial_pose = objects["initial_pose"];

      XmlRpc::XmlRpcValue& position = initial_pose["position"];
      params_.initial_pose.position.x = static_cast<double>(position[0]);
      params_.initial_pose.position.y = static_cast<double>(position[1]);
      params_.initial_pose.position.z = static_cast<double>(position[2]);

      XmlRpc::XmlRpcValue& orientation = initial_pose["orientation"];
      params_.initial_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(static_cast<double>(orientation[0]),
                                                                                 static_cast<double>(orientation[1]),
                                                                                 static_cast<double>(orientation[2]));

      // Get the reference frame
      XmlRpc::XmlRpcValue& frame = objects["reference_frame"];
      params_.reference_frame = static_cast<std::string>(frame);

      // Get the lateral placement variance
      XmlRpc::XmlRpcValue& lpv = objects["lateral_placement_variance"];
      params_.lateral_placement_variance = static_cast<double>(lpv);

      // Get the yaw placement_variance
      XmlRpc::XmlRpcValue& ypv = objects["yaw_placement_variance"];
      params_.yaw_placement_variance = static_cast<double>(ypv) * M_PI / 180.0f;

      // Get the spawn timing variance
      XmlRpc::XmlRpcValue& stv = objects["spawn_timing_variance"];
      params_.spawn_timing_variance = static_cast<double>(stv);

      // Get the spawn timing variance
      XmlRpc::XmlRpcValue& period = objects["spawn_period"];
      params_.spawn_period = static_cast<double>(period);

      // Get the randomization seed
      XmlRpc::XmlRpcValue& seed = objects["randomization_seed"];
      params_.randomization_seed = static_cast<int>(seed);
    }
    catch(XmlRpc::XmlRpcException& ex)
    {
      ROS_ERROR("%s'", ex.getMessage().c_str());
      return false;
    }

    return true;
  }

  bool connectToROS()
  {
    for(const std::string& topic : params_.model_topics)
    {
      std::string description;
      if(!nh_.getParam(topic, description))
      {
        ROS_ERROR("Failed to find '%s' parameter", topic.c_str());
        return false;
      }
      else
      {
        params_.model_descriptions.push_back(description);
      }
    }

    spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SERVICE);
    if(!spawn_client_.waitForExistence(ros::Duration(SRV_TIMEOUT)))
    {
      ROS_ERROR("Timeout waiting for '%s' service", spawn_client_.getService().c_str());
      return false;
    }


    return true;
  }

  bool start(std_srvs::EmptyRequest& req,
             std_srvs::EmptyResponse& res)
  {
    ROS_INFO("Starting conveyor spawner...");
    timer_.start();
    return true;
  }

  bool stop(std_srvs::EmptyRequest& req,
            std_srvs::EmptyResponse& res)
  {
    ROS_INFO("Stopping conveyor spawner...");
    timer_.stop();
    return true;
  }

  void spawnObject(const ros::TimerEvent& e)
  {
    ++object_counter;

    gazebo_msgs::SpawnModel srv;
    srv.request.initial_pose = params_.initial_pose;
    srv.request.model_name = "object_" + std::to_string(object_counter);
    srv.request.reference_frame = params_.reference_frame;
    srv.request.robot_namespace = "gilbreth";

    // Randomize the model to be spawned
    int idx = rand() % params_.model_descriptions.size();
    ROS_INFO("Random index is %d", idx);
    srv.request.model_xml = params_.model_descriptions[idx];

    // Randomize the spawning time and location
    // Create a pseudo-random number
    double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

    // Randomize the object's lateral spawn position
    double& lpv = params_.lateral_placement_variance;
    double lpv_delta = -lpv + 2.0*r*lpv;
    srv.request.initial_pose.position.y += lpv_delta;

    // Randomize the object's spawn yaw angle
    double& ypv = params_.yaw_placement_variance;
    double ypv_delta = -ypv + 2.0*r*ypv;
    tf::Quaternion q;
    tf::quaternionMsgToTF(srv.request.initial_pose.orientation, q);
    tf::Quaternion dq = tf::createQuaternionFromRPY(0.0, 0.0, ypv_delta);
    q *= dq;
    tf::quaternionTFToMsg(q, srv.request.initial_pose.orientation);

    // Randomize the delay in spawning the object
    ros::Duration time_var (r * params_.spawn_timing_variance);
    time_var.sleep();

    // Call the spawn service
    if(!spawn_client_.call(srv))
    {
      ROS_ERROR("Failed to call '%s' service", spawn_client_.getService().c_str());
      --object_counter;
      return;
    }
    else
    {
      if(!srv.response.success)
      {
        ROS_ERROR("%s", static_cast<std::string>(srv.response.status_message).c_str());
        --object_counter;
        return;
      }
    }
  }

  SpawnParameters params_;
  ros::Timer timer_;
  ros::NodeHandle nh_;
  ros::ServiceClient spawn_client_;
  ros::ServiceServer start_server_;
  ros::ServiceServer stop_server_;
  int object_counter = 0;

};

}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conveyor_object_spawner");
  ros::NodeHandle pnh("~"), nh;

  XmlRpc::XmlRpcValue objects;
  if(!pnh.getParam("objects", objects))
  {
    ROS_ERROR("Failed to get objects parameter");
    return 1;
  }

  gilbreth::simulation::ConveyorSpawner spawner (nh);
  if(!spawner.init(objects))
  {
    ROS_ERROR("Failed to initialize spawner");
    return 2;
  }

  spawner.run();

  return 0;
}
