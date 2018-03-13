#ifndef GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
#define GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <urdf/model.h>
#include <XmlRpcValue.h>
#include <gazebo_msgs/ModelStates.h>

namespace gilbreth
{
namespace simulation
{

struct ObjectParameters
{
  std::string name;
  std::string mesh_resource;
  geometry_msgs::Pose initial_pose;
  double lateral_placement_variance;
  double yaw_placement_variance;
  double spawn_timing_variance;
};

struct SpawnParameters
{
  std::string reference_frame;
  double spawn_period;
  int randomization_seed;
  int max_objects = 40;                   /** @brief maximum number of objects to spawn, objects will then be recycled */
  std::vector<ObjectParameters> objects;
};

class ConveyorSpawner
{
public:
  ConveyorSpawner(ros::NodeHandle& nh);


  bool init(XmlRpc::XmlRpcValue& p);


  void run();


private:

  bool loadSpawnParameters(const XmlRpc::XmlRpcValue& p,
                           SpawnParameters& spawn_params) const;


  bool loadObjectParameters(const XmlRpc::XmlRpcValue& object,
                            ObjectParameters& object_params) const;

  bool connectToROS();


  bool start(std_srvs::EmptyRequest& req,
             std_srvs::EmptyResponse& res);


  bool stop(std_srvs::EmptyRequest& req,
            std_srvs::EmptyResponse& res);


  void spawnObjectTimerCb(const ros::TimerEvent& e);
  void spawnObject();
  void recirculateObject();

  void disposedObjectsCallback(const gazebo_msgs::ModelStatesConstPtr& msg);


  int object_counter_ = 0;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber disposed_objs_subs_;
  ros::ServiceClient spawn_client_;
  ros::ServiceClient set_state_client_;
  ros::ServiceServer start_server_;
  ros::ServiceServer stop_server_;
  ros::Timer timer_;
  SpawnParameters params_;
  std::vector<std::string> inactive_objects_ids_;
  std::map<std::string,std::string> spawned_objects_map_; /** @brief mapping between object names and models used <obj_id, obj_name>*/

};

} // namespace simulation
} // namespace gilbreth

#endif // GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
