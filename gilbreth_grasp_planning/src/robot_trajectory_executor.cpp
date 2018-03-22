#include <ros/ros.h>
#include <gilbreth_msgs/TargetToolPoses.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/optional.hpp>
#include <algorithm>
#include <gilbreth_gazebo/VacuumGripperState.h>
#include <gilbreth_gazebo/VacuumGripperControl.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/robot_state/conversions.h>
#include <atomic>
#include <Eigen/Core>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>

#define DEG2RAD(x) (M_PI *x)/180.0

static const std::string ROBOT_DESCRIPTION_PARAMETER = "robot_description";
static const std::string TARGET_TOOL_POSES_TOPIC = "gilbreth/target_tool_poses";
static const std::string PLANNING_SERVICE = "plan_kinematic_path";
static const std::string GRIPPER_STATE_TOPIC= "gilbreth/gripper/state";
static const std::string GRIPPER_CONTROL_SERVICE= "gilbreth/gripper/control";
static const std::string CONTROLLER_SERVICE_TOPIC= "controller_manager/switch_controller";

static const double SERVICE_TIMEOUT = 5.0;
static const double EXECUTE_TIMER_PERIOD = 0.1;
static const double ALLOWED_PLANNING_TIME = 1.0;
static const int ALLOWED_PLANNING_ATTEMPTS = 4;
static const double WAIT_ATTACHED_TIME = 2.0f;

static const std::string DEFAULT_PLANNER_ID = "RRTConnectkConfigDefault";
static const std::string WAIT_POSE_NAME = "RAIL_ARM_WAIT";


typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;

using namespace moveit::core;

void curateTrajectory(trajectory_msgs::JointTrajectory& jt)
{
//This is a hack that fixes the issue reported in the link below
// https://github.com/ros-controls/ros_controllers/issues/291
  if(jt.points.empty())
  {
    ROS_ERROR("Trajectory is empty");
  }
  jt.points.front().time_from_start = ros::Duration(0.01);
}

struct RobotControlInfo
{
  std::string controller_name;
  std::string group_name;
};

class TrajExecutor
{
public:
  TrajExecutor()
  {

  }

  ~TrajExecutor()
  {

  }

  bool run()
  {
    if(!init())
    {
      return false;
    }

    setGripper(false);
    moveToWaitPose();

    ros::waitForShutdown();
    return true;
  }

protected:

  bool loadParameters()
  {
    nh_.param<std::string>("rail_group_name",robot_rail_info_.group_name,"robot_rail");
    nh_.param<std::string>("rail_group_name",robot_rail_info_.controller_name,"robot_rail_controller");
    nh_.param<std::string>("arm_group_name",robot_arm_info_.group_name,"robot");
    nh_.param<std::string>("arm_group_name",robot_arm_info_.controller_name,"robot_controller");
    nh_.param<double>("preferred_pick_angle",prefered_pick_angle_,DEG2RAD(90.0));

    return true;
  }

  bool init()
  {

    if(!loadParameters())
    {
      return false;
    }

    // connect to ROS
    target_poses_subs_ = nh_.subscribe(TARGET_TOOL_POSES_TOPIC,1,&TrajExecutor::targetPosesCb,this);
    gripper_state_subs_ = nh_.subscribe(GRIPPER_STATE_TOPIC,1,&TrajExecutor::gripperStateCb, this);
    planning_client_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>(PLANNING_SERVICE);
    gripper_control_client_ = nh_.serviceClient<gilbreth_gazebo::VacuumGripperControl>(GRIPPER_CONTROL_SERVICE);
    controller_switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(CONTROLLER_SERVICE_TOPIC);

    std::vector<ros::ServiceClient> clients = {planning_client_,gripper_control_client_, controller_switch_client_};
    bool connected = std::all_of(clients.begin(),clients.end(),[](ros::ServiceClient& c)
                                 {
      if(!c.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
      {
        ROS_ERROR("Service %s was not found",c.getService().c_str());
        return false;
      }
      return true;
    });

    if(!connected)
    {
      return false;
    }

    // loading robot and moveit
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAMETER));
    robot_model_ = robot_model_loader_->getModel();
    std::vector<std::string> group_names = robot_model_->getJointModelGroupNames();
    for(auto& g :group_names)
    {
      if(g != robot_rail_info_.group_name && g != robot_arm_info_.group_name)
      {
        ROS_WARN("Group %s skipped",g.c_str());
        continue;
      }

      MoveGroupPtr move_group;
      move_group.reset(new moveit::planning_interface::MoveGroupInterface(g));
      move_groups_map_.insert(std::make_pair(g,move_group));
      ROS_INFO("Loaded move group '%s'",g.c_str());
    }

    if(move_groups_map_.empty())
    {
      ROS_ERROR("No valid groups were found");
      return false;
    }

    busy_ = false;
    execution_timer_ = nh_.createTimer(ros::Duration(EXECUTE_TIMER_PERIOD),&TrajExecutor::executionTimerCb,this);

    return true;
  }

  void targetPosesCb(const gilbreth_msgs::TargetToolPosesConstPtr& msg)
  {
    targets_queue_.push_back(*msg);
    ROS_INFO("Received new target");
  }

  void executionTimerCb(const ros::TimerEvent& evnt)
  {
    if(targets_queue_.empty())
    {
      return;
    }

    if(busy_)
    {
      ROS_WARN("Handling an object at the moment, will handle next object when finished with current");
      return;
    }

    busy_ = true;

    // initializing variables
    MoveGroupPtr robot_rail_group = move_groups_map_[robot_rail_info_.group_name];
    MoveGroupPtr robot_arm_group = move_groups_map_[robot_arm_info_.group_name];
    std::atomic<bool> proceed(true);
    Eigen::Quaterniond pick_rotation = Eigen::Quaterniond::Identity() * Eigen::AngleAxisd(prefered_pick_angle_,Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond place_rotation = pick_rotation * Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ());

    auto rotate_pose_funct = [](const geometry_msgs::Pose& pose,const Eigen::Quaterniond& rot) ->geometry_msgs::Pose{

      geometry_msgs::Pose new_pose = pose;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(pose.orientation,q);
      q = q * rot;
      tf::quaternionEigenToMsg(q,new_pose.orientation);
      return std::move(new_pose);
    };

    class ScopeExit
    {
    public:
      ScopeExit(TrajExecutor* obj,std::atomic<bool>* proceed):
        obj_(obj),
        proceed_(proceed)
      {

      }

      ~ScopeExit()
      {

        //obj_->moveToWaitPose();
        if(!(*proceed_)) // a failure took place
        {
          obj_->moveToWaitPose();
        }

        obj_->setGripper(false);
        obj_->activateController(obj_->robot_arm_info_.controller_name,false);
        obj_->activateController(obj_->robot_rail_info_.controller_name,false);
        obj_->monitor_attached_timer_.stop();
        obj_->busy_ = false;
      }

      TrajExecutor* obj_;
      std::atomic<bool>* proceed_;

    };
    ScopeExit sc(this,&proceed);

    activateController(robot_arm_info_.controller_name,false);
    activateController(robot_rail_info_.controller_name,false);
    setGripper(false);

    // =================================================================
    // ===================== Approach move =====================
    // =================================================================
    gilbreth_msgs::TargetToolPoses target_poses = targets_queue_.front();
    targets_queue_.pop_front();

    target_poses.pick_approach.pose = rotate_pose_funct(target_poses.pick_approach.pose,pick_rotation);
    boost::optional<moveit_msgs::RobotTrajectory> approach_traj = planTrajectory(robot_rail_group,target_poses.pick_approach);
    if(!approach_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Approach Motion Plan was found");

    activateController(robot_rail_info_.controller_name,true);
    if(!executeTrajectory(robot_rail_group,approach_traj.get()))
    {
      ROS_WARN("Approach Trajectory execution finished with errors");
      //return;
    }
    activateController(robot_rail_info_.controller_name,false);

    // =================================================================
    // ===================== Pick move =====================
    // =================================================================
    target_poses.pick_pose.pose = rotate_pose_funct(target_poses.pick_pose.pose,pick_rotation);
    boost::optional<moveit_msgs::RobotTrajectory> pick_traj = planTrajectory(robot_arm_group,target_poses.pick_pose);
    if(!pick_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Pick Motion Plan was found");

    // make sure we can reach it
    ros::Duration traj_duration = pick_traj.get().joint_trajectory.points.back().time_from_start;
    ros::Time pick_time = target_poses.pick_pose.header.stamp;
    ros::Time current_time = ros::Time::now();
    if(current_time + traj_duration > pick_time)
    {
      ROS_ERROR("Robot won't make it in time, dismissing object");
      return;
    }

    // executing pick trajectory
    ros::Duration wait_duration = (pick_time - current_time) - traj_duration;
    wait_duration.sleep();
    if(!setGripper(true))
    {
      ROS_ERROR("Gripper control failed");
      return ;
    }

    auto monitor_attached_funct = [&](const ros::TimerEvent& evnt){

      if(gripper_attached_)
      {
        ROS_INFO("Object attached");
        robot_arm_group->stop();
        monitor_attached_timer_.stop();
      }
    };
    monitor_attached_timer_ = nh_.createTimer(ros::Duration(0.1),monitor_attached_funct);

    activateController(robot_arm_info_.controller_name,true);
    if(!executeTrajectory(robot_arm_group,pick_traj.get()))
    {
      ROS_WARN("Pick Trajectory execution finished with errors");
      //return;
    }
    activateController(robot_arm_info_.controller_name,false);
    monitor_attached_timer_.stop();

    // wait until attached
    auto wait_until_attached_funct = [&](double wait_time) -> bool
    {
      ros::Duration wait_period(0.05);
      double time_elapsed = 0.0;
      bool success = false;
      while(time_elapsed < wait_time)
      {
        if(gripper_attached_)
        {
          success = true;
          break;
        }
        wait_period.sleep();
        time_elapsed += wait_period.toSec();
      }
      return success;
    };

    if(wait_until_attached_funct(WAIT_ATTACHED_TIME))
    {
      ROS_INFO("Object attached to gripper");
    }
    else
    {
      ROS_ERROR("Timed out waiting to grab object");
      return;
    }

    // monitoring gripper state
    auto monitor_gripper_funct = [&](const ros::TimerEvent& evnt){

      if(!gripper_attached_)
      {
        proceed = false;
        ROS_ERROR("Object  became detached");
        robot_rail_group->stop();
        robot_arm_group->stop();
        monitor_attached_timer_.stop();

      }
    };
    monitor_attached_timer_ = nh_.createTimer(ros::Duration(0.2),monitor_gripper_funct);

    // =================================================================
    // ===================== Retreat Move =====================
    // =================================================================
    target_poses.pick_retreat.pose = rotate_pose_funct(target_poses.pick_retreat.pose,pick_rotation);

    boost::optional<moveit_msgs::RobotTrajectory> retreat_traj = planTrajectory(robot_arm_group,target_poses.pick_retreat);
    if(!retreat_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Retreat Motion Plan was found");

    activateController(robot_arm_info_.controller_name,true);
    if(!executeTrajectory(robot_arm_group,retreat_traj.get()))
    {
      ROS_WARN("Retreat Trajectory execution finished with errors");
      //return;
    }
    activateController(robot_arm_info_.controller_name,false);

    if(!proceed)
    {
      return;
    }

    // =================================================================
    // ===================== Place Move =====================
    // =================================================================
    target_poses.place_pose.pose = rotate_pose_funct(target_poses.place_pose.pose,place_rotation);

    boost::optional<moveit_msgs::RobotTrajectory> place_traj = planTrajectory(robot_rail_group,target_poses.place_pose,3.14);
    if(!place_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Place Motion Plan was found");

    if(!proceed)
    {
      return;
    }

    activateController(robot_rail_info_.controller_name,true);
    if(!executeTrajectory(robot_rail_group,place_traj.get()))
    {
      ROS_WARN("Place Trajectory execution finished with errors");
      //return;
    }
    activateController(robot_rail_info_.controller_name,false);

    // detaching object
    monitor_attached_timer_.stop();
    if(!setGripper(false))
    {
      ROS_ERROR("Gripper Release failed");
      return ;
    }

  }

  bool moveToWaitPose()
  {

    activateController(robot_arm_info_.controller_name,false);
    activateController(robot_rail_info_.controller_name,true);
    MoveGroupPtr move_group = move_groups_map_[robot_rail_info_.group_name];
    move_group->setNamedTarget(WAIT_POSE_NAME);
    auto error_code = move_group->move();
    return error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }

  bool executeTrajectory(MoveGroupPtr move_group, moveit_msgs::RobotTrajectory& traj)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.planning_time_ = 5.0;
    robotStateToRobotStateMsg(*move_group->getCurrentState(),plan.start_state_);
    plan.trajectory_ = traj;
    auto res = move_group->execute(plan);
    return res.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }

  boost::optional<moveit_msgs::RobotTrajectory> planTrajectory(MoveGroupPtr move_group,geometry_msgs::PoseStamped& pose_st,
                                                               double z_angle_tolerance = 0.1)
  {
    boost::optional<moveit_msgs::Constraints> goal_constraints = createGoalConstraints(
        pose_st,move_group->getName(),z_angle_tolerance);

    if(!goal_constraints.is_initialized())
    {
      return boost::none;
    }

    //ROS_INFO_STREAM("Planning to constraint\n"<<goal_constraints.get()<<std::endl);

    // calling planning service
    moveit_msgs::GetMotionPlan srv;
    srv.request.motion_plan_request.goal_constraints.push_back(goal_constraints.get());
    srv.request.motion_plan_request.group_name = move_group->getName();
    srv.request.motion_plan_request.allowed_planning_time = ALLOWED_PLANNING_TIME;
    srv.request.motion_plan_request.num_planning_attempts = ALLOWED_PLANNING_ATTEMPTS;
    srv.request.motion_plan_request.planner_id = DEFAULT_PLANNER_ID;
    //srv.request.motion_plan_request.start_state = move_group->getCurrentState();

    if(!planning_client_.call(srv))
    {
      ROS_ERROR("Service call '%s' for planning failed",planning_client_.getService().c_str());
      return boost::none;
    }

    if(srv.response.motion_plan_response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR("Motion planning to pose failed");
      return boost::none;
    }

    curateTrajectory(srv.response.motion_plan_response.trajectory.joint_trajectory);
    return srv.response.motion_plan_response.trajectory;
  }

  boost::optional<moveit_msgs::Constraints> createGoalConstraints(geometry_msgs::PoseStamped& pose_st,std::string group_name,
                                                                  double z_angle_tolerance = 0.1)
  {
    boost::optional<moveit_msgs::Constraints> constraints;
    if(move_groups_map_.count(group_name) == 0)
    {
      ROS_ERROR("Invalid group name '%s'",group_name.c_str());
      return boost::none;
    }

    MoveGroupPtr move_group = move_groups_map_[group_name];
    constraints = kinematic_constraints::constructGoalConstraints(move_group->getEndEffectorLink(),pose_st,{0.01,0.01,0.01},
                                                                  {0.01,0.01,z_angle_tolerance});

    return boost::optional<moveit_msgs::Constraints>(constraints);
  }

  void gripperStateCb(const gilbreth_gazebo::VacuumGripperStateConstPtr& msg)
  {
    gripper_attached_ = msg->attached;
  }

  bool setGripper(bool on)
  {
    gilbreth_gazebo::VacuumGripperControl srv;
    srv.request.enable = on;
    if(!gripper_control_client_.call(srv) || !srv.response.success)
    {
      return false;
    }
    return true;
  }

  bool activateController(std::string controller_name, bool activate)
  {
    controller_manager_msgs::SwitchController srv;
    if(activate)
    {
      srv.request.start_controllers.push_back(controller_name);
    }
    else
    {
      srv.request.stop_controllers.push_back(controller_name);

    }
    srv.request.strictness = srv.request.BEST_EFFORT;

    if(!controller_switch_client_.call(srv) || !srv.response.ok)
    {
      ROS_ERROR_STREAM("Failed to "<< (activate ? "activate": "deactivate") <<" controller " << controller_name);
      return false;
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceClient planning_client_;
  ros::ServiceClient gripper_control_client_;
  ros::ServiceClient controller_switch_client_;
  ros::Subscriber gripper_state_subs_;
  ros::Subscriber target_poses_subs_;
  ros::Timer execution_timer_;
  ros::Timer monitor_attached_timer_;

  std::map<std::string,MoveGroupPtr> move_groups_map_;
  RobotControlInfo robot_rail_info_;
  RobotControlInfo robot_arm_info_;
  moveit::core::RobotModelConstPtr robot_model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  double prefered_pick_angle_;

  std::list<gilbreth_msgs::TargetToolPoses> targets_queue_;
  std::atomic<bool> gripper_attached_;
  std::atomic<bool> busy_;


};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"robot_trajectory_executor");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  TrajExecutor texec;
  texec.run();
  return 0;
}
