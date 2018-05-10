#include <ros/ros.h>
#include <ros/callback_queue.h>
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
#include <gilbreth_msgs/RobotTrajectories.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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
static const double MAX_STATE_DISTANCE = 0.25f;
static const std::string DEFAULT_PLANNER_ID = "RRTConnectkConfigDefault";


typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef moveit::planning_interface::MoveGroupInterface::Plan RobotPlan;

using namespace moveit::core;

void curateTrajectory(trajectory_msgs::JointTrajectory& jt)
{
//This is a hack that fixes the issue reported in the link below
// https://github.com/ros-controls/ros_controllers/issues/291
  if(jt.points.empty())
  {
    ROS_ERROR("Trajectory is empty");
  }

  double time_incrm = 0.01;
  double prev_time = 0.0;
  int count = 0;
  for(auto& jp :jt.points)
  {
    if(jp.time_from_start.toSec() <= prev_time)
    {
      count++;
      prev_time = count*time_incrm;
      jp.time_from_start = ros::Duration(prev_time);
      continue;
    }
    break;
  }
}


std::map<std::string,double> createJointsMap(const std::vector<std::string>& keys,
                                             const std::vector<double>& vals)
{
  std::map<std::string,double> m;
  for(std::size_t i = 0; i < keys.size() ; i++)
  {
    m.insert(std::make_pair(keys[i],vals[i]));
  }
  return std::move(m);
};

void setToLastPoint(const moveit_msgs::RobotTrajectory& traj,RobotState& rs)
{
  std::map<std::string,double> jv;
  jv = createJointsMap(traj.joint_trajectory.joint_names,
                           traj.joint_trajectory.points.back().positions);
  //rs.setToDefaultValues();
  rs.setVariablePositions(jv);
};

struct RobotControlInfo
{
  std::string controller_name;
  std::string group_name;
  std::string wait_pose_name;
};

struct TaskInfo
{
  std::string name;
  RobotPlan trajectory_plan;
  double delay;
  ros::Time arrival_time;
  RobotControlInfo robot_info;
};

struct RobotTasks
{
  gilbreth_msgs::TargetToolPoses target_poses;
  std::vector<TaskInfo> trajectory_list;
};





class TrajExecutor
{
public:
  TrajExecutor():
    traj_exec_nh_(),
    traj_exec_spinner_(2,&traj_callback_queue_)
  {
    traj_exec_nh_.setCallbackQueue(&traj_callback_queue_);
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
    ros::NodeHandle ph("~");
    ph.param<std::string>("rail_group_name",robot_rail_info_.group_name,"robot_rail");
    ph.param<std::string>("rail_group_name",robot_rail_info_.controller_name,"robot_rail_controller");
    ph.param<std::string>("rail_group_wait_pose",robot_rail_info_.wait_pose_name,"RAIL_ARM_WAIT");

    ph.param<std::string>("arm_group_name",robot_arm_info_.group_name,"robot");
    ph.param<std::string>("arm_group_name",robot_arm_info_.controller_name,"robot_controller");
    ph.param<std::string>("arm_group_wait_pose",robot_arm_info_.wait_pose_name,"ARM_WAIT");

    ph.param<double>("preferred_pick_angle",prefered_pick_angle_,DEG2RAD(90.0));
    ph.param<int>("cartesian_num_points",cartesian_num_points_,40);
    ph.param<double>("cartesian_eef_max_step",cartesian_eef_max_step_,0.1);
    ph.param<double>("cartesian_jump_threshold",cartesian_jump_threshold_,2.0);
    ph.param<double>("cartesian_dynamics_scaling",cartesian_dynamics_scaling_,0.2);


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
    //execution_timer_ = traj_exec_nh_.createTimer(ros::Duration(EXECUTE_TIMER_PERIOD),&TrajExecutor::executionTimerCb,this);
    execution_timer_ = traj_exec_nh_.createTimer(ros::Duration(EXECUTE_TIMER_PERIOD),&TrajExecutor::executeTrajectoryQueue,this);
    traj_exec_spinner_.start();

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
      ScopeExit(TrajExecutor* obj,std::atomic<bool>* success):
        obj_(obj),
        success_(success)
      {

      }

      ~ScopeExit()
      {

        if(!(*success_)) // a failure took place
        {
          obj_->moveToWaitPose();
        }
        else
        {
          obj_->moveToWaitPose(true);
          ros::Duration(2.0).sleep();
        }

        obj_->setGripper(false);
        obj_->activateController(obj_->robot_arm_info_.controller_name,false);
        obj_->activateController(obj_->robot_rail_info_.controller_name,false);
        obj_->monitor_attached_timer_.stop();
        obj_->busy_ = false;
      }

      TrajExecutor* obj_;
      std::atomic<bool>* success_;

    };
    ScopeExit sc(this,&proceed);

    activateController(robot_arm_info_.controller_name,false);
    activateController(robot_rail_info_.controller_name,false);
    setGripper(false);
    robot_rail_group->stop();
    robot_arm_group->stop();

    // =================================================================
    // ===================== Approach move =====================
    // =================================================================
    gilbreth_msgs::TargetToolPoses target_poses = targets_queue_.front();
    targets_queue_.pop_front();

    target_poses.pick_approach.pose = rotate_pose_funct(target_poses.pick_approach.pose,pick_rotation);
    boost::optional<RobotPlan> approach_traj = planJointTrajectory(robot_rail_group->getCurrentState(),
                                                                                 robot_rail_group,target_poses.pick_approach);
    if(!approach_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Approach Motion Plan was found");

    if(!executeTrajectory(robot_rail_info_,approach_traj.get()))
    {
      ROS_ERROR("Approach Trajectory execution finished with errors");
      proceed = false;
      return;
    }

    // =================================================================
    // ===================== Pick move =====================
    // =================================================================
    target_poses.pick_pose.pose = rotate_pose_funct(target_poses.pick_pose.pose,pick_rotation);
    boost::optional<RobotPlan> pick_traj = planJointTrajectory(robot_arm_group->getCurrentState(),
                                                                             robot_arm_group,target_poses.pick_pose);
    if(!pick_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Pick Motion Plan was found");

    // make sure we can reach it
    ros::Duration traj_duration = pick_traj.get().trajectory_.joint_trajectory.points.back().time_from_start;
    ros::Time pick_time = target_poses.pick_pose.header.stamp;
    ros::Time current_time = ros::Time::now();
    if(current_time + traj_duration > pick_time)
    {
      double time_available = (pick_time - current_time).toSec();
      ROS_ERROR("Robot won't make it in time, dismissing object. Pick traj duration: %f > time available: %f",
                traj_duration.toSec(),time_available);
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
    monitor_attached_timer_ = traj_exec_nh_.createTimer(ros::Duration(0.1),monitor_attached_funct);

    if(!executeTrajectory(robot_arm_info_,pick_traj.get()))
    {
      ROS_ERROR("Pick Trajectory execution finished with errors");
      proceed = false;
      return;
    }
    monitor_attached_timer_.stop();

    // wait until attached
    auto wait_until_attached_funct = [&](double wait_time) -> bool
    {
      ros::Duration wait_period(0.01);
      double time_elapsed = 0.0;
      bool success = false;
      std::cout<<std::endl;
      while(time_elapsed < wait_time)
      {
        if(gripper_attached_)
        {
          success = true;
          break;
        }
        wait_period.sleep();
        time_elapsed += wait_period.toSec();
        std::cout<<"\rWaiting until attached, time elapsed: "<<time_elapsed<<std::flush;
      }
      std::cout<<std::endl;
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
    monitor_attached_timer_ = traj_exec_nh_.createTimer(ros::Duration(0.2),monitor_gripper_funct);

    // =================================================================
    // ===================== Retreat Move =====================
    // =================================================================
    target_poses.pick_retreat.pose = rotate_pose_funct(target_poses.pick_retreat.pose,pick_rotation);

    boost::optional<RobotPlan> retreat_traj = planJointTrajectory(robot_arm_group->getCurrentState()
                                                                                ,robot_arm_group,target_poses.pick_retreat);
    if(!retreat_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Retreat Motion Plan was found");

    if(!executeTrajectory(robot_arm_info_,retreat_traj.get()))
    {
      ROS_ERROR("Retreat Trajectory execution finished with errors");
      proceed = false;
      return;
    }

    if(!proceed)
    {
      return;
    }

    // =================================================================
    // ===================== Place Move =====================
    // =================================================================
    target_poses.place_pose.pose = rotate_pose_funct(target_poses.place_pose.pose,place_rotation);

    boost::optional<RobotPlan> place_traj = planJointTrajectory(robot_rail_group->getCurrentState(),
                                                                              robot_rail_group,target_poses.place_pose,3.14);
    if(!place_traj.is_initialized())
    {
      return;
    }
    ROS_INFO("Place Motion Plan was found");

    if(!proceed)
    {
      return;
    }

    if(!executeTrajectory(robot_rail_info_,place_traj.get()))
    {
      ROS_ERROR("Place Trajectory execution finished with errors");
      proceed = false;
      return;
    }

    // detaching object
    monitor_attached_timer_.stop();
    if(!setGripper(false))
    {
      ROS_ERROR("Gripper Release failed");
      return ;
    }

  }

  bool moveToWaitPose(bool async = false)
  {

    activateController(robot_arm_info_.controller_name,false);
    activateController(robot_rail_info_.controller_name,true);
    MoveGroupPtr move_group = move_groups_map_[robot_rail_info_.group_name];
    move_group->setNamedTarget(robot_rail_info_.wait_pose_name);
    moveit_msgs::MoveItErrorCodes error_code;
    if(async)
    {
      error_code = move_group->asyncMove();
    }
    else
    {
      error_code = move_group->move();
    }

    return error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }

  bool executeTrajectory(const RobotControlInfo& robot_info, moveit_msgs::RobotTrajectory& traj)
  {
    auto fuse_vectors = [](const std::vector<std::string>& keys, const std::vector<double>& vals) -> std::map<std::string,double>
    {
      std::map<std::string,double> m;
      for(std::size_t i = 0; i < keys.size() ; i++)
      {
        m.insert(std::make_pair(keys[i],vals[i]));
      }
      return std::move(m);
    };

    activateController(robot_info.controller_name,true);
    MoveGroupPtr move_group = move_groups_map_[robot_info.group_name];
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.planning_time_ = 5.0;
    plan.trajectory_ = traj;

    RobotStatePtr robot_st(new RobotState(robot_model_));
    std::map<std::string,double> start_joints = fuse_vectors(traj.joint_trajectory.joint_names,
                                                             traj.joint_trajectory.points.front().positions);
    robot_st->setToDefaultValues();
    robot_st->setVariablePositions(start_joints);
    robotStateToRobotStateMsg(*robot_st,plan.start_state_);

    auto res = move_group->execute(plan);
    activateController(robot_info.controller_name,false);
    return res.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }

  bool executeTrajectory(const RobotControlInfo& robot_info, const RobotPlan& rp)
  {

    activateController(robot_info.controller_name,true);
    MoveGroupPtr move_group = move_groups_map_[robot_info.group_name];
    auto res = move_group->execute(rp);
    activateController(robot_info.controller_name,false);

    bool success = res.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
    if(!success) // check if at least it is close enough
    {
      ROS_WARN("Trajectory didn't run to completion, checking if close enough");
      RobotStatePtr rs_final(new RobotState(robot_model_));
      robotStateMsgToRobotState(rp.start_state_,*rs_final);
      setToLastPoint(rp.trajectory_,*rs_final);

      RobotStatePtr rs_current = move_group->getCurrentState();
      if(rs_current)
      {
        std::vector<double> jf, jc;
        rs_current->copyJointGroupPositions(move_group->getName(),jc);
        rs_final->copyJointGroupPositions(move_group->getName(),jf);

        success = std::equal(jf.begin(),jf.end(),jc.data(),[](const double& a,const double &b){
          return std::abs(a - b) < MAX_STATE_DISTANCE;
        });
      }
    }

    return success;
  }

  std::vector<TaskInfo> planTaskTrajectories(const gilbreth_msgs::TargetToolPoses& target_poses)
  {
    using namespace moveit::core;



    RobotStatePtr robot_st(new RobotState(robot_model_));
    std::vector<TaskInfo> task_info;
    TaskInfo task_traj;

    MoveGroupPtr robot_arm_group = move_groups_map_[robot_arm_info_.group_name];
    MoveGroupPtr robot_rail_group = move_groups_map_[robot_rail_info_.group_name];

    // ========================================================
    // planing start to approach
    std::map<std::string,double> joint_vals = robot_rail_group->getNamedTargetValues(robot_rail_info_.wait_pose_name);
    robot_st->setToDefaultValues();
    robot_st->setVariablePositions(joint_vals);

    //auto start_to_approach_traj = planJointTrajectory(robot_st,robot_rail_group,target_poses.pick_approach,3.14);
    auto start_to_approach_traj = planCartesianTrajectory(robot_st,robot_rail_group,target_poses.pick_approach);
    if(!start_to_approach_traj.is_initialized())
    {
      ROS_ERROR("Planning from start to approach failed");
      return {};
    }
    task_traj = TaskInfo();
    task_traj.trajectory_plan = std::move(start_to_approach_traj.get());
    task_traj.name = "approach";
    task_traj.robot_info = robot_rail_info_;
    task_traj.delay = -1.0; // no delay
    task_info.push_back(task_traj);

    // ========================================================
    // planing from approach to pick
    setToLastPoint(task_info.back().trajectory_plan.trajectory_,*robot_st);
    //auto approach_to_pick_traj = planJointTrajectory(robot_st,robot_arm_group,target_poses.pick_pose,0.1);
    auto approach_to_pick_traj = planCartesianTrajectory(robot_st,robot_arm_group,target_poses.pick_pose);
    if(!approach_to_pick_traj.is_initialized())
    {
      ROS_ERROR("Planning from approach to pick failed");
      return {};
    }
    task_traj = TaskInfo();
    task_traj.trajectory_plan = std::move(approach_to_pick_traj.get());
    task_traj.name = "pick";
    task_traj.robot_info = robot_arm_info_;
    task_traj.delay = -1.0; // no delay
    task_info.push_back(task_traj);

    // ========================================================
    // planing from pick to retreat
    setToLastPoint(task_info.back().trajectory_plan.trajectory_,*robot_st);
    //auto pick_to_retreat_traj = planJointTrajectory(robot_st,robot_arm_group,target_poses.pick_retreat,0.1);
    auto pick_to_retreat_traj = planCartesianTrajectory(robot_st,robot_arm_group,target_poses.pick_retreat);
    if(!pick_to_retreat_traj.is_initialized())
    {
      ROS_ERROR("Planning from pick to retreat failed");
      return {};
    }
    task_traj = TaskInfo();
    task_traj.trajectory_plan = std::move(pick_to_retreat_traj.get());
    task_traj.name = "retreat";
    task_traj.robot_info = robot_arm_info_;
    task_traj.delay = -1.0; // no delay
    task_info.push_back(task_traj);

    // ========================================================
    // planing from retreat to place
    setToLastPoint(task_info.back().trajectory_plan.trajectory_,*robot_st);
    auto retreat_to_place_traj = planJointTrajectory(robot_st,robot_rail_group,target_poses.place_pose,3.14);
    //auto retreat_to_place_traj = planCartesianTrajectory(robot_st,robot_rail_group,target_poses.place_pose);

    if(!retreat_to_place_traj.is_initialized())
    {
      ROS_ERROR("Planning from retreat to place failed");
      return {};
    }
    task_traj = TaskInfo();
    task_traj.trajectory_plan = std::move(retreat_to_place_traj.get());
    task_traj.name = "place";
    task_traj.robot_info = robot_rail_info_;
    task_traj.delay = -1.0; // no delay
    task_info.push_back(task_traj);


    // ========================================================
    // planing from place to wait
    setToLastPoint(task_info.back().trajectory_plan.trajectory_,*robot_st);
    robot_rail_group->setNamedTarget(robot_rail_info_.wait_pose_name);
    robot_rail_group->setStartState(*robot_st);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(robot_rail_group->plan(plan).val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR("Planning from place to place failed");
      return {};
    }
    task_traj = TaskInfo();
    task_traj.trajectory_plan = std::move(plan);
    task_traj.name = "return";
    task_traj.robot_info = robot_rail_info_;
    task_traj.delay = -1.0; // no delay
    task_info.push_back(task_traj);


    return task_info;
  }

  void executeTrajectoryQueue(const ros::TimerEvent& evnt)
  {
    /**
     * DON'T USE THIS
     * TODO: improve or remove
     * Moveit can't seem to plan and move at the same time.  This function might produce such scenario.
     */
    static const int PICK_TRAJ_INDEX = 1;
    std::atomic<bool> succeeded(true);

    // allows conducting clean up actions before exiting
    class ScopeExit
    {
    public:
      ScopeExit(TrajExecutor* obj,std::atomic<bool>* succeeded):
        obj_(obj),
        succeeded_(succeeded)
      {

      }

      ~ScopeExit()
      {

        //obj_->moveToWaitPose();

        obj_->setGripper(false);
        obj_->monitor_attached_timer_.stop();
        if(!(*succeeded_)) // a failure took place
        {
          obj_->moveToWaitPose(false);
        }

        obj_->activateController(obj_->robot_arm_info_.controller_name,false);
        obj_->activateController(obj_->robot_rail_info_.controller_name,false);
        obj_->busy_ = false;
      }

      TrajExecutor* obj_;
      std::atomic<bool>* succeeded_;
    };

    // check buffer
    if(targets_queue_.empty())
    {
      return;
    }

    if(busy_)
    {
      ROS_WARN("Busy handling another target at the moment...");
      return;
    }

    // initializing scope exit variable
    ScopeExit sc(this,&succeeded); // will exit gracefully when exiting scope
    busy_ = true;

    ROS_INFO("Planning all target trajectories");
    RobotTasks robot_traj;
    robot_traj.target_poses = std::move(targets_queue_.front());
    robot_traj.trajectory_list = std::move(planTaskTrajectories(robot_traj.target_poses));
    targets_queue_.pop_front();
    if(robot_traj.trajectory_list.empty())
    {
      return;
    }
    ROS_INFO("Computed target trajectories, proceeding with execution");


    // clear all active componets
    activateController(robot_arm_info_.controller_name,false);
    activateController(robot_rail_info_.controller_name,false);
    setGripper(false);


    // ===================================================
    // Pre checks
    // make sure we can reach it
    auto compute_traj_time = [](const std::vector<TaskInfo>& task_trajs, int count) -> ros::Duration
    {
      double total_time = 0.0;
      for(std::size_t i = 0; i < count; i++)
      {
        const TaskInfo& task_info = task_trajs[i];
        ros::Duration traj_duration = task_info.trajectory_plan.trajectory_.joint_trajectory.points.back().time_from_start;
        total_time += traj_duration.toSec();
      }
      return std::move(ros::Duration(total_time));
    };

    geometry_msgs::PoseStamped& pick_pose = robot_traj.target_poses.pick_pose;
    ros::Duration traj_duration = compute_traj_time(robot_traj.trajectory_list,PICK_TRAJ_INDEX+1);
    ros::Time pick_time(pick_pose.header.stamp.toSec());
    ros::Time current_time = ros::Time::now();
    if(current_time + traj_duration > pick_time)
    {
      double time_available = (pick_time - current_time).toSec();
      ROS_ERROR("Robot won't make it in time, dismissing object. Pick traj duration: %f > time available: %f",
                traj_duration.toSec(),time_available);
      return;
    }

    // =========================================
    // declaring helper functions

    // issues a stop when contact is first made made
    MoveGroupPtr robot_arm_group = move_groups_map_[robot_arm_info_.group_name];
    MoveGroupPtr robot_rail_group = move_groups_map_[robot_rail_info_.group_name];
    auto monitor_attached_funct = [&](const ros::TimerEvent& evnt){

      //std::cout<<"Checking Attached\r"<<std::flush;
      if(gripper_attached_)
      {
        ROS_INFO("Object attached");
        robot_arm_group->stop();
        robot_rail_group->stop();
        monitor_attached_timer_.stop();
      }
    };

    // used to wait for contact
    auto wait_until_attached_funct = [&](double wait_time) -> bool
    {
      ros::Duration wait_period(0.1);
      double time_elapsed = 0.0;
      bool success = false;
      std::cout<<std::endl;
      while(time_elapsed < wait_time)
      {
        if(gripper_attached_)
        {
          success = true;
          break;
        }
        wait_period.sleep();
        time_elapsed += wait_period.toSec();
        std::cout<<"\rWaiting until attached, time elapsed: "<<time_elapsed<<std::flush;
      }
      robot_arm_group->stop();
      std::cout<<std::endl;
      return success;
    };

    // used to monitor that the gripper still has an attached object
    auto monitor_gripper_funct = [&](const ros::TimerEvent& evnt){
      if(!gripper_attached_)
      {
        succeeded = false;
        ROS_ERROR("Object  became detached");
        robot_rail_group->stop();
        robot_arm_group->stop();
        monitor_attached_timer_.stop();
      }
    };


    // ===================================================
    // Trajectory Execution
    for(auto& traj : robot_traj.trajectory_list)
    {
      if(traj.name == "pick")
      {
        if(!setGripper(true))
        {
          ROS_ERROR("Gripper control failed");
          return ;
        }

        // will stop execution when contact is first made
        monitor_attached_timer_ = nh_.createTimer(ros::Duration(0.02),monitor_attached_funct);

        // wait until object gets to pick position
        ros::Duration wait_duration = (pick_time - current_time) - traj_duration;
        ROS_INFO("Waiting %f seconds for object to arrive to pick position",wait_duration.toSec());
        wait_duration.sleep();
      }

      ROS_INFO("Moving to %s",traj.name.c_str());
      if(!executeTrajectory(traj.robot_info,traj.trajectory_plan))
      {
        ROS_ERROR("%s Trajectory execution finished with errors",traj.name.c_str());
        succeeded = false;
        return;
      }

      if(traj.name == "pick")
      {
        // waiting to ensure object is attached
        ROS_INFO("Waiting to make contact");
        if(!wait_until_attached_funct(WAIT_ATTACHED_TIME))
        {
          ROS_ERROR("Timed out waiting to grab object");
          succeeded = false;
          return;
        }

        ROS_INFO("Object attached to gripper");

        // will stop if contact is broken
        monitor_attached_timer_ = nh_.createTimer(ros::Duration(0.05),monitor_gripper_funct);
      }

      if(traj.name == "place")
      {
        monitor_attached_timer_.stop();
        if(!setGripper(false))
        {
          ROS_ERROR("Gripper control failed");
          return ;
        }
      }

      if(!succeeded)
      {
        return;
      }
    }

  }



  boost::optional<RobotPlan> planJointTrajectory(RobotStateConstPtr start_state,MoveGroupPtr move_group,const geometry_msgs::PoseStamped& pose_st,
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
    robot_state::robotStateToRobotStateMsg(*start_state,srv.request.motion_plan_request.start_state,true);

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
    RobotPlan plan;
    plan.planning_time_ = 0.0; // doesn't matter
    plan.trajectory_ = srv.response.motion_plan_response.trajectory;
    plan.start_state_ = srv.response.motion_plan_response.trajectory_start;
    return plan;
  }

  boost::optional<RobotPlan> planCartesianTrajectory(RobotStatePtr start_state,MoveGroupPtr move_group,
                                                     const geometry_msgs::PoseStamped& pose_st)
  {
    start_state->updateLinkTransforms();
    Eigen::Affine3d start_tool_pose = start_state->getFrameTransform(move_group->getEndEffectorLink());
    Eigen::Affine3d final_tool_pose;
    tf::poseMsgToEigen(pose_st.pose,final_tool_pose);

    auto interpolate = [](const Eigen::Affine3d& p0,const Eigen::Affine3d& pf,
        int num_poses) -> std::vector<geometry_msgs::Pose>
    {
      std::vector<geometry_msgs::Pose> poses;
      double increment = 1.0/double(num_poses-1);
      Eigen::Vector3d t0 = p0.translation();
      Eigen::Vector3d tf = pf.translation();
      Eigen::Quaterniond q0(p0.rotation());
      Eigen::Quaterniond qf(pf.rotation());

      Eigen::Vector3d t_new;
      Eigen::Quaterniond q_new;
      Eigen::Affine3d p_new;
      geometry_msgs::Pose pose_msg;
      for(std::size_t i = 0; i < num_poses; i++)
      {
        double t = i*increment;
        t_new = t0 + (tf - t0) * (t);
        q_new = q0.slerp(t,qf);
        p_new = Eigen::Translation3d(t_new) * q_new;
        tf::poseEigenToMsg(p_new,pose_msg);
        poses.push_back(pose_msg);
      }
      return poses;
    };

    // planning in tool space
    RobotPlan plan;
    plan.planning_time_ = 0.0f;
    robotStateToRobotStateMsg(*start_state,plan.start_state_);
    std::vector<geometry_msgs::Pose> waypoints = interpolate(start_tool_pose,final_tool_pose,cartesian_num_points_);
    move_group->setPoseReferenceFrame(pose_st.header.frame_id);
    move_group->setStartState(*start_state);
    double res = move_group->computeCartesianPath(waypoints,cartesian_eef_max_step_, cartesian_jump_threshold_,plan.trajectory_);
    if(res<0.999)
    {
      ROS_ERROR("Cartesian plan only solved %f of the %lu points",100.0 * res,waypoints.size());
      return boost::none;
    }

    // computing time stamps
    robot_trajectory::RobotTrajectory robot_traj(robot_model_,move_group->getName());
    robot_traj.setRobotTrajectoryMsg(*start_state,plan.trajectory_);
    trajectory_processing::IterativeParabolicTimeParameterization tp;
    if(!tp.computeTimeStamps(robot_traj,cartesian_dynamics_scaling_,cartesian_dynamics_scaling_))
    {
      ROS_ERROR("Cartesian trajectory time parameterization failed");
      return boost::none;
    }
    plan.trajectory_ = moveit_msgs::RobotTrajectory();
    robot_traj.getRobotTrajectoryMsg(plan.trajectory_);

    curateTrajectory(plan.trajectory_.joint_trajectory);
    //ROS_INFO_STREAM(plan.trajectory_);
    return plan;
    //return boost::none;
  }

  boost::optional<moveit_msgs::Constraints> createGoalConstraints(const geometry_msgs::PoseStamped& pose_st,std::string group_name,
                                                                  double z_angle_tolerance = 0.1)
  {
    boost::optional<moveit_msgs::Constraints> constraints;
    if(move_groups_map_.count(group_name) == 0)
    {
      ROS_ERROR("Invalid group name '%s'",group_name.c_str());
      return boost::none;
    }

    MoveGroupPtr move_group = move_groups_map_[group_name];
    constraints = kinematic_constraints::constructGoalConstraints(move_group->getEndEffectorLink(),pose_st,{0.005,0.005,0.005},
                                                                  {0.1,0.1,z_angle_tolerance});

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

  // execute trajectories in its own callback queue to avoid blocking
  ros::NodeHandle traj_exec_nh_;
  ros::CallbackQueue traj_callback_queue_;
  ros::AsyncSpinner traj_exec_spinner_;

  ros::ServiceClient planning_client_;
  ros::ServiceClient gripper_control_client_;
  ros::ServiceClient controller_switch_client_;
  ros::Subscriber gripper_state_subs_;
  ros::Subscriber target_poses_subs_;
  ros::Timer execution_timer_;
  ros::Timer monitor_attached_timer_;

  std::map<std::string,MoveGroupPtr> move_groups_map_;
  moveit::core::RobotModelConstPtr robot_model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  // ros parameters
  RobotControlInfo robot_rail_info_;
  RobotControlInfo robot_arm_info_;
  double prefered_pick_angle_;
  int cartesian_num_points_;
  double cartesian_eef_max_step_;
  double cartesian_jump_threshold_; // max change in the configuration space
  double cartesian_dynamics_scaling_; // used for time parameterization

  std::list<gilbreth_msgs::TargetToolPoses> targets_queue_;
  //std::list<RobotTasks> robot_tasks_queue_;
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
