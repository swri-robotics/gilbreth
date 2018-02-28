#!/usr/bin/env python
## This is the motion planning node for gilbreth project.
## It subscribes from a tool pose publisher(tool_publisher node) and publish the robot trajectories for the robot execution node.
## The trajectories can be visualized in RVIZ. 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

from gilbreth_msgs.msg import TargetToolPoses
from gilbreth_msgs.msg import RobotTrajectories
from moveit_msgs.msg import RobotState
from std_msgs.msg import Duration

# Global variables (don't do this at home)
ARM_GROUP_NAME = 'robot_rail'
MOVEIT_PLANNING_SERVICE = 'plan_kinematic_path'

def waitForMoveGroup(wait_time = 10.0):

  ready = False
  try:
    rospy.wait_for_service(MOVEIT_PLANNING_SERVICE,wait_time)
    ready = True
  except rospy.ROSException as expt:
    pass

  except rospy.ROSInterruptionException as expt:
    pass
    
  return ready


class TrajectoryPlanner():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.timeout = 1.0
        self.max_planning_count = 5
        self.group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)
        self.group.set_planning_time(self.timeout)

        self.robot_trajectories_publisher = rospy.Publisher('/gilbreth/robot_trajectories', RobotTrajectories, queue_size = 20)
        self.tool_poses_sub = rospy.Subscriber('/gilbreth/target_tool_poses', TargetToolPoses, self.tool_poses_callback)

    def tool_poses_callback(self,tool_msg):
        if tool_msg is not None:
            rospy.loginfo("Received new tool pose information")
            self.tool_poses = copy.deepcopy(tool_msg)
            try:
                start_time = time.time()
                if self.motion_planning():
                    self.compute_execution_dur()
                    rospy.loginfo("Run time is %f seconds",(time.time()-start_time))
                else:
                    rospy.logerr("Failed to compute trajectories. Waiting for next objects")
            except rospy.ROSException as e:
                rospy.logerr("Failed to compute trajectories: %s"%e)
            finally:
                self.tool_poses = None
                self.trajectories_msgs = None
        else:
            rospy.logerr("Tool pose is invalid.")

    def compute_trajectory(self):
        self.group.set_planning_time(self.timeout)
        for i in range(self.max_planning_count):
            plan = self.group.plan()
            if len(plan.joint_trajectory.points):
                rospy.loginfo("Motion Plan Success.")
                return plan
            else:
                rospy.logerr("Failed to compute trajectory %d time. Recalculating", i)
        return False

    def motion_planning(self):
        self.trajectories_msgs = RobotTrajectories()

        ## Current ==> Pick Approach.
        rospy.loginfo("======Start Motion Planning: Current ==> Pick Approach.========")
        target = self.tool_poses.pick_approach.pose
        ## Set start and target pose
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target)

        traj = self.compute_trajectory()

        if traj:
            self.trajectories_msgs.cur_to_approach = copy.deepcopy(traj)
        else:
            rospy.logerr("Motion Plan Failed. Waiting for next object.")
            return False

        ## Pick Approach ==> Pick
        rospy.loginfo("=====Start Motion Planning: Pick Approach ==> Pick.=====")
        ## Get start and target pose
        start_state = RobotState()
        start_state.joint_state.name = self.trajectories_msgs.cur_to_approach.joint_trajectory.joint_names
        start_state.joint_state.position = self.trajectories_msgs.cur_to_approach.joint_trajectory.points[-1].positions
        self.group.set_start_state(start_state)
        target = self.tool_poses.pick_pose.pose
        self.group.set_pose_target(target)

        traj = self.compute_trajectory()
        if traj:
            self.trajectories_msgs.approach_to_pick =  copy.deepcopy(traj)
        else:
            rospy.logerr("Motion Plan Failed. Waiting for next object.")
            return False
        

        ## Pick  ==> Pick retreat
        rospy.loginfo("=====Start Motion Planning: Pick ==> Pick retreat.=====")
        ## Get start and target pose
        start_state = RobotState()
        start_state.joint_state.name = self.trajectories_msgs.approach_to_pick.joint_trajectory.joint_names
        start_state.joint_state.position = self.trajectories_msgs.approach_to_pick.joint_trajectory.points[-1].positions
        target = self.tool_poses.pick_retreat.pose
        self.group.set_start_state(start_state)
        self.group.set_pose_target(target)

        traj = self.compute_trajectory()
        if traj:
            self.trajectories_msgs.pick_to_retreat =  copy.deepcopy(traj)
        else:
            rospy.logerr("Motion Plan Failed. Waiting for next object.")
            return False
        
        ## Pick retreat ==> Place
        rospy.loginfo("=====Start Motion Planning: Retreat ==> Place.=====")
        ## Get start and target pose
        start_state = RobotState()
        start_state.joint_state.name = self.trajectories_msgs.pick_to_retreat.joint_trajectory.joint_names
        start_state.joint_state.position = self.trajectories_msgs.pick_to_retreat.joint_trajectory.points[-1].positions
        target = self.tool_poses.place_pose.pose
        self.group.set_start_state(start_state)
        self.group.set_pose_target(target)

        traj = self.compute_trajectory()
        if traj:
            self.trajectories_msgs.retreat_to_place =  copy.deepcopy(traj)
        else:
            rospy.logerr("Motion Plan Failed. Waiting for next object.")
            return False

        self.trajectories_msgs.header.stamp = rospy.get_rostime()
        rospy.loginfo("************Successfully plannign all trajectories***********")
        return True

    def compute_execution_dur(self):
        ## Get planned robot execution time
        approach_duration = self.trajectories_msgs.cur_to_approach.joint_trajectory.points[-1].time_from_start
        pick_duration = self.trajectories_msgs.approach_to_pick.joint_trajectory.points[-1].time_from_start
        retreat_duration = self.trajectories_msgs.pick_to_retreat.joint_trajectory.points[-1].time_from_start
        place_duration = self.trajectories_msgs.retreat_to_place.joint_trajectory.points[-1].time_from_start
    
        time_dur = [None] * 4
        time_dur[0] = approach_duration
        time_dur[1] = pick_duration
        time_dur[2] = retreat_duration
        time_dur[3] = place_duration

        self.trajectories_msgs.execution_duration = time_dur
        
        rospy.loginfo("Execution dur:%f,%f,%f,%f\n",approach_duration.to_sec(),pick_duration.to_sec(),retreat_duration.to_sec(),place_duration.to_sec())
    
        ## Get deadline time for reaching picking pose
        self.trajectories_msgs.pick_deadline = copy.deepcopy(self.tool_poses.pick_pose.header.stamp)

        ## Publish robot trajectories messages 
        self.robot_trajectories_publisher.publish(self.trajectories_msgs)
        rospy.loginfo("Publishing robot trajectories to /gilbreth/robot_trajectories")
if __name__=='__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('trajectory_planner')

        rospy.loginfo("Waiting for move group")
        if waitForMoveGroup():
          rospy.loginfo("Found move group node, proceeding")
        else:
          rospy.logerr("Timed out waiting for move group node, exiting ...")
          sys.exit(-1)
    
        traj_planner = TrajectoryPlanner()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

