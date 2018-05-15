#!/usr/bin/env python
## A node executes the robot. 
## Including Gripper control.

import sys
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import random

from gilbreth_msgs.msg import RobotTrajectories
from gilbreth_gazebo.msg import VacuumGripperState
from gilbreth_gazebo.srv import VacuumGripperControl
from geometry_msgs.msg import Pose
from kitchen.iterutils import iterate

ROBOT_TRAJ_TOPIC='gilbreth/robot_trajectories'
GRIPPER_STATE_TOPIC='gilbreth/gripper/state'
GRIPPER_SERIVE_TOPIC='gilbreth/gripper/control'
ARM_GROUP_NAME = 'robot_rail'
MOVEIT_PLANNING_SERVICE = 'plan_kinematic_path'
HOME_JOINT_POSE = 'robot_rail_home'
NUM_GOAL_POSES = 1
JOINT_RAND_FACTOR = [0.5] + [0.4]*6

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

def curateTrajectory(traj):
  # This is a hack that fixes the issue reported in the link below
  # https://github.com/ros-controls/ros_controllers/issues/291
  traj.joint_trajectory.points[0].time_from_start = rospy.Duration(0.01)
  return traj    
            
class RobotExecution:

    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group.set_planning_time(2.0)

    def moveRobot(self):      
  
      ## Motion Planning    
      robot_traj = self.group.plan()
      
  
      ## if trajectory is valid, move the robot
      if robot_traj:
          
          robot_traj = curateTrajectory(robot_traj)
          rospy.loginfo("Motion Planing Succeeded")
          if self.group.execute(robot_traj):
            rospy.loginfo("Moved Robot to Target Pose")
          else:
            rospy.logerr("Trajectory execution failed")
            print self.group.get_current_pose()
            return False
      else:
          rospy.logerr("Motion Planning Failed.")
          return False
       
      return True
            
    def run(self):
      
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.y = 1.0
        pose_target.position.x = 1.2
        pose_target.position.y = 0.0
        pose_target.position.z = 1.1
      ## Set start and target pose
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_target)

      #seed_joint_pose = self.moveit_commander.get_joint_value_target() 
        rospy.loginfo("Move to Home Pose:\n%s\n"%(str()))
      
        if(not self.moveRobot()):
            return False
     
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.y = 1.0
        pose_target.position.x = 1.2
        pose_target.position.y = 0.0
        pose_target.position.z = 1.0
      ## Set start and target pose
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_target)

      #seed_joint_pose = self.moveit_commander.get_joint_value_target() 
        rospy.loginfo("Move to Home Pose:\n%s\n"%(str()))
      
        if(not self.moveRobot()):
            return False

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.y = 1.0
        pose_target.position.x = 1.2
        pose_target.position.y = 0.4
        pose_target.position.z = 0.93
      ## Set start and target pose
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_target)

      #seed_joint_pose = self.moveit_commander.get_joint_value_target() 
        rospy.loginfo("Move to Home Pose:\n%s\n"%(str()))
      
        if(not self.moveRobot()):
            return False
        return True     

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_execution',anonymous=True)
    rate = rospy.Rate(10)

    rospy.loginfo("Waiting for move_group node")
    if waitForMoveGroup():
      rospy.loginfo("Found move_group node, proceeding")
    else:
      rospy.logerr("Timed out waiting for move_group node, exiting ...")
      sys.exit(-1)

    rb_exec = RobotExecution()
    if not rb_exec.run():
      sys.exit(-1)
      
    sys.exit(0)
    
    

if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

