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
  
def createPoses(seed_joint_pose, num_poses = 4):
  
  joint_poses = []
  num_joints = len(seed_joint_pose)
  for i in range(num_poses):
    jp = copy.deepcopy(seed_joint_pose)
    for j in range(num_joints):
      jp[j] = (random.uniform(-1, 1)*JOINT_RAND_FACTOR[j]) + seed_joint_pose[j]
    
    joint_poses.append(jp)
    
  return joint_poses       

def curateTrajectory(traj):
  # This is a hack that fixes the issue reported in the link below
  # https://github.com/ros-controls/ros_controllers/issues/291
  traj.joint_trajectory.points[0].time_from_start = rospy.Duration(0.01)
  return traj    
            


class RobotExecution:

    def __init__(self):
        self.moveit_commander = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.moveit_commander.set_planning_time(2.0)

    def moveRobot(self):      
  
      ## Motion Planning    
      robot_traj = self.moveit_commander.plan()
      
  
      ## if trajectory is valid, move the robot
      if robot_traj:
          
          robot_traj = curateTrajectory(robot_traj)
          rospy.loginfo("Motion Plan Success: Current Pose ==> Waiting Pose.")
          if self.moveit_commander.execute(robot_traj):
            rospy.loginfo("Moved Robot to Target Pose")
            
          else:
            rospy.logerr("Joint trajectory execution failed")
            return False
      else:
          rospy.logerr("Motion Plan Failed time: Current Pose ==> Waiting Pose.")
          return False
       
      return True
            
    def run(self):
      
      
      ## Set start and target pose
      self.moveit_commander.set_start_state_to_current_state()
      self.moveit_commander.set_named_target(HOME_JOINT_POSE)      
      seed_joint_pose = self.moveit_commander.get_joint_value_target() 
      rospy.loginfo("Move to Home Pose:\n%s\n"%(str()))
      
      if(not self.moveRobot()):
        return False
      
      #return True  
      
      # Moving to each pose
      joint_poses = createPoses(seed_joint_pose ,8)
      for i, p in enumerate(joint_poses):
        self.moveit_commander.set_start_state_to_current_state()
        self.moveit_commander.set_joint_value_target(p)
        
        rospy.loginfo("Moving to position: %s"%(str(p) ))
        
        if not self.moveRobot():
          return False
             

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

