#!/usr/bin/env python
## A node executes the robot. 
## Including Gripper control.

import sys
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gilbreth_msgs.msg import RobotTrajectories
from gilbreth_gazebo.msg import VacuumGripperState
from gilbreth_gazebo.srv import VacuumGripperControl
from geometry_msgs.msg import Pose


class robot_execution:

    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander("robot_rail")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group.set_planning_time(1.0)

        self.trajectory_sub = rospy.Subscriber('/gilbreth/robot_trajectories', RobotTrajectories, self.trajectory_callback)
        self.gripper_sub = rospy.Subscriber('/gilbreth/gripper/state', VacuumGripperState, self.gripper_callback)
        self.gripper_client = rospy.ServiceProxy('/gilbreth/gripper/control', VacuumGripperControl)

        self.gripper_state = VacuumGripperState()
        self.robot_trajectory = RobotTrajectories()
        self.EXECUTE = False

        ## define a waiting pose for robot
        self.waiting_pose = Pose()
        self.waiting_pose.position.x = 1.2
        self.waiting_pose.position.y = 0.0
        self.waiting_pose.position.z = 1.2
        self.waiting_pose.orientation.x = 0
        self.waiting_pose.orientation.y = 0.9999999
        self.waiting_pose.orientation.z = 0
        self.waiting_pose.orientation.w = 0


    def goto_waiting_pose(self):
        print "Start Motion Planning: Current Pose ==> Waiting Pose."
        ## Set start and target pose
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(self.waiting_pose)

        ## Motion Planning    
        waiting_plan = self.group.plan()

        ## if trajectory is valid, move the robot
        if waiting_plan:
            rospy.loginfo("Motion Plan Success: Current Pose ==> Waiting Pose.")
            print "Moving robot to waiting pose."
            self.group.execute(waiting_plan)
        else:
            rospy.logerr("Motion Plan Failed time: Current Pose ==> Pick Approach.")

    ## callback functions
    def gripper_callback(self, gripper_data):
        self.gripper_state = gripper_data
        
    def trajectory_callback(self,trajectory_data):
        if trajectory_data:        
            self.robot_trajectory = trajectory_data
            self.EXECUTE = True
            print "EXECUTING:%s" %self.EXECUTE 
#            print self.EXECUTE
        
    ## enable vacuum gripper suction cup
    def enable_gripper(self):
        print "Enabling Vacuum Gripper..."
        rospy.wait_for_service('/gilbreth/gripper/control')
        try:
            resp = self.gripper_client(enable = True)
            print "Gripper Enabling Success "
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    ## disable vacuum gripper suction cup
    def disable_gripper(self):
        print "Disabling Vacuum Gripper..."
        rospy.wait_for_service('/gilbreth/gripper/control')
        try:
            resp = self.gripper_client(enable = False)
            print "Gripper Disabling Success "
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def gripper_attached(self, pick_dead):
        while (rospy.Time.now() < pick_dead + rospy.Duration(2.0)):
            print self.gripper_state.attached
            if self.gripper_state.attached:
                return True
            else:
                rospy.sleep(0.2)
        print "Nothing Attached"
        return self.gripper_state.attached
    
    ## execute the robot based on robot_trajectories
    def execute_robot(self):

        if self.EXECUTE:
            print "Moving robot from current pose to pick approach pose"
            approach_dur = self.robot_trajectory.execution_duration[0]
            self.group.execute(self.robot_trajectory.cur_to_approach)
            print "Approaching duration is : %f" % approach_dur.to_sec()
            rospy.sleep(approach_dur.to_sec())
    
            ## check if can reach pick pose on time    
            pick_deadline = self.robot_trajectory.pick_deadline
            pick_dur = self.robot_trajectory.execution_duration[1]
            current_time = rospy.Time.now()

            if (current_time + pick_dur > pick_deadline):
                rospy.logerr("Can not reach pick pose on time. Drop this object picking assignment.")
                #self.goto_waiting_pose()
                self.EXECUTE = False
            else:
                ## Wait to execute robot to pick item
                print "Waiting to pick object"
                current_time = rospy.Time.now()
                wait_dur = pick_deadline.to_sec()-current_time.to_sec()-pick_dur.to_sec()-0.5
                rospy.sleep(wait_dur)

                ## enable gripper for object grasping
                self.enable_gripper()

                print "Moving robot from approach pose to pick pose"
                self.group.execute(self.robot_trajectory.approach_to_pick)
                print "Picking duration is : %f" % pick_dur.to_sec()
                rospy.sleep(pick_dur.to_sec())

                ## check if any object is attached to the vacuum gripper
                
                if self.gripper_attached(pick_deadline):

                    print "Executing pick to retreat plan"
                    self.group.execute(self.robot_trajectory.pick_to_retreat)
                    rospy.sleep(math.ceil(self.robot_trajectory.execution_duration[2].to_sec()))
            
                    rospy.sleep(1)
                    print "Executing retreat to place plan"
                    self.group.execute(self.robot_trajectory.retreat_to_place)
                    rospy.sleep(math.ceil(self.robot_trajectory.execution_duration[3].to_sec()))

                else:
    
                    rospy.logerr("Nothing attached to the gripper. Go to home pose now.")
                    self.goto_waiting_pose()

                self.disable_gripper()
            self.EXECUTE = False   
        

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_execution',anonymous=True)
    rate = rospy.Rate(10)
    test = robot_execution()
    test.goto_waiting_pose()
    while not rospy.is_shutdown():
        test.execute_robot()
        rate.sleep()


if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

