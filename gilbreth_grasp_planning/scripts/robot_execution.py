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
from osrf_gear.msg import VacuumGripperState
from osrf_gear.srv import VacuumGripperControl

class robot_execution:

    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander("robot_rail")
        self.trajectory_sub = rospy.Subscriber('/gilbreth/robot_trajectories', RobotTrajectories, self.trajectory_callback)
        self.gripper_sub = rospy.Subscriber('/gilbreth/gripper/state', VacuumGripperState, self.gripper_callback)
        self.gripper_client = rospy.ServiceProxy('/gilbreth/gripper/control', VacuumGripperControl)
        self.gripper_state = VacuumGripperState()
        self.robot_trajectory = RobotTrajectories()
        self.EXECUTE = False

    def gripper_callback(self, gripper_data):
        self.gripper_state = gripper_data
        
    def trajectory_callback(self,trajectory_data):
        if trajectory_data:        
            self.robot_trajectory = trajectory_data
            self.EXECUTE = True
            print "EXECUTING:" 
            print self.EXECUTE
        
        
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

    ## check if object is attached to gripper or not
    def gripper_attached(self, pick_dl):
        while (rospy.Time.now() < pick_dl + rospy.Duration(1.5)):
            print self.gripper_state.attached
            if self.gripper_state.attached:
                return True
            else:
                rospy.sleep(0.2)
        print "Nothing Attached"
        return False
    
    ## execute the robot based on robot_trajectories
    def execute_robot(self):

        if self.EXECUTE:
            print "Moving robot from current pose to approach pick pose"
            approach_dur = self.robot_trajectory.execution_duration[0]
            self.group.execute(self.robot_trajectory.cur_to_approach)
            #approach_dur = math.ceil(self.robot_trajectory.execution_duration[0].to_sec())
            print "Approaching duration is : %f" % approach_dur.to_sec()
            rospy.sleep(approach_dur.to_sec())
    
            ## check if can reach pick pose on time    
            current_time = rospy.Time.now()
            pick_deadline = self.robot_trajectory.pick_deadline
            pick_dur = self.robot_trajectory.execution_duration[1]

            if (current_time + pick_dur > pick_deadline):
                rospy.logerr("Can not reach pick pose on time. Drop this object picking assignment.")
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
                rospy.sleep(math.ceil(pick_dur.to_sec()))

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
                    self.group.execute(self.robot_trajectory.pick_to_retreat)
                    rospy.sleep(math.ceil(self.robot_trajectory.execution_duration[2].to_sec()))

                self.disable_gripper()
                self.EXECUTE = False   

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_execution',anonymous=True)
    rate = rospy.Rate(10)
    test = robot_execution()
    while not rospy.is_shutdown():
        test.execute_robot()
        rate.sleep()


if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

