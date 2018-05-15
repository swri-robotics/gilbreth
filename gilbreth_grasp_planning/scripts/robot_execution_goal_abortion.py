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
import time

from gilbreth_msgs.msg import RobotTrajectories
from gilbreth_msgs.msg import TargetToolPoses
from gilbreth_gazebo.msg import VacuumGripperState
from gilbreth_gazebo.srv import VacuumGripperControl
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose

ROBOT_TRAJ_TOPIC='gilbreth/robot_trajectories'
TOOL_POSE_TOPIC='gilbreth/target_tool_poses'
GRIPPER_STATE_TOPIC='gilbreth/gripper/state'
GRIPPER_SERVICE_TOPIC='gilbreth/gripper/control'
CONTROLLER_SERVICE_TOPIC='controller_manager/switch_controller'
ARM_GROUP_NAME = 'robot'
ARM_CONTROLLER = ['robot_controller']
RAIL_GROUP_NAME = 'robot_rail'
RAIL_CONTROLLER = ['robot_rail_controller']
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

def curateTrajectory(traj):
  # This is a hack that fixes the issue reported in the link below
  # https://github.com/ros-controls/ros_controllers/issues/291
  if len(traj.joint_trajectory.points) > 0:  
    #rospy.logwarn("Trajectory points list is empty")
    traj.joint_trajectory.points[0].time_from_start = rospy.Duration(0.01)
  return traj   

class RobotExecution:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()        
        self.arm_group = self.robot.get_group(ARM_GROUP_NAME)
        self.rail_group = self.robot.get_group(RAIL_GROUP_NAME)
        self.arm_group.set_planning_time(1.0)
        self.rail_group.set_planning_time(1.0)
        self.max_planning_count = 5

        self.tool_sub = rospy.Subscriber(TOOL_POSE_TOPIC,TargetToolPoses,self.tool_poses_callback)
        self.gripper_sub = rospy.Subscriber(GRIPPER_STATE_TOPIC, VacuumGripperState, self.gripper_callback)
        self.gripper_client = rospy.ServiceProxy(GRIPPER_SERVICE_TOPIC, VacuumGripperControl)
        self.controller_client = rospy.ServiceProxy(CONTROLLER_SERVICE_TOPIC, SwitchController)

        self.gripper_state = VacuumGripperState()
        self.robot_trajectory = RobotTrajectories()
        self.tool_pose = TargetToolPoses()
        self.tool_poses_list = []
        self.EXECUTE = False

        self.last_gripper_state_print = time.time()
        ## define a waiting pose for robot
        self.waiting_pose = Pose()
        self.waiting_pose.position.x = 1.2
        self.waiting_pose.position.y = 0.0
        self.waiting_pose.position.z = 1.15
        self.waiting_pose.orientation.x = 0
        self.waiting_pose.orientation.y = 0.9999999
        self.waiting_pose.orientation.z = 0
        self.waiting_pose.orientation.w = 0


    def switch_controller(self,start_group,stop_group,effort = 1):
        rospy.wait_for_service(CONTROLLER_SERVICE_TOPIC)
        try:        
            resp = self.controller_client(start_controllers = start_group, stop_controllers = stop_group, strictness=effort)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to switch the controller: %s" %e)
            return False
        if not resp.ok:
            rospy.logerr("Failed to switch the controller: %s" % response)
        else:
            rospy.loginfo("Controller switched successfully.")
        return resp.ok

    def control_gripper(self,enabled):
        rospy.wait_for_service(GRIPPER_SERVICE_TOPIC)
        try:
            response = self.gripper_client(enable = enabled)
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to control the gripper: %s" % exc)
            return False
        if not response.success:
            rospy.logerr("Failed to control the gripper: %s" % response)
        else:
            rospy.loginfo("Gripper controlled successfully. State: %s" % enabled)
        return response.success

    def gripper_callback(self, gripper_data):
        if time.time() - self.last_gripper_state_print >= 10:
            self.last_gripper_state_print = time.time()
        self.gripper_state = copy.deepcopy(gripper_data)

    def tool_poses_callback(self, tool_pose_data):
        if tool_pose_data is not None:
            rospy.loginfo("Received new tool pose.")
            self.tool_pose = copy.deepcopy(tool_pose_data)
            self.tool_poses_list.append(self.tool_pose)
        else:
            rospy.logerr("Received tool pose is invalid.")

    def gripper_attached(self, pick_deadline):
        while (rospy.Time.now() < pick_deadline + rospy.Duration(2.0)):
            if self.gripper_state.attached:
                rospy.loginfo("Object attached.")
                return True
            else:
                rospy.sleep(0.1)
        rospy.logerr("Nothing attached to the gripper.")
        return self.gripper_state.attached

    def goto_waiting_pose(self):
        rospy.loginfo("======Go to Home Pose======")
        waiting_plan = self.compute_trajectory(self.rail_group,self.waiting_pose)
        if not waiting_plan:
            rospy.logerr("Motion Plan Failed time: Current Pose ==> Home Pose.")
        else:
            waiting_plan=curateTrajectory(waiting_plan)
            #rospy.loginfo("Motion Plan Success: Current Pose ==> Waiting Pose.")
            if not self.switch_controller(RAIL_CONTROLLER,ARM_CONTROLLER,1):
                rospy.logerr("Switch controller failed.")
            else:
                if not self.rail_group.execute(waiting_plan):
                    rospy.logerr("Failed to move robot.")
                else:
                    rospy.loginfo("Moved to home pose.")

    # get next available object to pick
    def get_available_obj(self):
        for i in range(len(self.tool_poses_list)):
            if self.tool_poses_list[0].pick_approach.header.stamp - rospy.Duration(3.0)> rospy.Time.now():
                #rospy.loginfo("Availble object in the list. Proceeding...")
                self.EXECUTE = True                
                return True
            else:
                rospy.logerr("Not availble. Poping and checking next obj")
                self.tool_poses_list.pop(0)
        return False
    
    def compute_trajectory(self, group, target_pose):
        traj= RobotTrajectory()
        group.set_start_state_to_current_state()
        group.set_pose_target(target_pose)
        for i in range(self.max_planning_count):
            plan = group.plan()
            if len(plan.joint_trajectory.points)>0:
                rospy.loginfo("Successfully computed trajectory.")
                return plan
            else:
                rospy.logerr("Failed to compute trajectory %d time. Recalculating", i)
        rospy.logerr("Failed to campute trajectory.")
        return False

    def motion_plan(self,start_group,target_pose,start_ctrl,stop_ctrl):
        start_time = time.time()
        traj = self.compute_trajectory(start_group,target_pose)
        rospy.loginfo("Run time is %f seconds",(time.time()-start_time))
        if traj:
            if self.switch_controller(start_ctrl,stop_ctrl,1):
                #rospy.loginfo("Motion plan success")
                return traj
        self.EXECUTE = False
        return False

    def move_robot(self, group, traj):
        if not traj:
            rospy.logerr("Motion plan failed.")
            self.EXECUTE = False
        else:
            if not group.execute(traj):
                rospy.logerr("Execution failed.")
                self.EXECUTE = False
            else:
                rospy.loginfo("Moved to target pose.")
        return self.EXECUTE
            
    def execute_robot(self):

      class ScopeExit(object):
        def __init__(self,obj):
          self.obj_ = obj

        def __enter__(self):
          return self

        def __exit__(self, exc_type, exc_value, traceback):
          self.tool_pose = None
          self.obj_.EXECUTE = False
          self.obj_.get_available_obj()
          return True

      with ScopeExit(self) as sc:                   
        if self.EXECUTE:
            target_tool_pose = copy.deepcopy(self.tool_poses_list[0])
            rospy.loginfo("======Go to pick approach pose======")
            app_traj = self.motion_plan(self.rail_group,target_tool_pose.pick_approach.pose,RAIL_CONTROLLER,ARM_CONTROLLER)
        
            if self.move_robot(self.rail_group,app_traj):
                rospy.loginfo("======Go to pick pose======")
                pick_traj = self.motion_plan(self.arm_group,target_tool_pose.pick_pose.pose,ARM_CONTROLLER,RAIL_CONTROLLER)
            
                if pick_traj:
                    pick_dur = pick_traj.joint_trajectory.points[-1].time_from_start
                    pick_deadline = copy.deepcopy(target_tool_pose.pick_pose.header.stamp)
                    if (rospy.Time.now() + pick_dur >pick_deadline):
                        rospy.logerr("Can not reach pick pose on time. Drop this picking assignment.")
                    else:
                        ## Wait to execute robot to pick item
                        rospy.loginfo("------Waiting to pick object-----")
                        current_time = rospy.Time.now()
                        wait_dur = pick_deadline.to_sec()-current_time.to_sec()-pick_dur.to_sec()
                        rospy.sleep(wait_dur)

                        rospy.loginfo("Enable Gripper.")
                        if self.control_gripper(True) and self.arm_group.execute(pick_traj):
                            rospy.loginfo("======Check object attachment======")
                            if self.gripper_attached(pick_deadline): 
                                #rospy.sleep(0.5)                  
                                rospy.loginfo("======Go to retreat pose======")
                                retreat_traj = self.motion_plan(self.arm_group,target_tool_pose.pick_retreat.pose,ARM_CONTROLLER,RAIL_CONTROLLER)

                                if self.move_robot(self.arm_group,retreat_traj):
                                    rospy.loginfo("======Go to place pose======")   
                                    place_traj = self.motion_plan(self.rail_group,target_tool_pose.place_pose.pose,RAIL_CONTROLLER,ARM_CONTROLLER)

                                    if self.move_robot(self.rail_group,place_traj):
                                        rospy.loginfo("Disable Gripper")
                                        if self.control_gripper(False):
                                            rospy.loginfo("======Successsfully placed object======")
 
            self.tool_poses_list.pop(0)
            self.control_gripper(False)
            self.EXECUTE = False        

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_execution',anonymous=True)
    rate = rospy.Rate(10)

    rospy.loginfo("Waiting for move group")
    if waitForMoveGroup():
      rospy.loginfo("Found move group node, proceeding")
    else:
      rospy.logerr("Timed out waiting for move group node, exiting ...")
      sys.exit(-1)

    rb_exec = RobotExecution()
    rb_exec.goto_waiting_pose()

    while not rospy.is_shutdown():
        rb_exec.execute_robot()
        rate.sleep()


if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

