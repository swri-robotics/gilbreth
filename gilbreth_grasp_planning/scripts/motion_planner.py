#!/usr/bin/env python
## This is the motion planning node for gilbreth project.
## It subscribes from a fake tool pose publisher(tool_publisher node) and publish the robot trajectories for the robot execution node.
## The fake tool pose publisher will be replaced by tool planning node once the conveyorbelt population system is done.
## The trajectories can be visualized in RVIZ. 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gilbreth_msgs.msg import TargetToolPoses
from gilbreth_msgs.msg import RobotTrajectories
from moveit_msgs.msg import RobotState
from std_msgs.msg import Duration

tool_poses = TargetToolPoses()
trajectories_msgs = RobotTrajectories()

## Initialize robot, scene, move group and Rviz
def robot_init():
    global robot, scene, group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate "robot_rail" chain as a MoveGroupCommander object.
    group = moveit_commander.MoveGroupCommander("robot_rail")
    ## Basic Information
    print "======Getting Robot Information======"
    print "======Reference frame: %s" % group.get_planning_frame()
    print "======End effector: %s" % group.get_end_effector_link() 
    print "======Robot Groups:"
    print robot.get_group_names()
    print "======Printing robot state"
    print robot.get_current_state()
    print "====================================="
    print "======Ready for Motion Planning======"


    global display_trajectory_publisher, robot_trajectories_publisher
    ## Create a DisplayTrajectory publisher for RViz and debugging
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    ## Create a RobotTrajectories publisher for gilbreth execution node
    robot_trajectories_publisher = rospy.Publisher('/gilbreth/robot_trajectories', RobotTrajectories, queue_size = 20)

## The motion planner function use MoveIt to compute the robot trajectories. 
def motion_planner(tool_poses):
    ## Compute robot trajectories from current pose to pick_approach pose
    print "Start Motion Planning: Current ==> Pick Approach."
    approach_target = tool_poses.pick_approach.pose

    ## Set start pose as current robot pose
    group.set_start_state_to_current_state()
    ## Get target pose
    group.set_pose_target(approach_target)
    ## Motion Planning    
    approach_plan = group.plan()

    if approach_plan:
        ## Waiting for RVIZ to display
        rospy.loginfo("Motion Plan Success: Current ==> Pick Approach. Waiting for RVIZ to display...")
        ## Uncomment to visualize in RViz
        ## RVIZ visualizion trajectories from current state to approach state        
        #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #display_trajectory.trajectory_start = robot.get_current_state()
        #display_trajectory.trajectory.append(approach_plan)
        #display_trajectory_publisher.publish(display_trajectory);
        
        trajectories_msgs.cur_to_approach = approach_plan
        #rospy.sleep(2)

        ## uncomment to execute
        #group.execute(trajectories_msgs.cur_to_approach)
    else:
        rospy.logerr("Motion Plan Fail: Current ==> Pick Approach.")
        return "Motion Plan Fail"
    
    ## Compute trajectories from pick approach pose to pick pose
    print "Start Motion Planning: Pick Approach ==> Pick."

    ## start with approach pose
    approach_state = RobotState()
    approach_state.joint_state.name = trajectories_msgs.cur_to_approach.joint_trajectory.joint_names
    approach_state.joint_state.position = trajectories_msgs.cur_to_approach.joint_trajectory.points[-1].positions
    group.set_start_state(approach_state)

    ## Use cartesian path planning for pick motion
    waypoints = []
    waypoints.append(copy.deepcopy(approach_target))
    ## Get target pose
    pick_target = tool_poses.pick_pose.pose
    waypoints.append(copy.deepcopy(pick_target))
    
    (approach_to_pick_plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    if approach_to_pick_plan:
        rospy.loginfo("Motion Plan Success: Pick Approach ==> Pick. Waiting for RVIZ to display...")
        #rospy.sleep(2)

        trajectories_msgs.approach_to_pick = approach_to_pick_plan

        ## uncomment to execute for debugging
        #group.execute(trajectories_msgs.approach_to_pick)

    else:
        rospy.logerr("Motion Plan Fail: Pick Approach ==> Pick.")
        return "Motion Plan Fail"
    
    ## Compute trajectories from pick pose to retreat pose
    print "Start Motion Planning: Pick ==> Pick Retreat."

    ## start with pick pose
    pick_state = RobotState()
    pick_state.joint_state.name = trajectories_msgs.approach_to_pick.joint_trajectory.joint_names
    pick_state.joint_state.position = trajectories_msgs.approach_to_pick.joint_trajectory.points[-1].positions
    group.set_start_state(pick_state)

    ## target retreat pose
    retreat_target = tool_poses.pick_retreat.pose
    group.set_pose_target(retreat_target)

    pick_to_retreat_plan = group.plan()  

    if pick_to_retreat_plan:
        rospy.loginfo("Motion Plan Success: Pick ==> Pick Retreat. Waiting for RVIZ to display...")
        #rospy.sleep(2)
        
        trajectories_msgs.pick_to_retreat = pick_to_retreat_plan

        ## uncomment to execute robot
        #group.execute(trajectories_msgs.pick_to_retreat)
    else:
        rospy.logerr("Motion Plan Fail: Pick ==> Pick Retreat.")
        return "Motion Plan Fail"


    ## Compute trajectories from retreat pose to place pose
    print "Start Motion Planning: Pick Retreat ==> Place."

    ## start with retreat pose
    retreat_state = RobotState()
    retreat_state.joint_state.name = trajectories_msgs.pick_to_retreat.joint_trajectory.joint_names
    retreat_state.joint_state.position = trajectories_msgs.pick_to_retreat.joint_trajectory.points[-1].positions
    group.set_start_state(retreat_state)

    ## target place pose   
    place_target = tool_poses.place_pose.pose
    group.set_pose_target(place_target)
    place_plan = group.plan()   

    if place_plan:
        rospy.loginfo("Motion Plan Success: Pick Retreat ==> Place. Waiting for RVIZ to display...")
        #rospy.sleep(2)
        trajectories_msgs.retreat_to_place = place_plan

        ## uncomment to execute robot
        #group.execute(trajectories_msgs.retreat_to_place)
        #rospy.sleep(7)
    else:
        rospy.logerr("Motion Plan Fail: Pick Retreat ==> Place.")
        return "Motion Plan Fail"
    

    print "============Finish Motion Planning==========="
    ## Get Motion plan success time 
    trajectories_msgs.header.stamp = rospy.get_rostime()
    ## Publish robot trajectories

def compute_waiting_time(trajectories_msgs,tool_poses):
    ## Get planned robot execution time
    approach_duration = trajectories_msgs.cur_to_approach.joint_trajectory.points[-1].time_from_start
    pick_duration = trajectories_msgs.approach_to_pick.joint_trajectory.points[-1].time_from_start
    retreat_duration = trajectories_msgs.pick_to_retreat.joint_trajectory.points[-1].time_from_start
    place_duration = trajectories_msgs.retreat_to_place.joint_trajectory.points[-1].time_from_start
    
    time_dur = [None] * 4
    time_dur[0] = approach_duration
    time_dur[1] = pick_duration
    time_dur[2] = retreat_duration
    time_dur[3] = place_duration

    trajectories_msgs.execution_duration = time_dur
    rospy.loginfo(approach_duration.to_sec())
    rospy.loginfo(pick_duration.to_sec())
    rospy.loginfo(retreat_duration.to_sec())
    rospy.loginfo(place_duration.to_sec())  
  
    ## Get deadline time for reaching picking pose
    trajectories_msgs.pick_deadline = tool_poses.pick_pose.header.stamp

    ## Publish robot trajectories messages 
    robot_trajectories_publisher.publish(trajectories_msgs)
    print "Publishing robot trajectories to /gilbreth/robot_trajectories"


## A target poses callback function subscribing from the fake tool pose publisher.
## When it receives the tool poses, it calls motion planner to compute the trajectories.
def tool_poses_callback(data):
    tool_poses = data
    motion_planner(tool_poses)
    compute_waiting_time(trajectories_msgs,tool_poses)

if __name__=='__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('motion_planner',anonymous=True)
    
        ## a TargetToolPoses subscriber
        rospy.Subscriber('/gilbreth/target_tool_poses', TargetToolPoses, tool_poses_callback)
        rate = rospy.Rate(10)
        robot_init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

