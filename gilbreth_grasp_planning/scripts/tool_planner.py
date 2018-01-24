#!/usr/bin/env python
## a python script compute the tool poses
import rospy
import time

from gilbreth_msgs.msg import ObjectDetection
from gilbreth_msgs.msg import TargetToolPoses

tool_msg = TargetToolPoses()
obj_data = ObjectDetection()

## a tool_pose publisher
tool_pose_publisher = rospy.Publisher('/gilbreth/target_tool_poses', TargetToolPoses, queue_size=10)

## load poses of bins and the robot 
## load tunable parameters, like lifting distance and dropping distance, conveyor belt velocity
def load_param():
    parameters = rospy.get_param('/gilbreth/tool_plan/parameters')
    pose_timers = rospy.get_param('/gilbreth/tool_plan/pose_timers')
    
    global lift, drop, speed, length
    lift, drop, speed, length = parameters['lift_distance'], parameters['drop_distance'], parameters['conveyor_speed'],parameters['conveyor_belt_length']
    rospy.loginfo("params are %f, %f, %f, %f", lift, drop, speed, length)
    
    global conveyor_time
    conveyor_time = length / speed

    global approach_time, pick_time, retreat_time, place_time
    approach_time, pick_time,retreat_time, place_time = pose_timers['approach_time'], pose_timers['pick_time'], pose_timers['retreat_time'], pose_timers['place_time']

    rospy.loginfo("timers are %f, %f, %f, %f", approach_time, pick_time, retreat_time, place_time)

    #load bin poses into dict bin_origin
    global bin_origins
    bin_origins = rospy.get_param('/gilbreth/tool_plan/bin_origins')

    #load robot home pose
    global robot_home_pose
    robot_home_pose = rospy.get_param('/gilbreth/tool_plan/robot_origin/pose')
    
    global lift_distance, retreat_distance
    lift_distance = (pick_time - approach_time) * speed
    retreat_distance = (retreat_time - pick_time) * speed
    rospy.loginfo ("Success loading parameters")


## use object detected name to lookup correct bin
## calculate robot tool poses
def compute_tool_poses(obj_data):
    #find correct bin and its pose
    name = obj_data.name
    bin_pose = bin_origins[name]['pose']

    #print "Calculationg tool pose data..."
    rospy.loginfo("Current object is %s",obj_data.name)

    #compute tool poses
    #get ros time as header
    tool_msg.header.stamp =rospy.get_rostime()

    #compute pick_approach pose (%lift meter above the pick up position)
    tool_msg.pick_approach.pose.position.x = obj_data.pose.position.x
    tool_msg.pick_approach.pose.position.y = obj_data.pose.position.y - length 
    tool_msg.pick_approach.pose.position.z = obj_data.pose.position.z + lift
    tool_msg.pick_approach.pose.orientation = obj_data.pose.orientation

    #compute pick pose
    tool_msg.pick_pose.pose.position.x = obj_data.pose.position.x
    tool_msg.pick_pose.pose.position.y = obj_data.pose.position.y - length
    tool_msg.pick_pose.pose.position.z = obj_data.pose.position.z
    tool_msg.pick_pose.pose.orientation = obj_data.pose.orientation

    #compute pick_retreat pose
    tool_msg.pick_retreat.pose.position.x = obj_data.pose.position.x
    tool_msg.pick_retreat.pose.position.y = obj_data.pose.position.y - length
    tool_msg.pick_retreat.pose.position.z = obj_data.pose.position.z + drop
    tool_msg.pick_retreat.pose.orientation = obj_data.pose.orientation

    #compute place pose
    
    tool_msg.place_pose.pose.position.x = bin_pose['xyz'][0]
    tool_msg.place_pose.pose.position.y = bin_pose['xyz'][1]
    tool_msg.place_pose.pose.position.z = bin_pose['xyz'][2]
    tool_msg.place_pose.pose.orientation = obj_data.pose.orientation

    #rospy.loginfo("Tool poses for object %s is:",obj_data.name)    
    #rospy.loginfo(tool_msg)    


## compute the deadline for each robot pose
def compute_robot_timer(obj_data):
    tool_msg.pick_approach.header.frame_id = 'world_frame'
    tool_msg.pick_approach.header.stamp = obj_data.detection_time + rospy.Duration(approach_time)
    
    tool_msg.pick_pose.header.frame_id = 'world_frame'
    tool_msg.pick_pose.header.stamp = obj_data.detection_time + rospy.Duration(pick_time)

    tool_msg.pick_retreat.header.frame_id = 'world_frame'
    tool_msg.pick_retreat.header.stamp = obj_data.detection_time + rospy.Duration(retreat_time)

    tool_msg.place_pose.header.frame_id = 'world_frame'
    tool_msg.place_pose.header.stamp = obj_data.detection_time + rospy.Duration(place_time)

## a callback function to get object detection data
def obj_detect_callback(data):
    start_time = time.time()
    obj_data = data
    compute_robot_timer(obj_data)
    compute_tool_poses(obj_data)
    #rospy.loginfo(tool_msg)
    tool_pose_publisher.publish(tool_msg)
    rospy.loginfo("Publishing tool pose data...Run time is %f seconds",(time.time()-start_time))
    

if __name__ == '__main__':
    try:
        rospy.init_node('tool_planning_node')
        rate = rospy.Rate(10)
        load_param()
        rospy.Subscriber('/recognition_result_world',ObjectDetection,obj_detect_callback)
        rospy.spin()

    except rospy.ROSInterruptException: pass
