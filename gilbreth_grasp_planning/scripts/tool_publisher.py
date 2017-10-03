#!/usr/bin/env python
## A fake tool pose publisher since currently gilbreth does not support populating models on conveyorbelt.
## This fake tool_pose publisher will be replaced once conveyor belt is supported.

import rospy
import tf
import std_msgs.msg

from gilbreth_msgs.msg import TargetToolPoses

tool_msg = TargetToolPoses()

def fake_toolpose_publisher():
    tool_pose_pub = rospy.Publisher('/gilbreth/target_tool_poses', TargetToolPoses, queue_size=10)
    rospy.init_node('fake_toolpose_publisher')
    rate = rospy.Rate(1)
    rate.sleep()

    count = 1

    rospy.sleep(2)

    while not rospy.is_shutdown():
        tool_msg.header.stamp = rospy.get_rostime()
        tool_msg.pick_approach.header.frame_id = 'world_frame'
        tool_msg.pick_approach.header.stamp = tool_msg.header.stamp + rospy.Duration(4)
        tool_msg.pick_approach.header.frame_id = 'world_frame'
        tool_msg.pick_pose.header.stamp = tool_msg.header.stamp + rospy.Duration(6)
        tool_msg.pick_retreat.header.frame_id = 'world_frame'
        tool_msg.pick_retreat.header.stamp = tool_msg.header.stamp + rospy.Duration(8)
        tool_msg.place_pose.header.frame_id = 'world_frame'
        tool_msg.place_pose.header.stamp = tool_msg.header.stamp + rospy.Duration(12)
    
        tool_msg.pick_approach.pose.position.x = 1.20
        tool_msg.pick_approach.pose.position.y = 0.40
        tool_msg.pick_approach.pose.position.z = 1.10
        tool_msg.pick_approach.pose.orientation.x = 0.0
        tool_msg.pick_approach.pose.orientation.y = 0.9999
        tool_msg.pick_approach.pose.orientation.z = 0.0
        tool_msg.pick_approach.pose.orientation.w = 0.0
    
    
        tool_msg.pick_pose.pose.position.x = 1.20
        tool_msg.pick_pose.pose.position.y = 0.40
        tool_msg.pick_pose.pose.position.z = 0.93
        tool_msg.pick_pose.pose.orientation.x = 0.0
        tool_msg.pick_pose.pose.orientation.y = 0.9999
        tool_msg.pick_pose.pose.orientation.z = 0.0
        tool_msg.pick_pose.pose.orientation.w = 0.0
    
        tool_msg.pick_retreat.pose.position.x = 1.20
        tool_msg.pick_retreat.pose.position.y = 0.40
        tool_msg.pick_retreat.pose.position.z = 1.10
        tool_msg.pick_retreat.pose.orientation.x = 0.0
        tool_msg.pick_retreat.pose.orientation.y = 0.9999
        tool_msg.pick_retreat.pose.orientation.z = 0.0
        tool_msg.pick_retreat.pose.orientation.w = 0.0
        
        if count % 2 == 1:
            tool_msg.place_pose.pose.position.x = -0.30
            tool_msg.place_pose.pose.position.y = 0.995
            tool_msg.place_pose.pose.position.z = 0.90
            tool_msg.place_pose.pose.orientation.x = 0.0
            tool_msg.place_pose.pose.orientation.y = 0.9999
            tool_msg.place_pose.pose.orientation.z = 0.0
            tool_msg.place_pose.pose.orientation.w = 0.0 
    
        else:
            tool_msg.place_pose.pose.position.x = -0.30
            tool_msg.place_pose.pose.position.y = 0.23
            tool_msg.place_pose.pose.position.z = 0.90
            tool_msg.place_pose.pose.orientation.x = 0.0
            tool_msg.place_pose.pose.orientation.y = 0.9999
            tool_msg.place_pose.pose.orientation.z = 0.0
            tool_msg.place_pose.pose.orientation.w = 0.0 

        count = count + 1
        rospy.loginfo(tool_msg)
        tool_pose_pub.publish(tool_msg)
        rospy.loginfo("Publishing Poses.")
        rospy.sleep(30)
    
if __name__ == '__main__':
    try:
        fake_toolpose_publisher()
    except rospy.ROSInterruptException: 
        pass




