#!/usr/bin/env python
## a python script compute the tool poses
import rospy

from gilbreth_msgs.msg import TargetToolPoses
from std_msgs.msg import Int16

index_data = Int16()
tool_msg = TargetToolPoses()
#obj_data = ObjectDetection()

obj_dict = {0:"gear",1:"disk",2:"pulley",3:"gasket",4:"disk"}


## a tool_pose publisher
tool_pose_publisher = rospy.Publisher('gilbreth/target_tool_poses', TargetToolPoses, queue_size=10)

## load bin_origins, robot_origin and tunable parameters 
## like lifting distance and dropping distance, conveyor belt velocity
def load_param():
    parameters = rospy.get_param('/gilbreth/tool_plan/parameters')
    pose_timers = rospy.get_param('/gilbreth/tool_plan/pose_timers')
    global pick_poses
    pick_poses = rospy.get_param('gilbreth/pick_pose')
    print pick_poses

    
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
    print bin_origins

    #load robot home pose
    global robot_home_pose
    robot_home_pose = rospy.get_param('/gilbreth/tool_plan/robot_origin/pose')
    
    rospy.loginfo ("Success loading parameters")


## use object detected name to lookup correct bin
## calculate robot tool poses
def compute_tool_poses(index_data):
    #find correct bin and its pose
    index = index_data.data
    print index
    name = obj_dict[index]
    print name
    bin_pose = bin_origins[name]['pose']
   
    #rospy.loginfo(obj_data)

    #compute tool poses
    #get ros time as header
    #tool_msg.header.stamp =rospy.get_rostime()

    tool_msg.header.stamp = rospy.get_rostime()
    tool_msg.pick_approach.header.frame_id = 'world_frame'
    tool_msg.pick_approach.header.stamp = tool_msg.header.stamp + rospy.Duration(8)
    tool_msg.pick_approach.header.frame_id = 'world_frame'
    tool_msg.pick_pose.header.stamp = tool_msg.header.stamp + rospy.Duration(15)
    tool_msg.pick_retreat.header.frame_id = 'world_frame'
    tool_msg.pick_retreat.header.stamp = tool_msg.header.stamp + rospy.Duration(18)
    tool_msg.place_pose.header.frame_id = 'world_frame'
    tool_msg.place_pose.header.stamp = tool_msg.header.stamp + rospy.Duration(22)
    

    #compute pick_approach pose (%lift meter above the pick up position)
    tool_msg.pick_approach.pose.position.x = 1.20
    tool_msg.pick_approach.pose.position.y = 0.4
    tool_msg.pick_approach.pose.position.z = 0.93 + lift        
    tool_msg.pick_approach.pose.orientation.x = 0.0
    tool_msg.pick_approach.pose.orientation.y = 0.9999
    tool_msg.pick_approach.pose.orientation.z = 0.0
    tool_msg.pick_approach.pose.orientation.w = 0.0

    #compute pick pose
    tool_msg.pick_pose.pose.position.x = 1.20+pick_poses[name]['pose']['xyz'][0]
    tool_msg.pick_pose.pose.position.y = 0.4+pick_poses[name]['pose']['xyz'][1]
    tool_msg.pick_pose.pose.position.z = 0.920+pick_poses[name]['pose']['xyz'][2]
    print "z direction : %f" %tool_msg.pick_pose.pose.position.z       
    tool_msg.pick_pose.pose.orientation.x = 0.0
    tool_msg.pick_pose.pose.orientation.y = 0.9999
    tool_msg.pick_pose.pose.orientation.z = 0.0
    tool_msg.pick_pose.pose.orientation.w = 0.0

    #compute pick_retreat pose        
    tool_msg.pick_retreat.pose.position.x = 1.20
    tool_msg.pick_retreat.pose.position.y = 0.40
    tool_msg.pick_retreat.pose.position.z = 0.93 + lift 
    tool_msg.pick_retreat.pose.orientation.x = 0.0
    tool_msg.pick_retreat.pose.orientation.y = 0.9999
    tool_msg.pick_retreat.pose.orientation.z = 0.0
    tool_msg.pick_retreat.pose.orientation.w = 0.0
    #compute place pose
    
    tool_msg.place_pose.pose.position.x = bin_pose['xyz'][0]
    tool_msg.place_pose.pose.position.y = bin_pose['xyz'][1]
    tool_msg.place_pose.pose.position.z = 1.0
    tool_msg.place_pose.pose.orientation.x = 0.0
    tool_msg.place_pose.pose.orientation.y = 0.9999
    tool_msg.place_pose.pose.orientation.z = 0.0
    tool_msg.place_pose.pose.orientation.w = 0.0 

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
def obj_index_callback(data):
    index_data = data
    #rospy.loginfo(obj_data)
    #compute_robot_timer(index_data)
    compute_tool_poses(index_data)
    #rospy.loginfo(tool_msg)
    tool_pose_publisher.publish(tool_msg)
    print "Publishing tool pose data..."
    

if __name__ == '__main__':
    try:
        rospy.init_node('tool_planning_node')
        rate = rospy.Rate(10)
        load_param()
        rospy.Subscriber('/gilbreth/object_id',Int16,obj_index_callback)
        rospy.spin()

    except rospy.ROSInterruptException: pass
