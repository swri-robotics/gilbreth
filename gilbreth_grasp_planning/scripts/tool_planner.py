#!/usr/bin/env python
## a python script compute the tool poses
import rospy
import time
import copy
from gilbreth_msgs.msg import ObjectDetection
from gilbreth_msgs.msg import TargetToolPoses

tool_msg = TargetToolPoses()
obj_data = ObjectDetection()


TOOL_POSE_TOPIC='/gilbreth/target_tool_poses'
GENERAL_PARAM = '/gilbreth/tool_plan'



## a tool_pose publisher
tool_pose_publisher = rospy.Publisher(TOOL_POSE_TOPIC, TargetToolPoses, queue_size=10)

def loadParameters():
    try:
        params = rospy.get_param(GENERAL_PARAM)
        rospy.loginfo("Sucessfully loaded parameter server.")

        ## environment params: lift and drop distance; conveyor belt speed, power and length
        env_param = params['env_param']
        global lift, drop, vel, power, length 
        lift, drop, vel, power, length = env_param['lift_distance'], env_param['drop_distance'], env_param['conveyor_velocity'],env_param['conveyor_power'], env_param['conveyor_belt_length']
        vel = vel * power
        global conveyor_time
        conveyor_time = length / vel
        rospy.loginfo("lift distance: %f\ndrop distance: %f\nconveyor belt speed %f\nconveyor belt length: %f\ntime befor pickup: %f\n", lift, drop, vel, length, conveyor_time)
        
        ## timer params: the deadline of each motion stage
        pose_timers = params['pose_timer']
        global approach_time, pick_time, retreat_time, place_time
        approach_time, pick_time,retreat_time, place_time = pose_timers['approach_time'], pose_timers['pick_time'], pose_timers['retreat_time'], pose_timers['place_time']
        rospy.loginfo("timers are %f, %f, %f, %f\n", approach_time, pick_time, retreat_time, place_time)

        ## bin poses: where to drop the object of each type
        global bin_origins
        bin_origins = params['bin_origin']
        global robot_home_pose
        robot_home_pose = params['robot_origin']
        global pick_offset
        pick_offset = params['pick_offset']

        rospy.loginfo ("*******Successfully loaded parameters.********")
        return True
    except rospy.ROSInterruptException as e: 
        rospy.logerr("Failed to load parameters.\n %s",e)
        return False
        

class ToolPlanner():
    def __init__(self):
        self.tool_pose_pub = rospy.Publisher(TOOL_POSE_TOPIC, TargetToolPoses, queue_size=10)
        self.obj_dect_sub = rospy.Subscriber('/recognition_result_world',ObjectDetection,self.obj_detect_callback)
        self.obj_data = ObjectDetection()

    def obj_detect_callback(self,data):
        if data is not None:
            self.obj_data = copy.deepcopy(data)
            rospy.loginfo("Received new object information.")
        try:
            start_time = time.time()
            if self.compute_tool_poses():
                rospy.loginfo("Run time is %f seconds",(time.time()-start_time))
        finally:
            start_time = None
            self.obj_data = None
    
    def compute_tool_poses(self):
        try: 
            tool_msg = TargetToolPoses()

        #find correct bin and its pose
            name = self.obj_data.name
            bin_pose = bin_origins[name]['pose']
            offset = pick_offset[name]['offset']
            rospy.loginfo("Current object is %s",obj_data.name)

            tool_msg.header.stamp =rospy.get_rostime()

           #compute pick_approach pose (%lift meter above the pick up position)
            tool_msg.pick_approach.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_approach.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - length 
            tool_msg.pick_approach.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2] + lift
            tool_msg.pick_approach.pose.orientation = self.obj_data.pose.orientation

            #compute pick pose
            tool_msg.pick_pose.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_pose.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - length
            tool_msg.pick_pose.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2]
            tool_msg.pick_pose.pose.orientation = self.obj_data.pose.orientation
    
            #compute pick_retreat pose
            tool_msg.pick_retreat.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_retreat.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - length
            tool_msg.pick_retreat.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2] + drop
            tool_msg.pick_retreat.pose.orientation = self.obj_data.pose.orientation
        
            #compute place pose
            tool_msg.place_pose.pose.position.x = bin_pose['xyz'][0]
            tool_msg.place_pose.pose.position.y = bin_pose['xyz'][1]
            tool_msg.place_pose.pose.position.z = bin_pose['xyz'][2] + drop
            tool_msg.place_pose.pose.orientation = self.obj_data.pose.orientation
    
            tool_msg.pick_approach.header.frame_id = 'world_frame'
            tool_msg.pick_approach.header.stamp = self.obj_data.detection_time + rospy.Duration(conveyor_time+approach_time)   
            tool_msg.pick_pose.header.frame_id = 'world_frame'
            tool_msg.pick_pose.header.stamp = self.obj_data.detection_time + rospy.Duration(conveyor_time+pick_time)
            tool_msg.pick_retreat.header.frame_id = 'world_frame'
            tool_msg.pick_retreat.header.stamp = self.obj_data.detection_time + rospy.Duration(conveyor_time+retreat_time)
            tool_msg.place_pose.header.frame_id = 'world_frame'
            tool_msg.place_pose.header.stamp = self.obj_data.detection_time + rospy.Duration(conveyor_time+place_time)
    
            #rospy.loginfo("Tool poses for object %s is:",obj_data.name)    
            rospy.loginfo("publishing tool poses:\n%s",tool_msg)
            self.tool_pose_pub.publish(tool_msg)
            return True
        except rospy.ROSInterruptException: 
            rospy.logerr("Failed to calculate.")
            return False

if __name__ == '__main__':
    try:    
        rospy.init_node('tool_planning_node')
        rate = rospy.Rate(10)
        if loadParameters():
            rospy.loginfo("Found parameters proceeding.")
        else:
            rospy.logerr("Fail to load parameters. exiting.")
            sys.exit(-1)
        
        tool_planner = ToolPlanner()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
