#!/usr/bin/env python
## a python script compute the tool poses
import rospy
import time
import copy
from gilbreth_msgs.msg import ObjectDetection
from gilbreth_msgs.msg import TargetToolPoses

TOOL_POSE_TOPIC='/gilbreth/target_tool_poses'
OBJ_DETECTION_TOPIC = '/recognition_result_world'
GENERAL_PARAM = '/gilbreth/tool_plan'

class PlanningProperties():
    def __init__(self):
        self.SUCCESS = False
        try:           
            self.params = rospy.get_param(GENERAL_PARAM)
            ## environment params: lift and drop distance; conveyor belt velocity and duration
            env_param = self.params['env_param']
            self.lift = env_param['lift_distance']
            self.drop = env_param['drop_distance']
            self.vel = env_param['conveyor_power'] * env_param['conveyor_velocity']
            self.length = env_param['conveyor_belt_length']
            self.conveyor_time = self.length / self.vel
            rospy.loginfo("lift distance: %f\ndrop distance: %f\nconveyor belt speed %f\nconveyor belt length: %f\ntime befor pickup: %f\n", self.lift, self.drop, self.vel, self.length, self.conveyor_time)

            ## timer params: the deadline of each motion stage
            time_param = self.params['pose_timer']
            self.approach_time = time_param['approach_time']
            self.pick_time = time_param['pick_time']
            self.retreat_time = time_param['retreat_time']
            self.place_time = time_param['place_time']
            rospy.loginfo("timers are %f, %f, %f, %f\n", self.approach_time, self.pick_time, self.retreat_time, self.place_time)
            ## poses
            self.bin_origins = self.params['bin_origin']
            self.robot_home_pose = self.params['robot_origin']
            self.pick_offset = self.params['pick_offset']
            self.SUCCESS = True
            rospy.loginfo ("Successfully loaded parameters.")
        except rospy.ROSInterruptException as e: 
            rospy.logerr("Failed to load parameters.\n %s",e)
            

class ToolPlanner():
    def __init__(self):
        self.tool_pose_pub = rospy.Publisher(TOOL_POSE_TOPIC, TargetToolPoses, queue_size=10)
        self.obj_dect_sub = rospy.Subscriber(OBJ_DETECTION_TOPIC, ObjectDetection, self.obj_detect_callback)
        self.obj_data = ObjectDetection()
        self.property = PlanningProperties()

    def obj_detect_callback(self,data):
        if data is not None:
            self.obj_data = copy.deepcopy(data)
            rospy.loginfo("Received new object information.")
            start_time = time.time()
            if self.compute_tool_poses():
                rospy.loginfo("Successfully computed tool poses. Run time is %f seconds",(time.time()-start_time))
            else:
                rospy.logerr("Failed to compute tool poses")
            self.obj_data = None
    
    def compute_tool_poses(self):
        try: 
            tool_msg = TargetToolPoses()

            #find correct bin and its pose
            name = self.obj_data.name
            bin_pose = self.property.bin_origins[name]['pose']
            offset = self.property.pick_offset[name]['offset']
            tool_msg.header.stamp =rospy.get_rostime()

            #compute pick_approach pose (%lift meter above the pick up position)
            tool_msg.pick_approach.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_approach.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - self.property.length 
            tool_msg.pick_approach.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2] + self.property.lift
            tool_msg.pick_approach.pose.orientation = self.obj_data.pose.orientation

            #compute pick pose
            tool_msg.pick_pose.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_pose.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - self.property.length
            tool_msg.pick_pose.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2]
            tool_msg.pick_pose.pose.orientation = self.obj_data.pose.orientation
    
            #compute pick_retreat pose
            tool_msg.pick_retreat.pose.position.x = self.obj_data.pose.position.x + offset['xyz'][0]
            tool_msg.pick_retreat.pose.position.y = self.obj_data.pose.position.y + offset['xyz'][1] - self.property.length
            tool_msg.pick_retreat.pose.position.z = self.obj_data.pose.position.z + offset['xyz'][2] + self.property.drop
            tool_msg.pick_retreat.pose.orientation = self.obj_data.pose.orientation
        
            #compute place pose
            tool_msg.place_pose.pose.position.x = bin_pose['xyz'][0]
            tool_msg.place_pose.pose.position.y = bin_pose['xyz'][1]
            tool_msg.place_pose.pose.position.z = bin_pose['xyz'][2] + self.property.drop
            tool_msg.place_pose.pose.orientation = self.obj_data.pose.orientation
    
            tool_msg.pick_approach.header.frame_id = 'world_frame'
            total_moving_length = self.property.length - tool_msg.pick_approach.pose.position.y 
            total_time = total_moving_length / self.property.vel
            tool_msg.pick_approach.header.stamp = self.obj_data.detection_time + rospy.Duration(total_time + self.property.approach_time)   
            tool_msg.pick_pose.header.frame_id = 'world_frame'
            tool_msg.pick_pose.header.stamp = self.obj_data.detection_time + rospy.Duration(total_time + self.property.pick_time)
            tool_msg.pick_retreat.header.frame_id = 'world_frame'
            tool_msg.pick_retreat.header.stamp = self.obj_data.detection_time + rospy.Duration(total_time + self.property.retreat_time)
            tool_msg.place_pose.header.frame_id = 'world_frame'
            tool_msg.place_pose.header.stamp = self.obj_data.detection_time + rospy.Duration(total_time + self.property.place_time)
     
            rospy.loginfo("publishing tool poses.")
            self.tool_pose_pub.publish(tool_msg)
            return True
        except rospy.ROSInterruptException: 
            return False

if __name__ == '__main__':
    try:    
        rospy.init_node('tool_planning_node')
        rate = rospy.Rate(10)
        planning_properties = PlanningProperties()
        if planning_properties.SUCCESS:
            rospy.loginfo("Found parameters. Proceeding.")
        else:
            rospy.logerr("Fail to load parameters. Exiting.")
            sys.exit(-1)
        
        tool_planner = ToolPlanner()
        tool_planner.property = planning_properties
        
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
