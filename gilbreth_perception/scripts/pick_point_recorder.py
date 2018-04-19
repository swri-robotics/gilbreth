#!/usr/bin/env python2

import sys
import rospy
import math
import geometry_msgs.msg
import copy
import csv
import time
import datetime
from gilbreth_msgs.msg._ObjectDetection import ObjectDetection

PICK_POINT_TOPIC = 'recognition_result_world'

class PickPointRecorder:
  
  def __init__(self):
    self.pick_points_dict_ = {}
    
    self.trajectory_sub = rospy.Subscriber(PICK_POINT_TOPIC, ObjectDetection, self.pickPointMsgCallback)

      
  def pickPointMsgCallback(self,msg):
      
    part_name = msg.name
    part_point_list = []
    if part_name not in self.pick_points_dict_:
      self.pick_points_dict_[part_name] = []        
    
    part_point_list = self.pick_points_dict_[part_name]
    
    # storing point
    part_point_list.append(msg.pose.position)
    rospy.loginfo('Saved "%s" point %s'%(part_name,str(msg.pose.position)))
    
  def onShutdown(self):
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y%m%d_%H%M%S')
    
    file_name = 'results_' + st + '.csv'
    with open(file_name,'w') as csv_file:
      csv_writer = csv.writer(csv_file,delimiter = ' ')
      
      point_num_field = 'point#'
      object_names = self.pick_points_dict_.keys() 
      field_names = [point_num_field] + object_names 
      csv_writer = csv.DictWriter(csv_file, fieldnames=field_names)
      csv_writer.writeheader()
      
      index = 0      
      proceed = True
      while True:
        
        out_dict = {point_num_field: 'point' + str(index)}
        proceed = False
        for name in object_names:        
          
          points = self.pick_points_dict_[name]   
          
          if len(points) > index:
            out_dict[name] = points[index].z
            proceed = True
          else:
            out_dict[name] = ' '
            
        if not proceed:
          break
            
        csv_writer.writerow(out_dict)     
        index+=1
        
      rospy.loginfo('Saving results to file %s'%(file_name))     

    

if __name__ == '__main__':
  
  '''
  This node listens to the recognition pick points results message 
  and writes out the pick point z value for later analysis
  '''
    
  rospy.init_node('pick_point_analyser')
  point_recorder = PickPointRecorder()
  rospy.on_shutdown(point_recorder.onShutdown)
  rospy.spin()
    
