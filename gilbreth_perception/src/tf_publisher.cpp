#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_tf_publisher");
  ros::NodeHandle n;
  float x =atof(argv[1]);
  float y =atof(argv[2]);
  float z =atof(argv[3]);
  float w =atof(argv[4]);
  

  tf::TransformBroadcaster broadcaster;
  ros::Rate r(100);
 while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(x,y,z,w), tf::Vector3(1.3, 3.3, 2.06)),
        ros::Time::now(),"world_frame", "sensor_frame"));
       r.sleep();
    
  }
}
