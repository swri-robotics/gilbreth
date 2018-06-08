#include "gilbreth_msgs/ObjectDetection.h"
#include "gilbreth_msgs/ObjectType.h"
#include <cstdio>
#include <ctime>
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

static const std::string WORLD_FRAME = "world";
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

class AlignmentClass {
public:
  explicit AlignmentClass(ros::NodeHandle &nh) {
    // initializing parameters
    down_sample = 0.01;
    print_detailed_info = false;
    iterations = 10;
    pub_tf = nh.advertise<gilbreth_msgs::ObjectDetection>("recognition_result_world", 10);
    scene.reset(new pcl::PointCloud<PointType>());
    loadParameter();
    loadModel();
  }
  void loadParameter() {
    // General parameters
    std::map<std::string, float> parameter_map;
    std::map<std::string, bool> switch_map;
    ros::NodeHandle ph("~");
    ph.getParam("parameters", parameter_map);
    ph.getParam("switches", switch_map);
    down_sample = parameter_map["down_sample"];
    print_detailed_info = switch_map["print_detailed_info"];
    iterations = parameter_map["iteration"];
  }

  void loadModel() {
    // Load model settings
    XmlRpc::XmlRpcValue model_map;
    std::string package_path;
    ros::NodeHandle ph("~");
    ph.getParam("part_list", model_map);
    ph.getParam("package_path", package_path);

    ROS_INFO("Loading Point Cloud Models");
    std::vector<pcl::PointCloud<PointType>::Ptr> model_raw_list;
    for (int i = 0; i < model_map.size(); i++) {
      pcl::PointCloud<PointType>::Ptr model_raw(new pcl::PointCloud<PointType>());
      std::string model_path = model_map[i]["path"];
      if (pcl::io::loadPCDFile(package_path + model_path, *model_raw) < 0) {
        ROS_ERROR("Error loading model cloud.");
        return;
      }
      model_raw_list.push_back(model_raw);
      model_name.push_back(model_map[i]["name"]);
      std::vector<double> pick_pose_sub;
      double pick_pose_sub_element;
      for (int j = 0; j < model_map[i]["pick_pose"].size(); j++) {
        pick_pose_sub_element = model_map[i]["pick_pose"][j];
        pick_pose_sub.push_back(pick_pose_sub_element);
      }
      pick_pose.push_back(pick_pose_sub);
    }

    // Downsample models
    ROS_INFO("Preparing Point Cloud Models");
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    for (int i = 0; i < model_raw_list.size(); i++) {
      pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
      if (down_sample > 0) {
        sor.setInputCloud(model_raw_list[i]);
        sor.setLeafSize(down_sample, down_sample, down_sample);
        sor.filter(*model);
      } else
        model = model_raw_list[i];
      model_list.push_back(model);
    }
  }

  void objectCallBack(const gilbreth_msgs::ObjectType::ConstPtr &object_type) {
    std::clock_t start;
    double duration;
    Result result;
    result.item_name = model_name[object_type->type];
    result.item_id = object_type->type;
    // Load scene
    pcl::fromROSMsg(object_type->pcd, *scene);
    // Use ICP to align model to scene
    start = std::clock();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(model_list[result.item_id]);
    icp.setInputTarget(scene);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    // Transform pick up point from model to scene
    pcl::PointCloud<PointType>::Ptr pick_point_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr rotated_pick_point_cloud(new pcl::PointCloud<PointType>());
    pcl::PointXYZ pick_point;
    pick_point.x = pick_pose[result.item_id][0];
    pick_point.y = pick_pose[result.item_id][1];
    pick_point.z = pick_pose[result.item_id][2];
    pick_point_cloud->push_back(pick_point);
    pcl::transformPointCloud(*pick_point_cloud, *rotated_pick_point_cloud, icp_transformation);
    // Generate output message
    gilbreth_msgs::ObjectDetection data;
    gilbreth_msgs::ObjectDetection data_tf;
    geometry_msgs::PointStamped sensor_point;
    geometry_msgs::PointStamped world_point;

    tf::Quaternion q;
    q.setEuler(pick_pose[result.item_id][4], pick_pose[result.item_id][3], pick_pose[result.item_id][5]);
    data.name = result.item_name;
    data.pose.position.x = rotated_pick_point_cloud->points[0].x;
    data.pose.position.y = rotated_pick_point_cloud->points[0].y;
    data.pose.position.z = rotated_pick_point_cloud->points[0].z;

    data.pose.orientation.x = q.getX();
    data.pose.orientation.y = q.getY();
    data.pose.orientation.z = q.getZ();
    data.pose.orientation.w = q.getW();
    // Transform point to world coordination
    sensor_point.point.x = data.pose.position.x;
    sensor_point.point.y = data.pose.position.y;
    sensor_point.point.z = data.pose.position.z;
    sensor_point.header.frame_id = "depth_camera_camera_link_optical";
    listener.transformPoint(WORLD_FRAME, sensor_point, world_point);

    data_tf.name = data.name;
    data_tf.pose.position.x = world_point.point.x;
    data_tf.pose.position.y = world_point.point.y;
    data_tf.pose.position.z = world_point.point.z;
    data_tf.pose.orientation.x = q.getX();
    data_tf.pose.orientation.y = q.getY();
    data_tf.pose.orientation.z = q.getZ();
    data_tf.pose.orientation.w = q.getW();
    data_tf.detection_time = object_type->detection_time;
    data_tf.header.stamp = ros::Time::now();
    data_tf.header.frame_id = WORLD_FRAME;
    pub_tf.publish(data_tf);

    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    ROS_INFO_STREAM("Object: " << result.item_name << ".");
    ROS_INFO_STREAM("Alignment runtime: " << duration << " seconds.");
  }

private:
  struct Result {
    std::string item_name;
    int item_id;
  };
  ros::Publisher pub_tf;
  std::vector<std::vector<double>> pick_pose;
  std::vector<std::string> model_name;
  std::vector<pcl::PointCloud<PointType>::Ptr> model_list;
  pcl::PointCloud<PointType>::Ptr scene;
  tf::TransformListener listener;
  // Algorithm params
  float descr_dis_thrd;
  float descr_rad;
  float down_sample;
  float min_sample_distance;
  float max_correspondence_distance;
  int nr_iterations;
  bool visualizer;
  bool icp;
  float cg_size;
  float cg_thresh;
  bool print_detailed_info;
  float key_point_sampling;
  int k_nearest_neighbors;
  int iterations;
};

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "alignment_node");
  ros::NodeHandle nh;
  AlignmentClass alignmentNode(nh);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_2 = nh.subscribe<gilbreth_msgs::ObjectType>("object_type", 100, &AlignmentClass::objectCallBack, &alignmentNode);
  ROS_INFO("Alignment Node Ready ...");
  // Spin
  ros::spin();
}
