#include "gilbreth_msgs/ObjectDetection.h"
#include <cstdio>
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

// Algorithm params
float descr_rad(0.02f);
float down_sample(0.01);
float min_sample_distance(0.025f);
float max_correspondence_distance(0.01f * 0.01f);
int nr_iterations(20);
bool visualizer(false);

ros::Publisher pub_tf;

struct Result {
  std::string item_name;
  int item_id;
  float fitness_score;
  pcl::PointXYZ pick_point;
  Eigen::Matrix4f final_transformation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void loadParameter() {
  // General parameters
  std::map<std::string, float> parameter_map;
  std::map<std::string, bool> switch_map;
  ros::NodeHandle ph("~");
  ph.getParam("parameters", parameter_map);
  ph.getParam("switches", switch_map);
  descr_rad = parameter_map["descr_rad"];
  down_sample = parameter_map["down_sample"];
  min_sample_distance = parameter_map["min_sample_distance"];
  max_correspondence_distance = parameter_map["max_correspondence_distance"];
  nr_iterations = parameter_map["nr_iterations"];
  visualizer = switch_map["visualizer"];
}

void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const tf::TransformListener &listener, int argc, char **argv) {

  // Load model settings
  XmlRpc::XmlRpcValue model_map;
  std::vector<std::vector<double> > pick_pose;
  std::string package_path;
  ros::NodeHandle ph("~");
  ph.getParam("part_list", model_map);
  ph.getParam("package_path", package_path);

  pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PassThrough<PointType> pass;
  pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  std::vector<pcl::PointCloud<PointType>::Ptr> model_raw_list;
  std::vector<std::string> model_name;
  for (int i = 0; i < model_map.size(); i++) {
    pcl::PointCloud<PointType>::Ptr model_raw(new pcl::PointCloud<PointType>());
    std::string model_path = model_map[i]["path"];

    if (pcl::io::loadPCDFile(package_path + model_path, *model_raw) < 0) {
      std::cout << "Error loading model cloud." << std::endl;
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

  // Load scene
  pcl::fromROSMsg(*cloud_msg, *scene);

  // Downsample models
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  std::vector<pcl::PointCloud<PointType>::Ptr> model_list;
  for (int i = 0; i < model_raw_list.size(); i++) {
    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
    sor.setInputCloud(model_raw_list[i]);
    sor.setLeafSize(down_sample, down_sample, down_sample);
    sor.filter(*model);
    model_list.push_back(model);
  }

  //  Compute Feature
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  std::vector<pcl::PointCloud<NormalType>::Ptr> model_normals_list;
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> model_features_list;

  norm_est.setSearchMethod(tree);
  norm_est.setRadiusSearch(descr_rad);
  norm_est.setInputCloud(scene);
  norm_est.compute(*scene_normals);

  fpfh_est.setSearchMethod(tree);
  fpfh_est.setRadiusSearch(descr_rad);
  fpfh_est.setInputCloud(scene);
  fpfh_est.setInputNormals(scene_normals);
  fpfh_est.compute(*scene_features);

  for (int i = 0; i < model_list.size(); i++) {
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
    norm_est.setRadiusSearch(descr_rad);
    norm_est.setSearchMethod(tree);
    norm_est.setInputCloud(model_list[i]);
    norm_est.compute(*model_normals);
    model_normals_list.push_back(model_normals);
  }

  for (int i = 0; i < model_list.size(); i++) {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_est.setInputCloud(model_list[i]);
    fpfh_est.setInputNormals(model_normals_list[i]);
    fpfh_est.compute(*model_features);
    model_features_list.push_back(model_features);
  }

  // ICP
  std::vector<pcl::PointIndices> recognized_indices;
  pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr recognized_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  std::vector<Result, Eigen::aligned_allocator<Result> > results_temp;
  Result result;

  // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA)
  // algorithm
  sac_ia_.setMinSampleDistance(min_sample_distance);
  sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance);
  sac_ia_.setMaximumIterations(nr_iterations);

  int min_index;

  sac_ia_.setInputTarget(scene);
  sac_ia_.setTargetFeatures(scene_features);
  results_temp.resize(model_list.size());
  float best_score = 1;
  for (int j = 0; j < model_list.size(); j++) {
    sac_ia_.setInputSource(model_list[j]);
    sac_ia_.setSourceFeatures(model_features_list[j]);
    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align(registration_output);
    results_temp[j].item_name = model_name[j];
    results_temp[j].item_id = j;
    results_temp[j].fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance);
    results_temp[j].final_transformation = sac_ia_.getFinalTransformation();
    std::cerr << "model " << j << " FitnessScore " << results_temp[j].fitness_score << std::endl;

    if (results_temp[j].fitness_score < best_score) {
      min_index = j;
      best_score = results_temp[j].fitness_score;
    }
  }
  result = results_temp[min_index];
  std::cerr << "Item_Name: " << result.item_name << std::endl;

  pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*model_list[result.item_id], *rotated_model, result.final_transformation);
  // Transform pick up point from model to scene
  pcl::PointCloud<PointType>::Ptr pick_point_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr rotated_pick_point_cloud(new pcl::PointCloud<PointType>());
  pcl::PointXYZ pick_point;
  pick_point.x = pick_pose[result.item_id][0];
  pick_point.y = pick_pose[result.item_id][1];
  pick_point.z = pick_pose[result.item_id][2];
  pick_point_cloud->push_back(pick_point);
  pcl::transformPointCloud(*pick_point_cloud, *rotated_pick_point_cloud, result.final_transformation);

  //  Visualize recognition result
  if (visualizer) {
    pcl::visualization::PCLVisualizer viewer("recognition_result");
    viewer.addCoordinateSystem(0.1, -1, -0.1, 1.0);
    viewer.setCameraPosition(-0.088, 0.3409, -0.082791, -0.0282727, 0.239596, 0.716447, -0.00732219, -0.99209, -0.125313);
    viewer.setCameraClipDistances(0.00300683, 3.00683);
    viewer.setCameraFieldOfView(0.5236);
    viewer.removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 137, 137, 137);
    viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
    viewer.addPointCloud(rotated_model, rotated_model_color_handler, "rotated_model");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> pick_points_color_handler(rotated_pick_point_cloud, 0, 0, 255);
    viewer.addPointCloud(rotated_pick_point_cloud, pick_points_color_handler, "pick_points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "pick_points");
    viewer.spinOnce();
  }
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
  sensor_point.header.frame_id = "sensor_frame";
  listener.transformPoint("world_frame", sensor_point, world_point);

  data_tf.name = data.name;
  data_tf.pose.position.x = world_point.point.x;
  data_tf.pose.position.y = world_point.point.y;
  data_tf.pose.position.z = world_point.point.z;
  data_tf.pose.orientation.x = q.getX();
  data_tf.pose.orientation.y = q.getY();
  data_tf.pose.orientation.z = q.getZ();
  data_tf.pose.orientation.w = q.getW();
  data_tf.detection_time = cloud_msg->header.stamp;
  data_tf.header.stamp = ros::Time::now();
  data_tf.header.frame_id = "world_frame";
  pub_tf.publish(data_tf);
}

int main(int argc, char **argv) {
  // define PCD file subscribed
  // Initialize ROS
  ros::init(argc, argv, "recognition_node");
  ros::NodeHandle nh;
  loadParameter();
  tf::TransformListener listener;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("segmentation_result", 100, boost::bind(cloudCallBack, _1, boost::ref(listener), argc, argv));
  // ROS publisher
  pub_tf = nh.advertise<gilbreth_msgs::ObjectDetection>("recognition_result_world", 10);
  // Spin
  ros::spin();
}
