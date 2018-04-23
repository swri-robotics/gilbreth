#include "gilbreth_msgs/ObjectVoxel.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <XmlRpcException.h>
#include <array>
#include <boost/dynamic_bitset.hpp>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip> // setprecision
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

// Algorithm params
double down_sample(0.01);
double x_l, x_u, y_l, y_u, z_l, z_u;
bool print_detailed_info(false);
uint32_t g_min_cluster_size = 80;
uint32_t g_max_cluster_size = 25000;
double g_cluster_tolerance = 0.1;

ros::Publisher pub;   //publish PCD
ros::Publisher pub_v; //publish voxel

//pcd to voxel functions
struct Voxel {
  unsigned int x;
  unsigned int y;
  unsigned int z;
  Voxel() : x(0), y(0), z(0){};
  Voxel(const unsigned int _x, const unsigned int _y, const unsigned int _z) : x(_x), y(_y), z(_z){};
};

template <typename PointT>
const bool loadPointCloud(const std::string &file_path,
                          typename pcl::PointCloud<PointT> &cloud_out) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path.c_str(), cloud_out) == -1) {
    PCL_ERROR("Failed to load PCD file \n");
    return false;
  }

  return true;
}

// Get the linear index into a 1D array of voxels,
// for a voxel at location (ix, iy, iz).
const unsigned int getLinearIndex(const Voxel &voxel, const int grid_size) {
  return voxel.x * (grid_size * grid_size) + voxel.z * grid_size + voxel.y;
}

const Voxel getGridIndex(const pcl::PointXYZ &point, const pcl::PointXYZ &translate, const uint voxel_grid_size, const float scale) {
  // Needs to be signed to prevent overflow, because index can
  // be slightly negative which then gets rounded to zero.
  const int i = std::round(static_cast<float>(voxel_grid_size) * ((point.x - translate.x) / scale) - 0.5);
  const int j = std::round(static_cast<float>(voxel_grid_size) * ((point.y - translate.y) / scale) - 0.5);
  const int k = std::round(static_cast<float>(voxel_grid_size) * ((point.z - translate.z) / scale) - 0.5);
  return Voxel(i, j, k);
}

// Format a float number to ensure it always has at least one decimal place
// 0 --> 0.0
// 1.1 --> 1.1
// 1.10 --> 1.1
const std::string formatFloat(const float value) {
  std::stringstream ss;
  ss << std::setprecision(6) << std::fixed << value;
  std::string str;
  ss.str().swap(str);
  size_t last_zero_idx = str.find_last_not_of("0") + 1;
  if (last_zero_idx == str.length()) {
    // No trailing zeros
    return str;
  }
  if (str[last_zero_idx - 1] == '.') {
    // Last zero is after decimal point
    last_zero_idx += 1;
  }
  str.resize(last_zero_idx);
  return str;
}

bool loadParameter() {
  try {

    // General parameters
    XmlRpc::XmlRpcValue parameter_map;
    XmlRpc::XmlRpcValue camera_roi;
    std::map<std::string, bool> switch_map;
    ros::NodeHandle ph("~");

    ph.getParam("segmentation", parameter_map);
    down_sample = static_cast<double>(parameter_map["down_sample"]);
    g_min_cluster_size = static_cast<int>(parameter_map["min_cluster_size"]);
    g_max_cluster_size = static_cast<int>(parameter_map["max_cluster_size"]);
    g_cluster_tolerance = static_cast<double>(parameter_map["cluster_tolerance"]);

    ph.getParam("segmentation/camera_roi", camera_roi);
    x_l = camera_roi["x"][0];
    x_u = camera_roi["x"][1];
    y_l = camera_roi["y"][0];
    y_u = camera_roi["y"][1];
    z_l = camera_roi["z"][0];
    z_u = camera_roi["z"][1];

    ph.getParam("segmentation/switches", switch_map);
    print_detailed_info = static_cast<bool>(switch_map["print_detailed_info"]);
  } catch (XmlRpc::XmlRpcException &e) {
    ROS_ERROR("Failed to load segmentation parameters: %s", e.getMessage().c_str());
    return false;
  }
  return true;
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  ros::Time start_time = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_raw(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;

  pcl::fromROSMsg(*cloud_msg, *scene_raw);
  // Filter the input scene
  pass.setInputCloud(scene_raw);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_l, z_u);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_l, x_u);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_l, y_u);
  pass.filter(*scene_filtered);

  if (scene_filtered->points.size() <= 0) {
    ROS_WARN("Segmentation found no points after applying passthrough filters");
    return;
  }

  ROS_INFO_COND(print_detailed_info, "Segmentation will process cloud with %lu points", scene_filtered->size());

  // Downsample scene
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(scene_filtered);
  sor.setLeafSize(down_sample, down_sample, down_sample);
  sor.filter(*scene);

  ROS_INFO_COND(print_detailed_info, "Segmentation downsampled scene to %lu points with leafsize %f", scene->size(), down_sample);

  // Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(scene);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(g_cluster_tolerance);
  ec.setMinClusterSize(g_min_cluster_size);
  ec.setMaxClusterSize(g_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(scene);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    ROS_ERROR("Segmentation found no clusters after applying Clustering Extraction");
    return;
  }

  // Extracting and publishing cluster
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int voxel_grid_size = 32;
  for (int i = 0; i < cluster_indices.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 output;

    *indices = cluster_indices[i];
    extract.setInputCloud(scene);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cluster_cloud);
    //publish PCD
    pcl::toROSMsg(*cluster_cloud, output);
    output.header.stamp = cloud_msg->header.stamp;
    pub.publish(output);

    if (print_detailed_info) {
      double elapsed_time = (ros::Time::now() - start_time).toSec();
      ROS_INFO("Segmentation Found %lu clusters in %f seconds", cluster_indices.size(), elapsed_time);
    }
    //create and publish voxel
    start_time = ros::Time::now();
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
    // Calculate the scale factor so the longest side of the volume
    // is split into the desired number of voxels
    const float x_range = max_point.x - min_point.x;
    const float y_range = max_point.y - min_point.y;
    const float z_range = max_point.z - min_point.z;

    const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
    const float voxel_size = max_cloud_extent / (static_cast<float>(voxel_grid_size) - 1.0);
    std::cout << "voxel_size = " << voxel_size << std::endl;

    const float scale = (static_cast<float>(voxel_grid_size) * max_cloud_extent) / (static_cast<float>(voxel_grid_size) - 1.0);

    // Calculate the PointCloud's translation from the origin.
    // We need to subtract half the voxel size, because points
    // are located in the center of the voxel grid.
    float tx = min_point.x - voxel_size / 2.0;
    float ty = min_point.y - voxel_size / 2.0;
    float tz = min_point.z - voxel_size / 2.0;
    // Hack, change -0.0 to 0.0
    const float epsilon = 0.0000001;
    if ((tx > -epsilon) && (tx < 0.0)) {
      tx = -1.0 * tx;
    }
    if ((ty > -epsilon) && (ty < 0.0)) {
      ty = -1.0 * ty;
    }
    if ((tz > -epsilon) && (tz < 0.0)) {
      tz = -1.0 * tz;
    }
    const pcl::PointXYZ translate(tx, ty, tz);

    const unsigned int num_voxels = voxel_grid_size * voxel_grid_size * voxel_grid_size;

    // Voxelize the PointCloud into a linear array
    boost::dynamic_bitset<> voxels_bitset(num_voxels);
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cluster_cloud->begin(); it != cluster_cloud->end(); ++it) {
      const Voxel voxel = getGridIndex(*it, translate, voxel_grid_size, scale);
      const unsigned int idx = getLinearIndex(voxel, voxel_grid_size);
      voxels_bitset[idx] = 1;
    }
    gilbreth_msgs::ObjectVoxel voxel_data;
    std_msgs::Int32MultiArray dat;
    // fill out message:
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "x";
    dat.layout.dim[1].label = "y";
    dat.layout.dim[2].label = "z";
    dat.layout.dim[0].size = voxel_grid_size;
    dat.layout.dim[1].size = voxel_grid_size;
    dat.layout.dim[2].size = voxel_grid_size;
    dat.layout.dim[0].stride = voxel_grid_size * voxel_grid_size * voxel_grid_size;
    dat.layout.dim[1].stride = voxel_grid_size * voxel_grid_size;
    dat.layout.dim[1].stride = voxel_grid_size;
    dat.layout.data_offset = 0;
    std::vector<int> vec(voxel_grid_size * voxel_grid_size * voxel_grid_size, 0);
    for (int i = 0; i < voxel_grid_size * voxel_grid_size * voxel_grid_size; i++)
      vec[i] = voxels_bitset[i];
    dat.data = vec;
    voxel_data.voxel = dat;
    voxel_data.header.stamp = cloud_msg->header.stamp;
    pub_v.publish(voxel_data);
    double elapsed_time = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM("covert PCD to voxel time is " << elapsed_time << " seconds.");
  }
}

int main(int argc, char **argv) {
  // define PCD file subscribed
  // Initialize ROS
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;

  if (!loadParameter()) {
    return -1;
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("scene_point_cloud", 1, cloudCb);
  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("segmentation_result", 10);
  pub_v = nh.advertise<gilbreth_msgs::ObjectVoxel>("voxel_data", 10);
  ROS_INFO("Segmentation Node subscribed to %s", pub.getTopic().c_str());
  ROS_INFO("Segmentation Node Ready ...");
  // Spin

  ros::spin();
  //test
}
