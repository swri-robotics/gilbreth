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
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

// Algorithm params
static const int VOXEL_GRID_SIZE = 32;

ros::Publisher pub;

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

const Voxel getGridIndex(const pcl::PointXYZ &point, const pcl::PointXYZ &translate, const uint VOXEL_GRID_SIZE, const float scale) {
  // Needs to be signed to prevent overflow, because index can
  // be slightly negative which then gets rounded to zero.
  const int i = std::round(static_cast<float>(VOXEL_GRID_SIZE) * ((point.x - translate.x) / scale) - 0.5);
  const int j = std::round(static_cast<float>(VOXEL_GRID_SIZE) * ((point.y - translate.y) / scale) - 0.5);
  const int k = std::round(static_cast<float>(VOXEL_GRID_SIZE) * ((point.z - translate.z) / scale) - 0.5);
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

void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *scene);
  //create and publish voxel
  ros::Time start_time = ros::Time::now();
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::getMinMax3D(*scene, min_point, max_point);
  // Calculate the scale factor so the longest side of the volume
  // is split into the desired number of voxels
  const float x_range = max_point.x - min_point.x;
  const float y_range = max_point.y - min_point.y;
  const float z_range = max_point.z - min_point.z;

  const float max_cloud_extent = std::max(std::max(x_range, y_range), z_range);
  const float voxel_size = max_cloud_extent / (static_cast<float>(VOXEL_GRID_SIZE) - 1.0);
  ROS_INFO("voxel size is %f", voxel_size);

  const float scale = (static_cast<float>(VOXEL_GRID_SIZE) * max_cloud_extent) / (static_cast<float>(VOXEL_GRID_SIZE) - 1.0);

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

  const unsigned int num_voxels = std::pow(VOXEL_GRID_SIZE,3);

  // Voxelize the PointCloud into a linear array
  boost::dynamic_bitset<> voxels_bitset(num_voxels);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = scene->begin(); it != scene->end(); ++it) {
    const Voxel voxel = getGridIndex(*it, translate, VOXEL_GRID_SIZE, scale);
    const unsigned int idx = getLinearIndex(voxel, VOXEL_GRID_SIZE);
    voxels_bitset[idx] = 1;
  }
  gilbreth_msgs::ObjectVoxel voxel_data;
  std_msgs::Int32MultiArray dat;
  // fill voxel into message:
  dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
  dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
  dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
  dat.layout.dim[0].label = "x";
  dat.layout.dim[1].label = "y";
  dat.layout.dim[2].label = "z";
  dat.layout.dim[0].size = VOXEL_GRID_SIZE;
  dat.layout.dim[1].size = VOXEL_GRID_SIZE;
  dat.layout.dim[2].size = VOXEL_GRID_SIZE;
  dat.layout.dim[0].stride = VOXEL_GRID_SIZE * VOXEL_GRID_SIZE * VOXEL_GRID_SIZE;
  dat.layout.dim[1].stride = VOXEL_GRID_SIZE * VOXEL_GRID_SIZE;
  dat.layout.dim[1].stride = VOXEL_GRID_SIZE;
  dat.layout.data_offset = 0;
  std::vector<int> vec(VOXEL_GRID_SIZE * VOXEL_GRID_SIZE * VOXEL_GRID_SIZE, 0);
  for (int i = 0; i < VOXEL_GRID_SIZE * VOXEL_GRID_SIZE * VOXEL_GRID_SIZE; i++)
    vec[i] = voxels_bitset[i];
  dat.data = vec;
  voxel_data.voxel = dat;
  voxel_data.header.stamp = ros::Time::now();
  voxel_data.detection_time = cloud_msg->header.stamp;
  //fill pcd into msg
  sensor_msgs::PointCloud2 pcd_output;
  pcl::toROSMsg(*scene, pcd_output);
  voxel_data.pcd = pcd_output;
  //publish
  pub.publish(voxel_data);
  double elapsed_time = (ros::Time::now() - start_time).toSec();
  ROS_INFO("covert PCD to voxel time is %f seconds", elapsed_time);
}

int main(int argc, char **argv) {
  // define PCD file subscribed
  // Initialize ROS
  ros::init(argc, argv, "voxelizer_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("segmentation_result", 1, cloudCb);
  // ROS publisher
  pub = nh.advertise<gilbreth_msgs::ObjectVoxel>("voxel_data", 10);
  ROS_INFO("Voxelizer Node subscribed to %s", pub.getTopic().c_str());
  ROS_INFO("Voxelizer Node Ready ...");
  // Spin

  ros::spin();
}
