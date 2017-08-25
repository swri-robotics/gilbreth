#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Algorithm params
float down_sample_(0.01);

ros::Publisher pub;

void parseCommandLine(int argc, char *argv[]) {
  // General parameters
  pcl::console::parse_argument(argc, argv, "--down_sample", down_sample_);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, int argc,
              char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_raw(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PassThrough<pcl::PointXYZ> pass;

  parseCommandLine(argc, argv);
  pcl::fromROSMsg(*cloud_msg, *scene_raw);

  // Filter the input scene
  pass.setInputCloud(scene_raw);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.19);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.69, 0.69);
  pass.filter(*scene_filtered);

  pass.setInputCloud(scene_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.21, 0.45);
  pass.filter(*scene_filtered);

  // Downsample scene
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(scene_filtered);
  sor.setLeafSize(down_sample_, down_sample_, down_sample_);
  sor.filter(*scene);

  // Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(scene);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1); // 10cm
  ec.setMinClusterSize(80);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(scene);
  ec.extract(cluster_indices);

  std::cerr << "Cluster number is : " << cluster_indices.size() << std::endl;

  // Publish cluster clouds
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2 output;

  for (int i = 0; i < cluster_indices.size(); i++) {
    *indices = cluster_indices[i];
    extract.setInputCloud(scene);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cluster_cloud);

    pcl::toROSMsg(*cluster_cloud, output);
    output.header.stamp = cloud_msg->header.stamp;
    pub.publish(output);

    std::cerr << "Publish cluster " << i << std::endl;
  }
}

int main(int argc, char **argv) {
  // define PCD file subscribed
  // Initialize ROS
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>(
      "scene_point_cloud", 1, boost::bind(cloud_cb, _1, argc, argv));
  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("segmentation_result", 10);

  // Spin

  ros::spin();
}
