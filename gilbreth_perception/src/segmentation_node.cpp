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
float down_sample(0.01);
double x_l, x_u, y_l, y_u, z_l, z_u;

ros::Publisher pub;

void loadParameter() {
  // General parameters
  std::map<std::string, float> parameter_map;
  XmlRpc::XmlRpcValue camera_scope;
  ros::NodeHandle ph("~");
  ph.getParam("parameters", parameter_map);
  down_sample = parameter_map["down_sample"];
  ph.getParam("camera_scope", camera_scope);
  x_l=camera_scope["x"][0];
  x_u=camera_scope["x"][1];
  y_l=camera_scope["y"][0];
  y_u=camera_scope["y"][1];
  z_l=camera_scope["z"][0];
  z_u=camera_scope["z"][1];
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, int argc, char **argv) {
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

  // Downsample scene
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(scene_filtered);
  sor.setLeafSize(down_sample, down_sample, down_sample);
  sor.filter(*scene);

  // Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
  loadParameter();
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("scene_point_cloud", 1, boost::bind(cloudCb, _1, argc, argv));
  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("segmentation_result", 10);

  // Spin

  ros::spin();
}
