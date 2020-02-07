#include "gilbreth_msgs/ObjectDetection.h"
#include <cstdio>
#include <ctime>
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
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <XmlRpcException.h>
#include <boost/format.hpp>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
using ModelRecognizer = pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame>;
using ModelRecognizerPtr = std::shared_ptr<ModelRecognizer>;

class RecognitionClass {
public:
  explicit RecognitionClass(ros::NodeHandle &nh):
    nh_(nh)
  {
    // initializing parameters
    descr_dis_thrd = 0.25;
    descr_rad_cg = 0.02;
    descr_rad_icp = 0.02;
    down_sample = 0.01;
    min_sample_distance = 0.025;
    max_correspondence_distance = 0.01 * 0.01;
    nr_iterations = 20;
    visualizer = false;
    icp = true;
    cg_size = 0.05;
    cg_thresh = 8.0;
    print_detailed_info = false;
    key_point_sampling = 0.006;
    k_nearest_neighbors= 10;
    iterations=10;
  }

  bool run()
  {

    if(!loadParameter() || !loadModel())
    {
      return false;
    }

    cloud_subs_= nh_.subscribe<sensor_msgs::PointCloud2>("segmentation_result", 1, &RecognitionClass::cloudCallBack, this);
    pub_tf = nh_.advertise<gilbreth_msgs::ObjectDetection>("recognition_result_world", 10);

    return true;
  }

  bool loadParameter() {

    // General parameters
    try
    {
      XmlRpc::XmlRpcValue parameter_map;
      XmlRpc::XmlRpcValue switch_map;
      ros::NodeHandle ph("~");
      ph.getParam("recognition", parameter_map);
      ph.getParam("recognition/switches", switch_map);
      descr_rad_cg = static_cast<double>(parameter_map["descr_rad_cg"]);
      descr_rad_icp = static_cast<double>(parameter_map["descr_rad_icp"]);
      down_sample = static_cast<double>(parameter_map["down_sample"]);
      min_sample_distance = static_cast<double>(parameter_map["min_sample_distance"]);
      max_correspondence_distance = static_cast<double>(parameter_map["max_correspondence_distance"]);
      nr_iterations = static_cast<int>(parameter_map["nr_iterations"]);
      cg_size = static_cast<double>(parameter_map["cg_size"]);
      cg_thresh = static_cast<double>(parameter_map["cg_thresh"]);
      icp = static_cast<bool>(switch_map["ICP"]);
      print_detailed_info = static_cast<bool>(switch_map["print_detailed_info"]);
      descr_dis_thrd = static_cast<double>(parameter_map["descr_dis_thrd"]);
      key_point_sampling = static_cast<double>(parameter_map["key_point_sampling"]);
      k_nearest_neighbors = static_cast<int>(parameter_map["k_nearest_neighbors"]);
      iterations=static_cast<int>(parameter_map["iteration"]);
    }
    catch(XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("Recognition failed to load algorithm parameters: %s",e.getMessage().c_str());
      return false;
    }
    return true;
  }

  bool loadModel()
  {
    try
    {
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
        if (pcl::io::loadPCDFile(package_path + model_path, *model_raw) < 0)
        {
          ROS_ERROR("Recognition encountered Error loading model cloud.");
          return false;
        }
        model_raw_list.push_back(model_raw);
        model_names_.push_back(model_map[i]["name"]);
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
      for (int i = 0; i < model_raw_list.size(); i++)
      {
        pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
        if (down_sample > 0)
        {
          sor.setInputCloud(model_raw_list[i]);
          sor.setLeafSize(down_sample, down_sample, down_sample);
          sor.filter(*model);
        }
        else
        {
          model = model_raw_list[i];
        }
        model_list.push_back(model);
      }

      // if use ICP
      ROS_INFO("Loading Recognition Method Parameters");
      if (icp) {
        loadICPConfig();
      }
      // if use correspondence grouping
      else {
        loadCGConfig();
      }
    }
    catch(XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("Recognition failed to load model parameters: %s",e.getMessage().c_str());
      return false;
    }

    return true;
  }

  void loadICPConfig() {
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    std::vector<pcl::PointCloud<NormalType>::Ptr> model_normals_list;

    for (int i = 0; i < model_list.size(); i++) {
      pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
      norm_est.setRadiusSearch(descr_rad_icp);
      norm_est.setSearchMethod(tree);
      norm_est.setInputCloud(model_list[i]);
      norm_est.compute(*model_normals);
      model_normals_list.push_back(model_normals);
    }
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    for (int i = 0; i < model_list.size(); i++) {
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features(new pcl::PointCloud<pcl::FPFHSignature33>);
      fpfh_est.setSearchMethod(tree);
      fpfh_est.setRadiusSearch(descr_rad_icp);
      fpfh_est.setInputCloud(model_list[i]);
      fpfh_est.setInputNormals(model_normals_list[i]);
      fpfh_est.compute(*model_features);
      model_features_list.push_back(model_features);
    }
  }

  void loadCGConfig() {
    //Compute Normals
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    std::vector<pcl::PointCloud<NormalType>::Ptr> model_normals_list;

    for (int i = 0; i < model_list.size(); i++) {
      pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
      norm_est.setKSearch(k_nearest_neighbors);
      norm_est.setSearchMethod(tree);
      norm_est.setInputCloud(model_list[i]);
      norm_est.compute(*model_normals);
      model_normals_list.push_back(model_normals);
    }
    //Extract Keypoints
    pcl::UniformSampling<PointType> uniform_sampling;
    for (int i = 0; i < model_list.size(); i++) {
      pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
      uniform_sampling.setInputCloud(model_list[i]);
      uniform_sampling.setRadiusSearch(key_point_sampling);
      uniform_sampling.filter(*model_keypoints);
      model_keypoints_list.push_back(model_keypoints);
    }

    //Compute Descriptor for Keypoints
    pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch(descr_rad_cg);
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, pcl::ReferenceFrame> rf_est;

    for (int i = 0; i < model_list.size(); i++) {
      pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>());
      descr_est.setInputCloud(model_keypoints_list[i]);
      descr_est.setInputNormals(model_normals_list[i]);
      descr_est.setSearchSurface(model_list[i]);
      descr_est.compute(*model_descriptors);
      model_descriptor_list.push_back(model_descriptors);

      pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
      rf_est.setFindHoles(true);
      rf_est.setRadiusSearch(descr_rad_cg);
      rf_est.setInputCloud(model_keypoints_list[i]);
      rf_est.setInputNormals(model_normals_list[i]);
      rf_est.setSearchSurface(model_list[i]);
      rf_est.compute(*model_rf);
      model_rf_list.push_back(model_rf);
    }

    // Creating and training Hough3D Grouping objects

    for(std::size_t i = 0; i < model_list.size() ; i++)
    {
      ModelRecognizerPtr model_recg(new ModelRecognizer());
      model_recg->setHoughBinSize(cg_size);
      model_recg->setHoughThreshold(cg_thresh);
      model_recg->setUseInterpolation(true);
      model_recg->setUseDistanceWeight(false);
      model_recg->setInputCloud(model_keypoints_list[i]);
      model_recg->setInputRf(model_rf_list[i]);

      if(model_recg->train())
      {
        ROS_INFO_STREAM(boost::str(boost::format("Hough3D algorithm successfully trained for model %1%") % model_names_[i]));
      }
      else
      {
        ROS_ERROR_STREAM(boost::str(boost::format("Hough3D algorithm failed training for model %1%") % model_names_[i]));
      }

      model_recognizers_.push_back(model_recg);

    }
  }

  void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    std::clock_t start, t_start;
    double duration;
    start = std::clock();
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);

    // Load scene
    pcl::fromROSMsg(*cloud_msg, *scene);

    //  Compute Scene normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(k_nearest_neighbors);
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    std::vector<Result, Eigen::aligned_allocator<Result> > results_temp;
    Result result;
    int min_index = -1;

    // Recognition
    if (icp) {
      ROS_INFO_STREAM("Using ICP");
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setSearchMethod(tree);
      fpfh_est.setRadiusSearch(descr_rad_icp);
      fpfh_est.setInputCloud(scene);
      fpfh_est.setInputNormals(scene_normals);
      fpfh_est.compute(*scene_features);
      // ICP
      std::vector<pcl::PointIndices> recognized_indices;
      pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr recognized_cloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA)
      // algorithm
      sac_ia_.setMinSampleDistance(min_sample_distance);
      sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance);
      sac_ia_.setMaximumIterations(nr_iterations);
      sac_ia_.setInputTarget(scene);
      sac_ia_.setTargetFeatures(scene_features);
      results_temp.resize(model_list.size());
      float best_score = std::numeric_limits<float>::infinity();
      for (int j = 0; j < model_list.size(); j++)
      {
        sac_ia_.setInputSource(model_list[j]);
        sac_ia_.setSourceFeatures(model_features_list[j]);
        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align(registration_output);
        results_temp[j].item_name = model_names_[j];
        results_temp[j].item_id = j;
        results_temp[j].fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance);
        results_temp[j].final_transformation = sac_ia_.getFinalTransformation();

        ROS_INFO_STREAM_COND(print_detailed_info,"model (" << j << ") " << model_names_[j] <<
                             " fitnessScore " << results_temp[j].fitness_score);
        if (results_temp[j].fitness_score < best_score)
        {
          min_index = j;
          best_score = results_temp[j].fitness_score;
        }
      }
      result = results_temp[min_index];
    }

    else {
      //ROS_INFO_STREAM("Using Correspondence Grouping");
      // Extract Scene Keypoint
      pcl::UniformSampling<PointType> uniform_sampling;
      pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
      uniform_sampling.setInputCloud(scene);
      uniform_sampling.setRadiusSearch(key_point_sampling);
      uniform_sampling.filter(*scene_keypoints);

      //Compute Descriptor
      pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> descr_est;
      descr_est.setRadiusSearch(descr_rad_cg);
      pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
      descr_est.setInputCloud(scene_keypoints);
      descr_est.setInputNormals(scene_normals);
      descr_est.setSearchSurface(scene);
      descr_est.compute(*scene_descriptors);

      //Compute rf
      pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
      pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, pcl::ReferenceFrame> rf_est;
      rf_est.setFindHoles(true);
      rf_est.setRadiusSearch(descr_rad_cg);
      rf_est.setInputCloud(scene_keypoints);
      rf_est.setInputNormals(scene_normals);
      rf_est.setSearchSurface(scene);
      rf_est.compute(*scene_rf);

      //  Find Model-Scene Correspondences with KdTree
      for (int j = 0; j < model_list.size(); j++)
      {
        pcl::KdTreeFLANN<pcl::SHOT352> match_search;
        pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
        match_search.setInputCloud(model_descriptor_list[j]);

        // For each scene keypoint descriptor, find nearest neighbor into the model keypoints
        // descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < scene_descriptors->size(); ++i) {
          std::vector<int> neigh_indices(1);
          std::vector<float> neigh_sqr_dists(1);
          if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
          {
            continue;
          }

          int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
          for(size_t i = 0 ; i < found_neighs ; i++)
          {
            if(neigh_sqr_dists[i] > descr_dis_thrd)
            {
              continue;
            }
            pcl::Correspondence corr(neigh_indices[i], static_cast<int>(i), neigh_sqr_dists[i]);
            model_scene_corrs->push_back(corr);
          }
        }

        if(model_scene_corrs->empty())
        {
          ROS_WARN_STREAM_COND(print_detailed_info,
                               boost::str(boost::format("Model (%1%) %2% contains no scene correspondences") % j % model_names_[j] ));
          continue;
        }

        ROS_INFO_STREAM_COND(print_detailed_info && !model_scene_corrs->empty(),
                             "Model (" <<j<<") "<< model_names_[j] <<" has " << model_scene_corrs->size()<<" correspondences");

        // running pre-trained hough3d recognition
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;

        ModelRecognizerPtr model_recognizer = model_recognizers_[j];
        model_recognizer->setSceneCloud(scene_keypoints);
        model_recognizer->setSceneRf(scene_rf);
        model_recognizer->setModelSceneCorrespondences(model_scene_corrs);
        model_recognizer->recognize(rototranslations, clustered_corrs);

        if (clustered_corrs.size() > 0)
        {
          // computing total correspondences
          int total_corr = std::accumulate(clustered_corrs.begin(),clustered_corrs.end(), 0 ,[](int c,const pcl::Correspondences& v)
           {
             return c + v.size();
           });

          if (total_corr > min_index)
          {
            min_index = total_corr;
            result.item_name = model_names_[j];
            result.item_id = j;
            result.final_transformation = rototranslations[0];
          }

          ROS_INFO_STREAM_COND(print_detailed_info,
                               "Model (" << j <<") "<< model_names_[j] <<" contains " << total_corr <<" recognized correspondences");

        }

      }
    }

    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    ROS_INFO_STREAM("runtime is " << duration << " seconds.");

    if (min_index == -1) {
      ROS_ERROR_STREAM("-----------------------------");
      ROS_ERROR_STREAM("Recognition Failed, elapsed time: "<<duration<<" seconds");
      ROS_ERROR_STREAM("-----------------------------");
    }
    else
    {

      ROS_INFO_STREAM("-----------------------------");
      ROS_INFO_STREAM("Recognition found object: " << result.item_name<<" in "<<duration<<" seconds");
      ROS_INFO_STREAM("-----------------------------");

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

      // Use ICP to fine align model to scene
      t_start = std::clock();
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setMaximumIterations (iterations);
      icp.setInputSource(rotated_model);
      icp.setInputTarget(scene);
      pcl::PointCloud<pcl::PointXYZ> final;
      icp.align(final);

      duration = (std::clock() - t_start) / (double)CLOCKS_PER_SEC;
      ROS_INFO_STREAM_COND(print_detailed_info && icp.hasConverged(),"ICP refinement converged , with score: " <<icp.getFitnessScore()<<" in "<<
                           duration<<" seconds");

      ROS_ERROR_STREAM_COND(!icp.hasConverged(),"ICP refinement failed, using un-converged pose");

      Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
      pcl::transformPointCloud(*rotated_model, *rotated_model, icp_transformation);
      pcl::transformPointCloud(*rotated_pick_point_cloud, *rotated_pick_point_cloud, icp_transformation);

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
      sensor_point.header.frame_id = cloud_msg->header.frame_id;
      listener.transformPoint("world", sensor_point, world_point);

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
      data_tf.header.frame_id = "world";
      pub_tf.publish(data_tf);
    }

  }

private:
  struct Result {
    std::string item_name;
    int item_id;
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ros::NodeHandle nh_;
  ros::Publisher pub_tf;
  ros::Subscriber cloud_subs_;

  // recognition data structures
  std::vector<std::vector<double> > pick_pose;
  std::vector<std::string> model_names_;
  std::vector<pcl::PointCloud<PointType>::Ptr> model_list;
  std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> model_features_list;
  std::vector<pcl::PointCloud<pcl::SHOT352>::Ptr> model_descriptor_list;
  std::vector<pcl::PointCloud<pcl::ReferenceFrame>::Ptr> model_rf_list;
  std::vector<pcl::PointCloud<PointType>::Ptr> model_keypoints_list;
  std::vector<ModelRecognizerPtr> model_recognizers_;
  tf::TransformListener listener;

  // Algorithm params
  float descr_dis_thrd;
  float descr_rad_cg;
  float descr_rad_icp;
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

  // suppressing pcl output
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  // Initialize ROS
  ros::init(argc, argv, "recognition_node");
  ros::NodeHandle nh;
  RecognitionClass recognition(nh);
  if(!recognition.run())
  {
    return -1;
  }

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::PointCloud2>("segmentation_result", 100, &RecognitionClass::cloudCallBack, &recognition);
  ROS_INFO("Recognition Node Ready ...");
  ros::spin();
}
