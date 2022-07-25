//
// Copyright (c) 2022, Takahiro Miki. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

// STL
#include <set>
#include <mutex>
#include <iostream>

// Eigen
#include <Eigen/Dense>

// Pybind
#include <pybind11_catkin/pybind11/embed.h>  // everything needed for embedding

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_geometry/pinhole_camera_model.h> 

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <elevation_map_msgs/CheckSafety.h>
#include <elevation_map_msgs/Initialize.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCv
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "elevation_mapping_cupy/elevation_mapping_wrapper.hpp"

// Chrono
#include <chrono>

namespace py = pybind11;
using namespace std::chrono;

namespace elevation_mapping_cupy {

class ElevationMappingNode {
 public:
  ElevationMappingNode(ros::NodeHandle& nh);

 private:
  void readParameters();
  void initializeWithTF();
  void setupMapPublishers();
  void publishMapOfIndex(int index);
  void publishMapToOdom(double error);
  void updateTime(const ros::TimerEvent&);
  void updatePose(const ros::TimerEvent&);
  void updateGridMap(const ros::TimerEvent&);
  void updateVariance(const ros::TimerEvent&);
  void publishStatistics(const ros::TimerEvent&);
  void publishAsPointCloud(const grid_map::GridMap& map) const;
  void publishNormalAsArrow(const grid_map::GridMap& map) const;
  void setDepthCameraInfo(const sensor_msgs::CameraInfo& camInfo);
  bool clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void filterLegsFromPointcloud(pcl::PCLPointCloud2::Ptr cloud, const ros::Time& timeStamp); 
  bool setPublishPoint(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  void pointcloudCallback(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& pointcloudEvent);
  bool clearMapWithInitializer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
  bool checkSafety(elevation_map_msgs::CheckSafety::Request& request, elevation_map_msgs::CheckSafety::Response& response);
  bool initializeMap(elevation_map_msgs::Initialize::Request& request, elevation_map_msgs::Initialize::Response& response);

  visualization_msgs::Marker vectorToArrowMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const int id) const;
  ros::NodeHandle nh_;
  ros::Subscriber cameraInfoSub_;
  std::vector<ros::Subscriber> pointcloudSubs_;
  std::vector<ros::Publisher> mapPubs_;
  ros::Publisher filteredPointCloud;
  tf::TransformBroadcaster tfBroadcaster_;
  ros::Publisher alivePub_;
  ros::Publisher pointPub_;
  ros::Publisher normalPub_;
  ros::Publisher statisticsPub_;
  ros::ServiceServer rawSubmapService_;
  ros::ServiceServer clearMapService_;
  ros::ServiceServer clearMapWithInitializerService_;
  ros::ServiceServer initializeMapService_;
  ros::ServiceServer setPublishPointService_;
  ros::ServiceServer checkSafetyService_;
  ros::Timer updateVarianceTimer_;
  ros::Timer updateTimeTimer_;
  ros::Timer updatePoseTimer_;
  ros::Timer updateGridMapTimer_;
  ros::Timer publishStatisticsTimer_;
  ros::Time lastStatisticsPublishedTime_;
  tf::TransformListener transformListener_;
  ElevationMappingWrapper map_;
  std::string mapFrameId_;
  std::string correctedMapFrameId_;
  std::string baseFrameId_;
  std::string depthCameraInfoTopic_;
  sensor_msgs::CameraInfo depthCameraInfo_;
  std::vector<std::string> legPointcloudTopics_;

  // map topics info
  std::vector<double> map_fps_;
  std::set<double> map_fps_unique_;
  std::vector<ros::Timer> mapTimers_;
  std::set<std::string> map_layers_all_;
  std::set<std::string> map_layers_sync_;
  std::vector<std::vector<std::string>> map_topics_;
  std::vector<std::vector<std::string>> map_layers_;
  std::vector<std::vector<std::string>> map_basic_layers_;

  std::string initializeMethod_;
  std::vector<double> initialize_tf_offset_;
  std::vector<std::string> initialize_frame_id_;

  Eigen::Vector3d lowpassPosition_;
  Eigen::Vector4d lowpassOrientation_;

  std::mutex mapMutex_;  // protects gridMap_
  grid_map::GridMap gridMap_;
  std::atomic_bool isGridmapUpdated_;  // needs to be atomic (read is not protected by mapMutex_)

  double positionError_;
  std::mutex errorMutex_; // protects positionError_, and orientationError_
  double orientationError_;

  double positionAlpha_;
  double orientationAlpha_;

  int meanK_;
  double stdDevK_;
  int sampleSize_;
  double voxelSize_;

  int legsSafetyRegion_;
  bool setDepthCameraInfo_;

  bool printRoutinesRuntimes_;

  double recordableFps_;
  bool enableVoxelFiltering_;
  bool useInitializerAtStart_;
  double initializeTfGridSize_;
  bool enablePointCloudSampling_;
  bool enableStatisticalFiltering_;
  bool enableNormalArrowPublishing_;
  bool enableDriftCorrectedTFPublishing_;
  std::atomic_bool enablePointCloudPublishing_;
  std::atomic_int pointCloudProcessCounter_;

  // image_geometry::PinholeCameraModel pcm_;
};

}  // namespace elevation_mapping_cupy