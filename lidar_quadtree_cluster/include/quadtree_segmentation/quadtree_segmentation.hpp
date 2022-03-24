/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * Lee Jae min
 */
 
#ifndef __QUADTREE_SEGMENTATION__
#define __QUADTREE_SEGMENTATION__

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <visualization_msgs/Marker.h>
#include <queue>

/* Quadtree header */
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>
#include<string>
#include<cmath>
#include<Eigen/Dense>
#include<pcl_conversions/pcl_conversions.h>
#include<ctime>
#include<cstdlib>

/* obb_generator header */
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define ROAD_EXTRACTOR false
#define LIDAR_FILTER true

namespace quadtree_segmentation
{
  class QuadTreeSegmentation
  {
    public:
      QuadTreeSegmentation();
      ~QuadTreeSegmentation();
      
      void setInputCloud(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
      void setInputRoad(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
      void setInputRoadPointsRaw(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
      void setTimeStamp(const ros::Time& stamp);
      void setParam(double height_from_ground_to_lidar, double x_limit_forward, double x_limit_back, double y_limit, double z_high_limit, double erase_road_threshold);
      void initScan();

      pcl::PointCloud<pcl::PointXYZ> getPointsRawScan();
      pcl::PointCloud<pcl::PointXYZ> getRoadPointsScan();
      pcl::PointCloud<pcl::PointXYZI> getFilteredScan();
      pcl::PointCloud<pcl::PointXYZI> getPclScan();
      pcl::PointCloud<pcl::PointXYZI> getNoneFilteredScan();
      pcl::PointCloud<pcl::PointXYZI> getNonePclScan();
      pcl::PointCloud<pcl::PointXYZ>::Ptr getRansacCloud();
      pcl::PointCloud<pcl::PointXYZ>::Ptr getPointsRawCloud();
      visualization_msgs::Marker getPoints();
      size_t getNormalVectorQueueSize();
      size_t getPlanePointVectSize();
      size_t getDQueueSize();

      void makePlaneRANSAC(bool version);
      void projectionOntoPlane(bool version, pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud_,pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw_cloud_, pcl::PointCloud<pcl::PointXYZ>& road_points_scan_);
      void extractNormalVector();
      void printRansacPlane();
      void reset();
      bool isNearPoint(pcl::PointXYZ searchPoint, pcl::PointXYZ point);

    private:
      pcl::PointCloud<pcl::PointXYZ> points_raw_scan_, road_points_scan_, road_points_raw_scan_;
      pcl::PointCloud<pcl::PointXYZI> filtered_scan_, pcl_scan_;
      pcl::PointCloud<pcl::PointXYZI> non_filtered_scan_, non_pcl_scan_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw_cloud_;

      geometry_msgs::Point normal_vector_; 
      visualization_msgs::Marker points_;
      
      std::queue< geometry_msgs::Point > normal_vector_queue_; 
      std::vector< geometry_msgs::Point > plane_point_vec_;           
      std::queue< float > D_queue_;

      float D_; 
      bool ok_;
      bool road_based_version_;
      bool full_version_;
      
      double height_from_ground_to_lidar_;
      double x_limit_forward_;
      double x_limit_back_;
      double y_limit_;
      double z_high_limit_;
      double erase_road_threshold_;

      ros::Time lidarTimeStamp_;
  };
}

#endif
