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
 
#ifndef __QUADTREE_SEGMENTATION_NODE__
#define __QUADTREE_SEGMENTATION_NODE__

#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/transforms.h>

//using namespace std::chrono_literals;

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
 
/* cloudclusterArray */
#include <autoware_msgs/CloudClusterArray.h>


#include <quadtree_segmentation/quadtree_segmentation.hpp>
#include <quadtree_segmentation/quadtree.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/extract_indices.h>

namespace quadtree_segmentation_node
{
  class QuadTreeSegmentationNode
  {
    public:
      QuadTreeSegmentationNode();
      ~QuadTreeSegmentationNode();

      void callbackGetPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr);
      void callbackGetRoadPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr);
      void callbackGetRoadPoints(const sensor_msgs::PointCloud2::ConstPtr &ptr);
      void callbackTimer(const ros::TimerEvent& event);

      void setScan(quadtree_segmentation::QuadTreeSegmentation& QTS);
      void setPointsForPlanePrint(quadtree_segmentation::QuadTreeSegmentation& QTS);
      void setQuadtreeScan(quadtree_segmentation::QuadTreeSegmentation& QTS);
      void makeBoundingBoxes(std::vector<sensor_msgs::PointCloud2>& in_obb);
      void makeDetectedObjects(jsk_recognition_msgs::BoundingBoxArray& in_objects);
      void makeDetectedObjects_velodyne(jsk_recognition_msgs::BoundingBoxArray& in_objects);
      void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
      jsk_recognition_msgs::BoundingBoxArray mergeNearPolygons();
      void checkMemory();
      cv::Point2f getCenterPoint(std::vector<cv::Point2f> polygon);
      void makeCloudClusters(jsk_recognition_msgs::BoundingBoxArray& in_objects);
      void lateralFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, double in_left_lane_threshold = 12.0,
                    double in_right_lane_threshold = 12.0);
      int getParent(std::vector<int> &parent, int x);
      void unionParent(std::vector<int> &parent, int a, int b);
      int findParent(std::vector<int> &parent, int a, int b);

      jsk_recognition_msgs::BoundingBox estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, double longest_local_z);
      double calcClosenessCriterion(const std::vector<double>& C_1, const std::vector<double>& C_2);
      double setRadius(pcl::PointXYZ searchPoint);
      pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

    private:
      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

      //publisher
      ros::Publisher _pub_plane_print;
      ros::Publisher _pub_projected_cloud;
      ros::Publisher _pub_none_projected_cloud;
      ros::Publisher _pub_detected_boxes;
      ros::Publisher _pub_lidar_detector_objects_polygon;
      ros::Publisher _pub_meter_marker;
      ros::Publisher _pub_total_pixel_marker;
      
      ros::Publisher _pub_obb_boxes;
      ros::Publisher _pub_map_obb_boxes;
      ros::Publisher _pub_transformed_obb_boxes;
      ros::Publisher _pub_cloud_clusters;
      ros::Publisher _pub_lidar_detector_objects, _pub_lidar_detector_objects_velodyne;
      ros::Publisher _pub_used_point_cloud;

      //subscriber
      ros::Subscriber _sub_points_raw;
      ros::Subscriber _sub_road_points;
      ros::Subscriber _sub_road_points_raw;
      ros::Subscriber _sub_clicked_point;

      std::string points_raw_topic;
      std::string road_points_topic;
      std::string road_points_raw_topic;
      std::string publish_cloud_clusters_topic_name_;
      std::string publish_objects_topic_name_;

      quadtree_segmentation::QuadTreeSegmentation QTS;
      quadtree::QuadTree QT;
 

      sensor_msgs::PointCloud2 used_point_cloud_;

      pcl::PointCloud<pcl::PointXYZ> temp_scan_1, temp_scan_2;
      pcl::PointCloud<pcl::PointXYZ> road_points_raw_scan_;
      pcl::PointCloud<pcl::PointXYZ> points_raw_scan_, road_points_scan_;
      pcl::PointCloud<pcl::PointXYZ> fake_scan_;

      pcl::PointCloud<pcl::PointXYZI> filtered_scan_, pcl_scan_;
      pcl::PointCloud<pcl::PointXYZI> non_filtered_scan_, non_pcl_scan_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud_;
      pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw_cloud_;

      jsk_recognition_msgs::BoundingBoxArray clusteredBox_;
      visualization_msgs::Marker points_, points_meter_marker;

      std::vector<sensor_msgs::PointCloud2> cloudarray_;
      std::vector<sensor_msgs::PointCloud2> cloudarray_filtered_;
      std::vector<sensor_msgs::PointCloud2> cloudarray_merged_;

      std::vector<double> zarray_;
      std::vector<double> zarray_filtered_;
      std::vector<double> zarray_merged_;

      std::vector<std::vector<cv::Point2f>> hull_vec_;
      bool ok_;
      bool road_based_version_;
      bool full_version_;
      bool istrackdriving_;
      
      double height_from_ground_to_lidar_;
      double x_limit_forward_;
      double x_limit_back_;
      double y_limit_;
      double z_high_limit_;
      double erase_road_threshold_;

      double my_car_width_;
      double my_car_height_;
      double max_cluster_size_;
      
      unsigned int seq;

      tf::TransformListener listener;
      ros::Time lidarTimeStamp_;
  };
}

#endif
