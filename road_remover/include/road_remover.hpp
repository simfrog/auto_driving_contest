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


#ifndef __ROAD_REMOVER__
#define __ROAD_REMOVER__

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <ctime>
#include <pcl/segmentation/segment_differences.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <memory>
#include <pcl/search/organized.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace road_remover
{
    class RoadRemover
    {
        public:
            RoadRemover();
            ~RoadRemover();
            void callbackGetPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr);
            void callbackLocalizePose(const geometry_msgs::PoseStampedConstPtr &ptr);
            void callBackPointsMap(const sensor_msgs::PointCloud2ConstPtr &ptr);

            void getUnfilteredPoint();
            void findTransformMatrix();
            void getLocalPointMap();
            void segmentDiff();
            void initData();
            void getRoad();
        private:
            ros::NodeHandle nh;
            ros::NodeHandle private_nh;

            //publisher
            ros::Publisher _pub_filtered_points_raw;
            ros::Publisher _pub_unfiltered_points_raw;
            ros::Publisher _pub_local_points_map_raw;
            ros::Publisher _pub_road_points_raw;

            //subscriber
            ros::Subscriber _sub_points_raw;
            ros::Subscriber _sub_road_points;
            ros::Subscriber _sub_point_map;

            std::string points_raw_topic_name_;
            std::string pose_topic_name_;
            std::string point_map_topic_name_;
            std::string publish_no_road_point_topic_name_;
            std::string publish_road_point_topic_name_;
            
            pcl::PointCloud<pcl::PointXYZ> points_raw_data;
            pcl::PointCloud<pcl::PointXYZ> filtered_points_raw;
            pcl::PointCloud<pcl::PointXYZ> unfiltered_points_raw;
            pcl::PointCloud<pcl::PointXYZ> road_data;
            pcl::PointCloud<pcl::PointXYZ> point_map_data;
            pcl::PointCloud<pcl::PointXYZ> last_filtered;
            pcl::PointCloud<pcl::PointXYZ> road_points_raw;

            pcl::KdTreeFLANN<pcl::PointXYZ> pmap_kdtree;

            pcl::PointCloud<pcl::PointXYZ> local_point_map;

            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> point_map_octree;
            pcl::search::Search<pcl::PointXYZ>::Ptr organ_search;

            geometry_msgs::PoseStamped pose_;

            pcl::PointXYZ searchPoint;

            int seq;
            bool getPointsRaw;
            bool getPointMap;
            bool getCurrentPose;
            double z_dist;
            ros::Time avg_time;
            int count;

            ros::Time lidarTimeStamp_;
    };
}

#endif
