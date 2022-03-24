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
 
#ifndef __QUADTREE__
#define __QUADTREE__

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

/* octree header */
#include <pcl/octree/octree_search.h>
#include <unordered_map>


/* Quadtree macro variable */
#define LEVELX_BOX_PIXEL 256
#define LEVEL0_BOX_PIXEL 128
#define LEVEL1_BOX_PIXEL 64
#define LEVEL2_BOX_PIXEL 32
#define LEVEL3_BOX_PIXEL 16
#define LEVEL4_BOX_PIXEL 8
#define LEVEL5_BOX_PIXEL 4
#define LEVEL6_BOX_PIXEL 2
#define LEVEL7_BOX_PIXEL 1

#define LEVELX_THRESHOLD 65536      // 65536 
#define LEVEL0_THRESHOLD 16384      // 16384 
#define LEVEL1_THRESHOLD 4096       // 4096
#define LEVEL2_THRESHOLD 1024       // 1024 or 700
#define LEVEL3_THRESHOLD 256       // 256 or 180
#define LEVEL4_THRESHOLD 64        // 64 or 40
#define LEVEL5_THRESHOLD 16         // 16 or 13
#define LEVEL6_THRESHOLD 4          // 4 or 2
#define LEVEL7_THRESHOLD 1          // 1

/* DEBUG PARAMS */
#define POINT_DEBUG 0

/* DETECTION AREA SIZE */
/* 1 must be exist only one*/
#define DETECT_LEVEL_25M 0
#define DETECT_LEVEL_50M 1

namespace quadtree
{
  struct XYI
  {
    int count;
    double x, y, z;
    double maxZ;
    bool isIn_;
    std::vector<pcl::PointXYZI> points_vec;
  };

  class Box
  {
    public:
      Box(double xpose, double ypose, double bsize, int pixel);
      Box();
      ~Box();

      Box getUR();
      Box getUL();
      Box getDR();
      Box getDL();
      Box getR();
      Box getL();
      Box getU();
      Box getD();
      
      int getPixelSize();
      double getXPose();
      double getYPose();
      double getSize();

    private:
      double size_;
      double Xpose_, Ypose_;
      int pixelSize_;
  };

  class QuadTree
  {
    public:
      QuadTree();
      QuadTree(Box& re);
      ~QuadTree();

      jsk_recognition_msgs::BoundingBoxArray quadTreeMain(sensor_msgs::PointCloud2* in_points, sensor_msgs::PointCloud2* pcl_points);
      void initPixel();
      void initTotalBox();
      void setParam();
      void setTimeStamp(const ros::Time& stamp);
      int divideQuadTree();
      void divideQuadTree2();
      void makeGroup();
      void makeGroup_Octree();
      int getParent(std::vector<int> &parent, int x);
      void unionParent(std::vector<int> &parent, int a, int b);
      int findParent(std::vector<int> &parent, int a, int b);

      void grouping(int index, int box_label, pcl::PointCloud<pcl::PointXYZI>& in_points, double& longest_local_z);
      bool checkChild(std::vector< std::vector<XYI> >& in_pixel, Box& in_box);
      bool checkThreshold(Box& in_box, int in_Cnt);
      void eraseOverlappedElement();
      bool checkDist(jsk_recognition_msgs::BoundingBox& in_box1, jsk_recognition_msgs::BoundingBox& in_box2);
      void reset();
      std::vector<sensor_msgs::PointCloud2> getCloudArray();
      std::vector<double> getZArray();
      size_t getClusteredBoxSize();
      size_t getCloudarraySize();
      double setRadius(pcl::PointXYZ searchPoint);
      void printTotalPixel();
      visualization_msgs::Marker getPoints();

    private:
      sensor_msgs::PointCloud2 points_msg_;
      
      jsk_recognition_msgs::BoundingBoxArray clusteredBox_;
      visualization_msgs::Marker points_;

      std::vector<sensor_msgs::PointCloud2> cloudarray_;
      std::vector<double> zarray_, zarray_temp_;

      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;

      double default_x_;
      double default_y_;
      double default_size_;
      int default_pixel_;
      double boxSize_;
      int start_Xindex_;
      int start_Yindex_;
      double box_z_;
      int point_pixel_x_;
      int point_pixel_y_;
      int pixel_Xmax_;
      int pixel_Ymax_;
      double box_threshold_;
      double height_from_ground_to_lidar_;

      double boxPose_x_, boxPose_y_;
      bool haschildren_, haspoints_;
      unsigned int seq;

      Box rect_;
      QuadTree* children_[4];

      ros::Time lidarTimeStamp_;
  };

  

}

#endif
