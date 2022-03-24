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


#include "road_remover.hpp"

namespace road_remover
{
    RoadRemover::RoadRemover() : private_nh("~"), point_map_octree(128.0), seq(0), getPointsRaw(false), getPointMap(false), getCurrentPose(false), count(0)
    {
      if(!private_nh.getParam("points_raw_topic_name", points_raw_topic_name_))                          throw std::runtime_error("fail to get points_raw topic name");
      if(!private_nh.getParam("pose_topic_name", pose_topic_name_))                                      throw std::runtime_error("fail to get pose topic name");
      if(!private_nh.getParam("point_map_topic_name", point_map_topic_name_))                            throw std::runtime_error("fail to get point_map topic name");
      if(!private_nh.getParam("publish_no_road_point_topic_name", publish_no_road_point_topic_name_))    throw std::runtime_error("fail to get publish_no_road_point topic name");
      if(!private_nh.getParam("publish_road_point_topic_name", publish_road_point_topic_name_))          throw std::runtime_error("fail to get publish_road_point topic name");
    

      //publisher
      _pub_filtered_points_raw = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_raw", 1);
      _pub_unfiltered_points_raw = nh.advertise<sensor_msgs::PointCloud2>("/unfiltered_points_raw", 1);
      _pub_local_points_map_raw = nh.advertise<sensor_msgs::PointCloud2>("/local_points_map_raw", 1);
      _pub_road_points_raw = nh.advertise<sensor_msgs::PointCloud2>("/road_points_raw", 1);

      //subscriber
      _sub_points_raw = nh.subscribe(points_raw_topic_name_, 1, &RoadRemover::callbackGetPointsRaw, this);
      _sub_road_points = nh.subscribe(pose_topic_name_, 1, &RoadRemover::callbackLocalizePose, this);
      _sub_point_map = nh.subscribe(point_map_topic_name_, 1, &RoadRemover::callBackPointsMap, this);
      
      organ_search.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    RoadRemover::~RoadRemover()
    {

    }

    void RoadRemover::callBackPointsMap(const sensor_msgs::PointCloud2ConstPtr &ptr) // map frame
    {
      std::cout << "GET Points map!" << std::endl;
      pcl::fromROSMsg(*ptr, point_map_data);
      std::cout << "Point map size : " << point_map_data.size() << std::endl;

      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(point_map_data.makeShared());//scan PointCloud data copy
      vg.setMinimumPointsNumberPerVoxel(2); // erase noise
      vg.setLeafSize(0.4f,0.4f,4.0f);//set the voxel grid size
      vg.filter(point_map_data);//create the filtering object
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr octree_input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(point_map_data));
      // point_map_octree.setInputCloud(octree_input_cloud_ptr);
      // point_map_octree.addPointsFromInputCloud();
      //pmap_kdtree.setInputCloud(point_map_data.makeShared());

      organ_search->setInputCloud(octree_input_cloud_ptr);

      getPointMap = true;
    }

    void RoadRemover::callbackGetPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr)
    {
      if(getCurrentPose != true || getPointMap != true) return;
      lidarTimeStamp_ = ptr->header.stamp;
      initData();

      pcl::fromROSMsg(*ptr, points_raw_data);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(points_raw_data.makeShared());
      vg.setLeafSize(0.2f,0.2f,0.2f); // for voxeling points_raw
      vg.filter(points_raw_data); // for voxeling points_raw

      for(size_t i = 0; i < points_raw_data.points.size(); ++i)
      {
        if(points_raw_data.points[i].z < 0.5 && fabs(points_raw_data.points[i].x) <= 70 && fabs(points_raw_data.points[i].y) <= 70)
          filtered_points_raw.push_back(points_raw_data.points[i]);
      }
    
      getLocalPointMap();
      segmentDiff();
      getUnfilteredPoint();
      //getRoad();


      sensor_msgs::PointCloud2 filtered_points_cloud;
      sensor_msgs::PointCloud2 unfiltered_points_cloud;
      sensor_msgs::PointCloud2 local_points_map_cloud;
      sensor_msgs::PointCloud2 road_points_cloud;

      pcl::PointCloud<pcl::PointXYZ>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZ>(last_filtered));
      pcl::PointCloud<pcl::PointXYZ>::Ptr n_ptr_2(new pcl::PointCloud<pcl::PointXYZ>(unfiltered_points_raw));
      pcl::PointCloud<pcl::PointXYZ>::Ptr n_ptr_3(new pcl::PointCloud<pcl::PointXYZ>(local_point_map));
      pcl::PointCloud<pcl::PointXYZ>::Ptr n_ptr_4(new pcl::PointCloud<pcl::PointXYZ>(road_points_raw));

      pcl::toROSMsg(*n_ptr, filtered_points_cloud);
      pcl::toROSMsg(*n_ptr_2, unfiltered_points_cloud);
      pcl::toROSMsg(*n_ptr_3, local_points_map_cloud);
      pcl::toROSMsg(*n_ptr_4, road_points_cloud);

      filtered_points_cloud.header.frame_id = "velodyne";
      filtered_points_cloud.header.stamp = lidarTimeStamp_;
      filtered_points_cloud.header.seq = seq;
      unfiltered_points_cloud.header.frame_id = "velodyne";
      unfiltered_points_cloud.header.stamp = lidarTimeStamp_;
      unfiltered_points_cloud.header.seq = seq;
      local_points_map_cloud.header.frame_id = "velodyne";
      local_points_map_cloud.header.stamp = lidarTimeStamp_;
      local_points_map_cloud.header.seq = seq;
      road_points_cloud.header.frame_id = "velodyne";
      road_points_cloud.header.stamp = lidarTimeStamp_;
      road_points_cloud.header.seq = seq++;

      _pub_filtered_points_raw.publish(filtered_points_cloud);
      _pub_unfiltered_points_raw.publish(unfiltered_points_cloud);
      _pub_local_points_map_raw.publish(local_points_map_cloud);
      _pub_road_points_raw.publish(road_points_cloud);

    }

    void RoadRemover::initData()
    {
      local_point_map.resize(0);
      filtered_points_raw.resize(0);
    }

    void RoadRemover::callbackLocalizePose(const geometry_msgs::PoseStampedConstPtr &ptr)
    {
      pose_.pose.position.x = ptr->pose.position.x;
      pose_.pose.position.y = ptr->pose.position.y;
      pose_.pose.position.z = ptr->pose.position.z;
      pose_.pose.orientation = ptr->pose.orientation;

      getCurrentPose = true;
    }

    void RoadRemover::findTransformMatrix()
    {
      z_dist = 0.0;
    }

    void RoadRemover::getLocalPointMap()
    {
      float x_ = pose_.pose.position.x;
      float y_ = pose_.pose.position.y;
      float z_ = pose_.pose.position.z;

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      pcl::PointXYZ searchPoint;
      searchPoint.x = pose_.pose.position.x;
      searchPoint.y = pose_.pose.position.y;
      searchPoint.z = pose_.pose.position.z;

      if (organ_search->radiusSearch(searchPoint, 70, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
          if(point_map_data.points[pointIdxRadiusSearch[i]].z > searchPoint.z + 1.8) continue;

          local_point_map.points.push_back(point_map_data.points[pointIdxRadiusSearch[i]]);
        }
      }

      // transform matrix
      tf2::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      Eigen::Matrix3f rotation_mat_roll, rotation_mat_pitch, rotation_mat_yaw, rotation_mat;

      // roll
      rotation_mat_roll << 1, 0, 0,
                0, cos(roll), -sin(roll),
                0, sin(roll), cos(roll);
      // pitch
      rotation_mat_pitch << cos(pitch), 0, sin(pitch),
                0, 1, 0,
                -sin(pitch), 0, cos(pitch);
      // yaw
      rotation_mat_yaw << cos(yaw), -sin(yaw), 0,
                sin(yaw),  cos(yaw) , 0,
                0, 0, 1;

      // ratation_matrix
      rotation_mat = rotation_mat_yaw * rotation_mat_pitch * rotation_mat_roll;

      for(size_t i = 0; i < local_point_map.points.size(); ++i)
      {
        Eigen::Vector3f p(local_point_map.points[i].x - x_, local_point_map.points[i].y - y_, local_point_map.points[i].z - z_);
        Eigen::Vector3f result;

        result = rotation_mat.inverse() * p;  

        local_point_map.points[i].x = result[0];
        local_point_map.points[i].y = result[1];
        local_point_map.points[i].z = result[2];
      }

      findTransformMatrix();

      for(size_t i = 0; i < local_point_map.points.size(); ++i)
      {
        local_point_map.points[i].z += z_dist;
      }
    }

    void RoadRemover::segmentDiff()
    {
      // std::cout << local_point_map.points.size() << " , " << filtered_points_raw.points.size() << std::endl;
      
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      // pcl::search::Octree<pcl::PointXYZ>::Ptr tree (new pcl::search::Octree<pcl::PointXYZ>(0.1));
      pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
      sdiff.setInputCloud(filtered_points_raw.makeShared());
      sdiff.setTargetCloud(local_point_map.makeShared());
      sdiff.setSearchMethod(tree);
      sdiff.setDistanceThreshold(0.1);
      sdiff.segment(last_filtered);
      // std::cout << last_filtered.points.size() << std::endl;
    }

    void RoadRemover::getUnfilteredPoint()
    {
      unfiltered_points_raw.points.resize(0);

      for(size_t i = 0; i < local_point_map.points.size(); ++i)
      {
        if(fabs(local_point_map.points[i].x) < 20 && fabs(local_point_map.points[i].y) < 20)
          unfiltered_points_raw.points.push_back(local_point_map.points[i]);
      }
    }

    void RoadRemover::getRoad()
    {
      
      road_points_raw.points.resize(0);
      

      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (local_point_map.makeShared());            
      sor.setMeanK (10);                     
      sor.setStddevMulThresh (0.3);         
      sor.setNegative(true);
      sor.filter (road_points_raw);
      // std::cout << road_points_raw.points.size() << std::endl;
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "road_remover");

  road_remover::RoadRemover RR;

  ros::spin();

  return 0;
}
