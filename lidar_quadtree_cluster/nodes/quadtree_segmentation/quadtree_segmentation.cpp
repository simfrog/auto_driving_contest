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
 
#include <quadtree_segmentation/quadtree_segmentation.hpp>

namespace quadtree_segmentation
{
  QuadTreeSegmentation::QuadTreeSegmentation() 
    : ransac_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , points_raw_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , road_based_version_(true)
    , full_version_(false)
  {
    
  }

  QuadTreeSegmentation::~QuadTreeSegmentation()
  {

  }

  void QuadTreeSegmentation::setInputCloud(pcl::PointCloud<pcl::PointXYZ>& point_cloud)
  {
    points_raw_scan_ = point_cloud;
  }

  void QuadTreeSegmentation::setInputRoad(pcl::PointCloud<pcl::PointXYZ>& point_cloud)
  {
    road_points_scan_ = point_cloud;
  }

  void QuadTreeSegmentation::setInputRoadPointsRaw(pcl::PointCloud<pcl::PointXYZ>& point_cloud)
  {
    road_points_raw_scan_ = point_cloud;
  }

  void QuadTreeSegmentation::setTimeStamp(const ros::Time& stamp)
  {
    lidarTimeStamp_ = stamp;
  }
  void QuadTreeSegmentation::initScan()
  {
    filtered_scan_.resize(0);
    non_filtered_scan_.resize(0);
    pcl_scan_.resize(0);
    non_pcl_scan_.resize(0);

    filtered_scan_.header.frame_id = "velodyne";
    pcl_scan_.header.frame_id = "velodyne";
    non_filtered_scan_.header.frame_id = "velodyne";
    non_pcl_scan_.header.frame_id = "velodyne";
  }

  void QuadTreeSegmentation::setParam(double height_from_ground_to_lidar, double x_limit_forward, double x_limit_back, double y_limit, double z_high_limit, double erase_road_threshold)
  {
    height_from_ground_to_lidar_ = height_from_ground_to_lidar;
    x_limit_forward_ = x_limit_forward;
    x_limit_back_ = x_limit_back;
    y_limit_ = y_limit;
    z_high_limit_ = z_high_limit;
    erase_road_threshold_ = erase_road_threshold;
  }

  pcl::PointCloud<pcl::PointXYZ> QuadTreeSegmentation::getPointsRawScan()
  {
    return points_raw_scan_;
  }

  pcl::PointCloud<pcl::PointXYZ> QuadTreeSegmentation::getRoadPointsScan()
  {
    return road_points_scan_;
  }

  pcl::PointCloud<pcl::PointXYZI> QuadTreeSegmentation::getFilteredScan()
  {
    return filtered_scan_;
  }

  pcl::PointCloud<pcl::PointXYZI> QuadTreeSegmentation::getPclScan()
  {
    return pcl_scan_;
  }

  pcl::PointCloud<pcl::PointXYZI> QuadTreeSegmentation::getNoneFilteredScan()
  {
    return non_filtered_scan_;
  }

  pcl::PointCloud<pcl::PointXYZI> QuadTreeSegmentation::getNonePclScan()
  {
    return non_pcl_scan_;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr QuadTreeSegmentation::getRansacCloud()
  {
    return ransac_cloud_;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr QuadTreeSegmentation::getPointsRawCloud()
  {
    return points_raw_cloud_;
  }

  visualization_msgs::Marker QuadTreeSegmentation::getPoints()
  {
    return points_;
  }

  size_t QuadTreeSegmentation::getNormalVectorQueueSize()
  {
    return normal_vector_queue_.size();
  }

  size_t QuadTreeSegmentation::getPlanePointVectSize()
  {
    return plane_point_vec_.size();
  }

  size_t QuadTreeSegmentation::getDQueueSize()
  {
    return D_queue_.size();
  }

  void QuadTreeSegmentation::makePlaneRANSAC(bool version)
  {
    if(version == ROAD_EXTRACTOR)
    {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.3);

      pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;

      for (size_t k = 0; k < ransac_cloud_->points.size(); ++k)
      {
        if (fabs(ransac_cloud_->points[k].x) < 50 && fabs(ransac_cloud_->points[k].y) < 50 && ransac_cloud_->points[k].z < (-1) * height_from_ground_to_lidar_ + 0.2 && ransac_cloud_->points[k].z > (-1) * height_from_ground_to_lidar_ - 0.2)
        {
          pcl::PointXYZ z_filtered_point;
          z_filtered_point.x = ransac_cloud_->points[k].x;
          z_filtered_point.y = ransac_cloud_->points[k].y;
          z_filtered_point.z = ransac_cloud_->points[k].z;
          filtered_points_cloud_z.push_back(z_filtered_point);
        }
      }
      ransac_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
      seg.setInputCloud(point_ptr);
      point_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        normal_vector_.x = 0;
        normal_vector_.y = 0;
        normal_vector_.z = 1;
        D_ = -2.0;

        normal_vector_queue_.push(normal_vector_);
        D_queue_.push(D_);
        extractNormalVector();
      }
      else
      {
        normal_vector_.x = coefficients->values[0];
        normal_vector_.y = coefficients->values[1];
        normal_vector_.z = coefficients->values[2];

        D_ = (-1) * coefficients->values[3];
        normal_vector_queue_.push(normal_vector_);
        D_queue_.push(D_);
        extractNormalVector();

        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
      }
      
    }
    
    if(version == LIDAR_FILTER)
    {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.3);

      pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;
      
      size_t count=0;
      // std::cout << ransac_cloud_->points.size() << std::endl;
      for (size_t k = 0; k < ransac_cloud_->points.size(); ++k)
      {
        if (fabs(ransac_cloud_->points[k].x) < 50 && fabs(ransac_cloud_->points[k].y) < 50 && ransac_cloud_->points[k].z < (-1) * height_from_ground_to_lidar_ + 0.2 && ransac_cloud_->points[k].z > (-1) * height_from_ground_to_lidar_ - 0.2)
        {
          count++;
          pcl::PointXYZ z_filtered_point;
          z_filtered_point.x = ransac_cloud_->points[k].x;
          z_filtered_point.y = ransac_cloud_->points[k].y;
          z_filtered_point.z = ransac_cloud_->points[k].z;
          filtered_points_cloud_z.push_back(z_filtered_point);
        }
      }
      // std::cout << count << std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
      seg.setInputCloud(point_ptr);
      point_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        normal_vector_.x = 0;
        normal_vector_.y = 0;
        normal_vector_.z = 1;
        D_ = -2.0;

        normal_vector_queue_.push(normal_vector_);
        D_queue_.push(D_);
        extractNormalVector();
      }
      else
      {
        normal_vector_.x = coefficients->values[0];
        normal_vector_.y = coefficients->values[1];
        normal_vector_.z = coefficients->values[2];

        D_ = (-1) * coefficients->values[3];
        normal_vector_queue_.push(normal_vector_);
        D_queue_.push(D_);
        extractNormalVector();

        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
      }
    }
  }

  void QuadTreeSegmentation::projectionOntoPlane(bool version, pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw_cloud, pcl::PointCloud<pcl::PointXYZ>& road_points_scan)
  {
    Eigen::Vector4f coeffs;
    coeffs << normal_vector_.x, normal_vector_.y, normal_vector_.z, -D_;

    if(version == road_based_version_)
    {
      // new calculation 2
      // first filtering
      // clock_t start, end;
      // start = clock();
      pcl::PointCloud<pcl::PointXYZ> first_filtered;
      pcl::PointCloud<pcl::PointXYZ> non_filtered;
      pcl::PointCloud<pcl::PointXYZ> last_filtered;
      
      std::vector<cv::Point2f> points;
      for(size_t i = 0; i < road_points_scan.points.size(); ++i)
      {
        cv::Point2f pt;
        pt.x = road_points_scan.points[i].x;
        pt.y = road_points_scan.points[i].y;
        points.push_back(pt);
      }
      std::vector<cv::Point2f> hull;
      cv::convexHull(points, hull);

      for(size_t i = 0; i < points_raw_cloud->points.size(); ++i)
      {
        cv::Point2f pt;
        pt.x = points_raw_cloud->points[i].x;
        pt.y = points_raw_cloud->points[i].y;
        int location = cv::pointPolygonTest(hull, pt, false);

        if(location >= 0) // 1 : inside , 0 : on the hull , -1 : outside
          first_filtered.points.push_back(points_raw_cloud->points[i]);
        else
          non_filtered.points.push_back(points_raw_cloud->points[i]);
      }
      // end = clock();
      // std::cout << "check 1 : " << end - start << std::endl;

      // second filtering
      pcl::PointCloud<pcl::PointXYZ>::Ptr octree_input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(road_points_scan));
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
      octree.setInputCloud(octree_input_cloud_ptr);
      octree.addPointsFromInputCloud();
      // start = clock();
      for(size_t i = 0; i < first_filtered.points.size(); ++i)
      {
        if(first_filtered.points[i].z < z_high_limit_ && first_filtered.points[i].z > (-1) * height_from_ground_to_lidar_)
        {
          pcl::PointXYZ searchPoint = first_filtered.points[i];
        
          int K = 1;
          std::vector<int> pointIdxNKNSearch; //Save the index result of the K nearest neighbor
          std::vector<float> pointNKNSquaredDistance;  //Save the index result of the K nearest neighbor

          if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
          {   
            for(size_t k = 0; k < pointIdxNKNSearch.size(); ++k)
            {
              double gap_z = searchPoint.z - road_points_scan.points[pointIdxNKNSearch[k]].z; 

              if(gap_z < 0.0) // road point > search point
                non_filtered.points.push_back(searchPoint);
              else
                last_filtered.points.push_back(searchPoint);
            }
          }
          else
            last_filtered.points.push_back(searchPoint);
        }
      }

      // end = clock();
      // std::cout << "check 2 : " << end - start << std::endl;
      // start = clock();
      // generate cloud
      for(size_t i = 0; i < last_filtered.points.size(); ++i)
      {
        pcl::PointXYZI projection, point;

        double distance = pcl::pointToPlaneDistance(last_filtered.points[i], normal_vector_.x, normal_vector_.y, normal_vector_.z, (-1) * D_);
        if(distance < 0.3) continue;

        projection.x = last_filtered.points[i].x;  
        projection.y = last_filtered.points[i].y;
        projection.z = (-1) * (normal_vector_.x * last_filtered.points[i].x + normal_vector_.y * last_filtered.points[i].y - D_) / normal_vector_.z;
        projection.intensity = 2.0;
        pcl_scan_.points.emplace_back(projection);

        point.x = last_filtered.points[i].x;  
        point.y = last_filtered.points[i].y;
        point.z = last_filtered.points[i].z;
        filtered_scan_.points.emplace_back(point);
      }

      for(size_t i = 0; i < non_filtered.points.size(); ++i)
      {
        pcl::PointXYZI non_projection, non_point;

        non_projection.x = non_filtered.points[i].x;  
        non_projection.y = non_filtered.points[i].y;
        non_projection.z = non_filtered.points[i].z;
        non_projection.intensity = 2.0;
        non_pcl_scan_.points.emplace_back(non_projection);

        non_point.x = non_filtered.points[i].x;  
        non_point.y = non_filtered.points[i].y;
        non_point.z = non_filtered.points[i].z;
        non_filtered_scan_.points.emplace_back(non_point);
      }
      // end = clock();
      // std::cout << "check 3 : " << end - start << std::endl;
    }
    else if(version == full_version_)
    {
      for(size_t i = 0; i < ransac_cloud->points.size(); ++i)
      {
        if(ransac_cloud->points[i].z < z_high_limit_ &&
          ransac_cloud->points[i].z > (-1) * height_from_ground_to_lidar_ + 0.1)
        {
          double distance = pcl::pointToPlaneDistance(ransac_cloud->points[i], normal_vector_.x, normal_vector_.y, normal_vector_.z, (-1) * D_);
          if(distance < 0.1) continue;

          pcl::PointXYZI projection, point;

          projection.x = ransac_cloud->points[i].x;  
          projection.y = ransac_cloud->points[i].y;
          projection.z = (-1) * (normal_vector_.x * ransac_cloud->points[i].x + normal_vector_.y * ransac_cloud->points[i].y - D_) / normal_vector_.z;
          projection.intensity = 2.0;
          pcl_scan_.points.emplace_back(projection);

          point.x = ransac_cloud->points[i].x;  
          point.y = ransac_cloud->points[i].y;
          point.z = ransac_cloud->points[i].z;
          filtered_scan_.points.emplace_back(point);
        }
      }
    }
    
  }

  bool QuadTreeSegmentation::isNearPoint(pcl::PointXYZ searchPoint, pcl::PointXYZ point)
  {
    if(fabs(searchPoint.x - point.x) < 1.0 && fabs(searchPoint.y - point.y) < 1.0)
      return true;
    else
      return false;
  }

  void QuadTreeSegmentation::extractNormalVector()
  {
    if(normal_vector_queue_.size() == 20 && D_queue_.size() == 20)
    {
      float sum_x = 0.0;
      float sum_y = 0.0;
      float sum_z = 0.0;
      float sum_d = 0.0;

      for(size_t k = 0; k<normal_vector_queue_.size(); ++k)
      {
        sum_x += normal_vector_queue_.front().x;
        sum_y += normal_vector_queue_.front().y;
        sum_z += normal_vector_queue_.front().z;
        sum_d += D_queue_.front();
      }

      normal_vector_.x = sum_x / normal_vector_queue_.size();
      normal_vector_.y = sum_y / normal_vector_queue_.size();
      normal_vector_.z = sum_z / normal_vector_queue_.size();
      D_ = sum_d /D_queue_.size();
    
      normal_vector_queue_.pop(); 
      D_queue_.pop();
    }
  }

  void QuadTreeSegmentation::printRansacPlane()
  {
    for(float x = -10.0; x < 10.0; x += 0.5)
    {
      for(float y = -10.0; y < 10.0; y += 0.5)
      {
        geometry_msgs::Point plane_point;
        plane_point.x = x;
        plane_point.y = y;
        plane_point.z = (-1) * (normal_vector_.x * x + normal_vector_.y * y - D_) / normal_vector_.z;
        plane_point_vec_.push_back(plane_point);
      }
    }

    std_msgs::ColorRGBA c;

    int id = 0;
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 1.0;

    for(size_t k = 0; k < plane_point_vec_.size(); ++k)
    {
      points_.header.frame_id = "velodyne";
      points_.header.stamp = lidarTimeStamp_;
      points_.ns = "points";
      points_.action = visualization_msgs::Marker::ADD;
      points_.pose.orientation.w = 1.0;
      points_.lifetime = ros::Duration(0.1);
      points_.id = id++;
      points_.type = visualization_msgs::Marker::POINTS;
      points_.scale.x = 0.1;
      points_.scale.y = 0.1;

      points_.color.r = 0.0;
      points_.color.g = 0.0;
      points_.color.b = 1.0;
      points_.color.a = 1.0;

      points_.points.push_back(plane_point_vec_[k]);
      points_.colors.push_back(c);
    }
  }

  void QuadTreeSegmentation::reset()
  {
    points_.points.resize(0);
    points_.colors.resize(0);
    plane_point_vec_.resize(0);
  }
}
