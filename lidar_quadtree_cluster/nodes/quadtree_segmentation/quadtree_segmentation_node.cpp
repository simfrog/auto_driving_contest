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


#include <quadtree_segmentation/quadtree_segmentation_node.hpp>

namespace quadtree_segmentation_node
{
  QuadTreeSegmentationNode::QuadTreeSegmentationNode() 
    : private_nh("~")
    , ransac_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , points_raw_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , ok_(false)
    , road_based_version_(true)
    , full_version_(false)
    , seq(0)
    , listener(ros::Duration(10))
  {
    if(!private_nh.getParam("points_raw_topic", points_raw_topic))                                    throw std::runtime_error("fail to get points_raw topic name");
    if(!private_nh.getParam("road_points_raw_topic", road_points_raw_topic))                          throw std::runtime_error("fail to get road_points_raw topic name");
    if(!private_nh.getParam("road_points_topic", road_points_topic))                                  throw std::runtime_error("fail to get road_points topic name");
    if(!private_nh.getParam("height_from_ground_to_lidar", height_from_ground_to_lidar_))             throw std::runtime_error("set height_from_ground_to_lidar");
    if(!private_nh.getParam("z_high_limit", z_high_limit_))                                           throw std::runtime_error("set z_high_limit");
    if(!private_nh.getParam("erase_road_threshold", erase_road_threshold_))                           throw std::runtime_error("set erase_road_threshold");
    if(!private_nh.getParam("my_car_width", my_car_width_))                                           throw std::runtime_error("set my_car_width");
    if(!private_nh.getParam("my_car_height", my_car_height_))                                         throw std::runtime_error("set my_car_height");
    if(!private_nh.getParam("max_cluster_size", max_cluster_size_))                                   throw std::runtime_error("set max_cluster_size");
    if(!private_nh.getParam("publish_cloud_clusters_topic_name", publish_cloud_clusters_topic_name_)) throw std::runtime_error("set publish_cloud_clusters_topic_name");
    if(!private_nh.getParam("publish_object_topic_name", publish_objects_topic_name_))                throw std::runtime_error("set publish_object_topic_name");
    if(!private_nh.getParam("istrackdriving", istrackdriving_))                                       throw std::runtime_error("set istrackdriving");

    _pub_plane_print = nh.advertise<visualization_msgs::Marker>("/plane_points", 1);
    _pub_projected_cloud = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 1);
    _pub_none_projected_cloud = nh.advertise<sensor_msgs::PointCloud2>("/none_projected_cloud", 1);
    _pub_detected_boxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_boxes", 1);
    _pub_lidar_detector_objects_polygon = nh.advertise<jsk_recognition_msgs::PolygonArray>("/object_polygons", 1);
    _pub_meter_marker = nh.advertise<visualization_msgs::Marker>("/meter_marker", 1);
    _pub_total_pixel_marker = nh.advertise<visualization_msgs::Marker>("/total_pixel_marker", 1);

    _pub_obb_boxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obb_boxes", 1);
    _pub_map_obb_boxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obb_boxes_map", 1);

    _pub_used_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/used_points_raw", 1);
  

    _sub_points_raw = nh.subscribe(points_raw_topic, 1, &QuadTreeSegmentationNode::callbackGetPointsRaw, this);
    _sub_road_points_raw = nh.subscribe(road_points_raw_topic, 1, &QuadTreeSegmentationNode::callbackGetRoadPointsRaw, this);
    _sub_road_points = nh.subscribe(road_points_topic, 1, &QuadTreeSegmentationNode::callbackGetRoadPoints, this);

    _pub_cloud_clusters = nh.advertise<autoware_msgs::CloudClusterArray>(publish_cloud_clusters_topic_name_,1);
    _pub_lidar_detector_objects = nh.advertise<autoware_msgs::DetectedObjectArray>(publish_objects_topic_name_, 1);
    _pub_lidar_detector_objects_velodyne = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    
    QTS.setParam(height_from_ground_to_lidar_, x_limit_forward_, x_limit_back_, y_limit_, z_high_limit_, erase_road_threshold_);

    //rh_ptr_->spin();

    // meter marker
    points_meter_marker.points.resize(0);
    points_meter_marker.colors.resize(0);

    std::vector<geometry_msgs::Point> vec;

    for(size_t i = 0; i <= 5; ++i)
    {
      geometry_msgs::Point pt;
      pt.x = 50.0 + 10 * i;
      pt.y = 0.0;
      pt.z = 0.0;
      vec.push_back(pt);
    }

    std_msgs::ColorRGBA c;

    int id = 0;
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 1.0;

    for(size_t k = 0; k < vec.size(); ++k)
    {
      points_meter_marker.header.frame_id = "velodyne";
      points_meter_marker.header.stamp = lidarTimeStamp_;
      points_meter_marker.ns = "points";
      points_meter_marker.action = visualization_msgs::Marker::ADD;
      points_meter_marker.pose.orientation.w = 1.0;
      points_meter_marker.lifetime = ros::Duration();
      points_meter_marker.id = id++;
      points_meter_marker.type = visualization_msgs::Marker::POINTS;
      points_meter_marker.scale.x = 1.0;
      points_meter_marker.scale.y = 1.0;

      points_meter_marker.color.r = 0.0;
      points_meter_marker.color.g = 0.0;
      points_meter_marker.color.b = 1.0;
      points_meter_marker.color.a = 1.0;

      points_meter_marker.points.push_back(vec[k]);
      points_meter_marker.colors.push_back(c);
    }

    
  }

  QuadTreeSegmentationNode::~QuadTreeSegmentationNode()
  {

  }

  void QuadTreeSegmentationNode::setScan(quadtree_segmentation::QuadTreeSegmentation& QTS)
  {
    filtered_scan_ = QTS.getFilteredScan();
    non_filtered_scan_ = QTS.getNoneFilteredScan();
    pcl_scan_ = QTS.getPclScan();
    non_pcl_scan_ = QTS.getNonePclScan();

    points_raw_scan_ = QTS.getPointsRawScan();
    road_points_scan_ = QTS.getRoadPointsScan();
    ransac_cloud_ = QTS.getRansacCloud();
    points_raw_cloud_ = QTS.getPointsRawCloud();
  }

  void QuadTreeSegmentationNode::setPointsForPlanePrint(quadtree_segmentation::QuadTreeSegmentation& QTS)
  {
    points_ = QTS.getPoints();
  }

  void QuadTreeSegmentationNode::setQuadtreeScan(quadtree_segmentation::QuadTreeSegmentation& QTS)
  {
    filtered_scan_ = QTS.getFilteredScan();
    non_filtered_scan_ = QTS.getNoneFilteredScan();
    pcl_scan_ = QTS.getPclScan();
    non_pcl_scan_ = QTS.getNonePclScan();
  }

  void QuadTreeSegmentationNode::callbackTimer(const ros::TimerEvent& event)
  {
    ok_ = false;
  }

  void QuadTreeSegmentationNode::callbackGetRoadPoints(const sensor_msgs::PointCloud2::ConstPtr &ptr)
  {
    pcl::fromROSMsg(*ptr, temp_scan_1);
    QTS.setInputRoad(temp_scan_1);

    if(ptr->data.size() != 0)
      ok_ = true;
    else
      ok_ = false;
    
  }

  void QuadTreeSegmentationNode::callbackGetRoadPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr)
  {
    pcl::fromROSMsg(*ptr, road_points_raw_scan_);
    QTS.setInputRoadPointsRaw(road_points_raw_scan_);
  }

  /* main callback */
  void QuadTreeSegmentationNode::callbackGetPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &ptr)
  {
    lidarTimeStamp_ = ptr->header.stamp;
    QTS.setTimeStamp(lidarTimeStamp_);
    QT.setTimeStamp(lidarTimeStamp_);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);	
    // pcl::PointCloud<pcl::PointXYZ>::Ptr lateral_filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*ptr, *input_cloud_ptr);
    // lateralFilter(input_cloud_ptr, lateral_filter_cloud_ptr, 10.0, 10.0);
    // QTS.setInputCloud(*lateral_filter_cloud_ptr.get());
    // QTS.setInputCloud(input_cloud_ptr);
    pcl::fromROSMsg(*ptr, temp_scan_2);
    QTS.setInputCloud(temp_scan_2);
    QTS.initScan();
    setScan(QTS);

    if(ok_ == true)
    {
      // ROS_INFO("ROAD based Version!!");

      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(points_raw_scan_.makeShared());
      // vg.setMinimumPointsNumberPerVoxel(2); // erase noise
      vg.setLeafSize(0.1f,0.1f,0.1f); // for RANSAC
      vg.filter(*ransac_cloud_); // for RANSAC
      vg.setLeafSize(0.01f,0.0f,0.01f); // for voxeling points_raw
      vg.filter(*points_raw_cloud_); // for voxeling points_raw

      QTS.makePlaneRANSAC(ROAD_EXTRACTOR); // road version
      //QTS.makePlaneRANSAC(LIDAR_FILTER); // road version + lidar filter version
      
      QTS.projectionOntoPlane(road_based_version_, ransac_cloud_, points_raw_cloud_, road_points_scan_); 
      //QTS.printRansacPlane();
    }
    else
    {
     ROS_INFO("FULL Version");

      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(points_raw_scan_.makeShared());
      vg.setLeafSize(0.1f,0.1f,0.1f);
      //vg.setMinimumPointsNumberPerVoxel(2); // erase noise
      vg.filter(*ransac_cloud_);
      
      QTS.makePlaneRANSAC(LIDAR_FILTER);
      QTS.projectionOntoPlane(full_version_, points_raw_scan_.makeShared(), points_raw_cloud_, road_points_scan_);
      QTS.printRansacPlane();
    }
    
    setQuadtreeScan(QTS);
    
    /* debug start */
    sensor_msgs::PointCloud2 projected_cloud;
    sensor_msgs::PointCloud2 non_projected_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan_));
    pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr_sub(new pcl::PointCloud<pcl::PointXYZI>(non_pcl_scan_));
    pcl::toROSMsg(*n_ptr, projected_cloud);
    pcl::toROSMsg(*n_ptr_sub, non_projected_cloud);
    n_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    n_ptr_sub.reset(new pcl::PointCloud<pcl::PointXYZI>);
    setPointsForPlanePrint(QTS);
    _pub_plane_print.publish(points_); // printRansacPlane
    _pub_projected_cloud.publish(projected_cloud); // print projected cloud
    _pub_none_projected_cloud.publish(non_projected_cloud); //print non_projected_cloud
    /* debug end */

    /*labeling*/
    pcl::PointCloud<pcl::PointXYZI> used_points_raw;
    copyPointCloud(points_raw_scan_, used_points_raw);
    pcl::toROSMsg(used_points_raw, used_point_cloud_);
    
    //////////////////////////////////////////////

    sensor_msgs::PointCloud2 filtered_msg, pcl_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filtered_scan_));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan_));
    pcl::toROSMsg(*scan_ptr, filtered_msg);
    pcl::toROSMsg(*pcl_ptr, pcl_msg);
    scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

    /* quadtree*/
    clusteredBox_ = QT.quadTreeMain(&filtered_msg, &pcl_msg);
    _pub_detected_boxes.publish(clusteredBox_);

    visualization_msgs::Marker total_pixel_points = QT.getPoints();
    _pub_total_pixel_marker.publish(total_pixel_points);

    /* obb */
    cloudarray_ = QT.getCloudArray();
    zarray_ = QT.getZArray();
    makeBoundingBoxes(cloudarray_);
    clusteredBox_.boxes.resize(0);
    QT.reset();
    QTS.reset();

    cloudarray_.resize(0);
    
    ransac_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    points_raw_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    _pub_meter_marker.publish(points_meter_marker);

  }

  void QuadTreeSegmentationNode::lateralFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, double in_left_lane_threshold = 12.0,
                    double in_right_lane_threshold = 12.0)
{
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
    {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

  void QuadTreeSegmentationNode::makeBoundingBoxes(std::vector<sensor_msgs::PointCloud2>& cloudarray)
  {
    pcl::PointCloud<pcl::PointXYZ> scan;

    jsk_recognition_msgs::BoundingBox obb;
    jsk_recognition_msgs::BoundingBoxArray obb_arr;

    obb_arr.header.frame_id = "velodyne";
    obb_arr.header.seq = seq++;
    obb_arr.header.stamp = lidarTimeStamp_;

    int z_idx = 0;
    zarray_filtered_.resize(0);

    for(size_t i = 0; i < cloudarray.size(); i++)
    {
      pcl::fromROSMsg(cloudarray[i], scan);
      
      if(scan.points.size() > (unsigned int)max_cluster_size_) continue;

      std::vector<cv::Point2f> points;
      if(scan.points.size() > 2)
      {
        for(size_t k = 0; k < scan.points.size(); k++)
        {
          cv::Point2f pt;
          pt.x = scan.points[k].x + 0.05;
          pt.y = scan.points[k].y + 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x + 0.05;
          pt.y = scan.points[k].y - 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x - 0.05;
          pt.y = scan.points[k].y + 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x - 0.05;
          pt.y = scan.points[k].y - 0.05;
          points.push_back(pt);
        }
      }
      else
      {
        for(size_t k = 0; k < scan.points.size(); k++)
        {
          cv::Point2f pt;
          pt.x = scan.points[k].x + 0.05;
          pt.y = scan.points[k].y + 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x + 0.05;
          pt.y = scan.points[k].y - 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x - 0.05;
          pt.y = scan.points[k].y + 0.05;
          points.push_back(pt);
          pt.x = scan.points[k].x - 0.05;
          pt.y = scan.points[k].y - 0.05;
          points.push_back(pt);
        }
      }

      std::vector<cv::Point2f> hull;
      cv::convexHull(points, hull);
      hull_vec_.push_back(hull);

      cv::RotatedRect box = minAreaRect(hull);

      if(fabs(box.center.x) < my_car_height_ && fabs(box.center.y) < my_car_width_)
      {
        z_idx++;
        hull_vec_.pop_back();
        continue;
      }

      zarray_filtered_.push_back(zarray_[i]);
      cloudarray_filtered_.push_back(cloudarray[i]);
    }
    obb_arr = mergeNearPolygons();
    if(istrackdriving_ == false)
    {
      makeDetectedObjects(obb_arr);
      makeCloudClusters(obb_arr);
    }
      
    makeDetectedObjects_velodyne(obb_arr);
    

    _pub_obb_boxes.publish(obb_arr);
    _pub_used_point_cloud.publish(used_point_cloud_); // labeling
    hull_vec_.resize(0);
  }

  jsk_recognition_msgs::BoundingBoxArray QuadTreeSegmentationNode::mergeNearPolygons()
  {
    std::vector<cv::Point2f> polygon_center_vec;
    jsk_recognition_msgs::BoundingBox obb;
    jsk_recognition_msgs::BoundingBoxArray obb_arr;

    obb_arr.header.frame_id = "velodyne";
    obb_arr.header.seq = seq++;
    obb_arr.header.stamp = lidarTimeStamp_;

    int z_idx = 0;

    // get polygon center
    std::vector<double> area_vec;

    for(size_t i = 0; i < hull_vec_.size(); ++i)
    {
      cv::Point2f center_point = getCenterPoint(hull_vec_[i]);
      polygon_center_vec.push_back(center_point);

      area_vec.push_back(cv::contourArea(hull_vec_[i]));
    }

    // set cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    for(size_t i = 0; i < polygon_center_vec.size(); ++i)
    {
      pcl::PointXYZ point;
      point.x = polygon_center_vec[i].x;
      point.y = polygon_center_vec[i].y;
      point.z = 0.0;

      cloud.points.push_back(point);
    }
    // std::cout << cloud.points.size() << std::endl;

    // merge
    pcl::PointCloud<pcl::PointXYZ>::Ptr octree_input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    std::unordered_map<int, std::vector<int>> umap;
    std::vector<bool> used_check_vec;
    std::vector<int> parent;

    used_check_vec.assign(cloud.points.size(), false);

    for(int i = 0; i < cloud.points.size(); i++)
      parent.push_back(i);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(1.0);
    octree.setInputCloud(octree_input_cloud_ptr);
    octree.addPointsFromInputCloud();

    
    /* union find algorithm */
    for(size_t k = 0; k < cloud.points.size(); ++k)
    {
      pcl::PointXYZ searchPoint;
      
      searchPoint = cloud.points[k];
      used_check_vec[k] = true;

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      double radius = 0.7;
      
      // std::cout << "radius : " << radius << "   searchpoint : "<< searchPoint.x << " , " << searchPoint.y << " , " << searchPoint.z << std::endl;
      if(octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
          if(fabs(searchPoint.y - cloud.points[pointIdxRadiusSearch[i]].y) > 1.2) continue;

          if(getParent(parent, k) == k) // don't have parent
          {
            if(getParent(parent, pointIdxRadiusSearch[i]) == pointIdxRadiusSearch[i]) // don't have parent
            {
              unionParent(parent, k, pointIdxRadiusSearch[i]);
            }
            else // have parent
            {
              unionParent(parent, k, getParent(parent, pointIdxRadiusSearch[i]));
            }
          }
          else  // have parent
          {
            unionParent(parent, k, getParent(parent, pointIdxRadiusSearch[i]));
          }
        }
      }
    }

    // set unordered map
    for(size_t i = 0; i < parent.size(); i++)
      umap[getParent(parent, i)].push_back(i);

    std::vector<std::vector<cv::Point2f>> hull_vec_temp;

    obb_arr.boxes.resize(0);

    std::unordered_map<int, std::vector<int>>::iterator it;

    cloudarray_merged_.resize(0);
    zarray_merged_.resize(0);
    for(it = umap.begin(); it != umap.end(); ++it)
    {
      std::vector<cv::Point2f> points;
      std::vector<int> index_vec;
      
      pcl::PointCloud<pcl::PointXYZ> scan_merged;

      double longest_local_z = -999; 

      for (size_t k = 0; k < it->second.size(); ++k)
      {
        points.insert(points.end(), hull_vec_[it->second[k]].begin(), hull_vec_[it->second[k]].end());

        //cloud clusters
        pcl::PointCloud<pcl::PointXYZ> scan;
        pcl::fromROSMsg(cloudarray_filtered_[it->second[k]], scan);
        scan_merged.points.insert(scan_merged.points.end(), scan.points.begin(), scan.points.end());

        if(longest_local_z < zarray_filtered_[it->second[k]])
          longest_local_z = zarray_filtered_[it->second[k]];
      }

      sensor_msgs::PointCloud2 msg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>(scan_merged));
      pcl::toROSMsg(*ptr, msg);

      cloudarray_merged_.push_back(msg);
      zarray_merged_.push_back(longest_local_z);

      size_t size = points.size();

      for(size_t k = 0; k < size; ++k)
      {
        cv::Point2f pt;
        pt.x = points[k].x + 0.01;
        pt.y = points[k].y + 0.01;
        points.push_back(pt);
        pt.x = points[k].x + 0.01;
        pt.y = points[k].y - 0.01;
        points.push_back(pt);
        pt.x = points[k].x - 0.01;
        pt.y = points[k].y + 0.01;
        points.push_back(pt);
        pt.x = points[k].x - 0.01;
        pt.y = points[k].y - 0.01;
        points.push_back(pt);
      }

      // std::cout << "8888" << std::endl;
      std::vector<cv::Point2f> hull;
      cv::convexHull(points, hull);
      hull_vec_temp.push_back(hull);

      obb = estimate(scan_merged, longest_local_z);
      obb_arr.boxes.emplace_back(obb);
    }

    hull_vec_.resize(0);
    cloudarray_filtered_.resize(0);
    hull_vec_ = hull_vec_temp;
    // std::cout << "4" << std::endl;
    return obb_arr;
  }

  double QuadTreeSegmentationNode::setRadius(pcl::PointXYZ searchPoint) // fix 20210103
  {
      double radius;
      
      double dist = sqrt(pow(searchPoint.x, 2) + pow(searchPoint.y, 2));

      if(dist <= 50)
        radius = 2.75;
      else if(dist <= 65)
        radius = 3.0;
      else if(dist <= 80)
        radius = 3.25;
      else
        radius = 3.5;

      return radius;
  }

  cv::Point2f QuadTreeSegmentationNode::getCenterPoint(std::vector<cv::Point2f> polygon)
  {
    /* version1 */
    cv::Point2f pt;

    double area = 0.0;
    double center_x = 0.0;
    double center_y = 0.0;

    for(size_t i = 0; i < polygon.size(); ++i)
    {
      int j = (i + 1) % polygon.size();

      cv::Point2f pt1, pt2;

      pt1 = polygon[i];
      pt2 = polygon[j];

      area += pt1.x * pt2.y;
      area -= pt1.y * pt2.x;

      center_x += ((pt1.x + pt2.x) * ((pt1.x * pt2.y) - (pt2.x * pt1.y)));
      center_y += ((pt1.y + pt2.y) * ((pt1.x * pt2.y) - (pt2.x * pt1.y)));
    }

    area /= 2.0;
    area = fabs(area);

    pt.x = center_x / (6 * area);
    pt.y = center_y / (6 * area);

    return pt;
  }
  
  void QuadTreeSegmentationNode::makeDetectedObjects_velodyne(jsk_recognition_msgs::BoundingBoxArray& in_objects)
  {
    autoware_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_objects.header;

    jsk_recognition_msgs::PolygonArray object_polygons;
    object_polygons.header = in_objects.header;

    size_t index = 0;

    for (auto& object : in_objects.boxes)
    {
      geometry_msgs::PolygonStamped object_polygon;
      object_polygon.header = in_objects.header;

      autoware_msgs::DetectedObject detected_object;
      detected_object.header = in_objects.header;
      detected_object.label = "unknown";
      detected_object.score = 1.;
      detected_object.space_frame = in_objects.header.frame_id;
    
      detected_object.pose = object.pose;
      detected_object.dimensions = object.dimensions;

      if(istrackdriving_ == true){
        if(object.dimensions.z > 0.8) continue;
        if(object.dimensions.x > 0.6) continue;
        if(object.dimensions.y > 0.6) continue;
      }
        

      for(size_t k = 0; k < hull_vec_[index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[index][k % hull_vec_[index].size()].x;
        point.y = hull_vec_[index][k % hull_vec_[index].size()].y;
        point.z = object.pose.position.z - object.dimensions.z / 2.0;
        detected_object.convex_hull.polygon.points.push_back(point);
        object_polygon.polygon.points.push_back(point);
      }
      
      for(size_t k = 0; k < hull_vec_[index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[index][k % hull_vec_[index].size()].x;
        point.y = hull_vec_[index][k % hull_vec_[index].size()].y;
        point.z = object.pose.position.z + object.dimensions.z / 2.0;
        detected_object.convex_hull.polygon.points.push_back(point);
        object_polygon.polygon.points.push_back(point);
      }

      index++;
      detected_object.valid = true;
      detected_objects.objects.push_back(detected_object);
      object_polygons.polygons.push_back(object_polygon);
    }

    _pub_lidar_detector_objects_velodyne.publish(detected_objects);
    if(istrackdriving_==true) 
      _pub_lidar_detector_objects_polygon.publish(object_polygons);
  }

  void QuadTreeSegmentationNode::makeDetectedObjects(jsk_recognition_msgs::BoundingBoxArray& in_objects)
  {
    tf::StampedTransform transform; // add
	  GetTransformFromTF("/map", "/velodyne", transform); // add

    autoware_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_objects.header;
    detected_objects.header.frame_id = "/map"; // add

    jsk_recognition_msgs::PolygonArray object_polygons;
    object_polygons.header = in_objects.header;

    jsk_recognition_msgs::BoundingBoxArray map_objects;
    map_objects.header = in_objects.header;
    map_objects.header.frame_id = "/map";

    size_t index = 0;
    size_t id = 1;

    for (auto& object : in_objects.boxes)
    {
      geometry_msgs::PolygonStamped object_polygon;
      object_polygon.header = in_objects.header;

      autoware_msgs::DetectedObject detected_object;
      detected_object.id = id++;
      detected_object.header = in_objects.header;
      detected_object.header.frame_id = "/map"; // add
      detected_object.label = "unknown";
      detected_object.score = 1.;
      detected_object.indicator_state = 3;
      detected_object.velocity.linear.x = 0.0;
      detected_object.velocity.linear.y = 0.0;
      detected_object.velocity.linear.z = 0.0;
      detected_object.velocity_reliable = false;
      detected_object.pose_reliable = true;


      pcl::PointXYZ input1; // add
      input1.x = object.pose.position.x;// add
      input1.y = object.pose.position.y;// add
      input1.z = object.pose.position.z;// add

      pcl::PointXYZ result1 = TransformPoint(input1, transform);// add

      //detected_object.pose = object.pose;
      detected_object.pose.position.x = result1.x;
      detected_object.pose.position.y = result1.y;
      detected_object.pose.position.z = result1.z;
      detected_object.pose.orientation = object.pose.orientation;
      detected_object.dimensions = object.dimensions;

      for(size_t k = 0; k < hull_vec_[index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[index][k % hull_vec_[index].size()].x;
        point.y = hull_vec_[index][k % hull_vec_[index].size()].y;
        point.z = object.pose.position.z - object.dimensions.z / 2.0;

        pcl::PointXYZ input2; // add
        input2.x = point.x;// add
        input2.y = point.y;// add
        input2.z = point.z;// add

        pcl::PointXYZ result2 = TransformPoint(input2, transform);// add
        
        geometry_msgs::Point32 point2;// add
        point2.x = result2.x;// add
        point2.y = result2.y;// add
        point2.z = result2.z;// add

        if(k != 0)
        {
          geometry_msgs::Point32 temp1, temp2, temp3;
          temp1.x = (detected_object.convex_hull.polygon.points[k-1].x + point2.x) / 2.0;
          temp1.y = (detected_object.convex_hull.polygon.points[k-1].y + point2.y) / 2.0;
          temp1.z = (detected_object.convex_hull.polygon.points[k-1].z + point2.z) / 2.0;

          detected_object.convex_hull.polygon.points.push_back(temp1);
        }

        detected_object.convex_hull.polygon.points.push_back(point2);
        object_polygon.polygon.points.push_back(point);
      }
      
      for(size_t k = 0; k < hull_vec_[index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[index][k % hull_vec_[index].size()].x;
        point.y = hull_vec_[index][k % hull_vec_[index].size()].y;
        point.z = object.pose.position.z + object.dimensions.z / 2.0;

        pcl::PointXYZ input2; // add
        input2.x = point.x;// add
        input2.y = point.y;// add
        input2.z = point.z;// add

        pcl::PointXYZ result2 = TransformPoint(input2, transform);// add
        
        geometry_msgs::Point32 point2;// add
        point2.x = result2.x;// add
        point2.y = result2.y;// add
        point2.z = result2.z;// add

        if(k != 0)
        {
          geometry_msgs::Point32 temp1, temp2, temp3;
          temp1.x = (detected_object.convex_hull.polygon.points[k-1].x + point2.x) / 2.0;
          temp1.y = (detected_object.convex_hull.polygon.points[k-1].y + point2.y) / 2.0;
          temp1.z = (detected_object.convex_hull.polygon.points[k-1].z + point2.z) / 2.0;

          detected_object.convex_hull.polygon.points.push_back(temp1);
        }
        
        detected_object.convex_hull.polygon.points.push_back(point2);
        object_polygon.polygon.points.push_back(point);
      }

      //sort
      
      index++;
      detected_object.valid = true;
      detected_objects.objects.push_back(detected_object);
      object_polygons.polygons.push_back(object_polygon);

      jsk_recognition_msgs::BoundingBox output;
      tf2::Quaternion quat;
      quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ 0);
      output.header.frame_id = "map";
      output.header.stamp = lidarTimeStamp_;
      output.pose.position.x = detected_object.pose.position.x;
      output.pose.position.y = detected_object.pose.position.y;
      output.pose.position.z = detected_object.pose.position.z;
      output.pose.orientation = tf2::toMsg(quat);
      output.dimensions = detected_object.dimensions;
    
      map_objects.boxes.push_back(output);
    }
    
    _pub_map_obb_boxes.publish(map_objects);
    _pub_lidar_detector_objects.publish(detected_objects);
    _pub_lidar_detector_objects_polygon.publish(object_polygons);
  }

  void QuadTreeSegmentationNode::makeCloudClusters(jsk_recognition_msgs::BoundingBoxArray& obb_array)
  {
    tf::StampedTransform transform;
	  GetTransformFromTF("/map", "/velodyne", transform);

    autoware_msgs::CloudCluster cc;
    autoware_msgs::CloudClusterArray cca;

    cca.header.frame_id = "/map";
    cca.header.stamp = lidarTimeStamp_;
    
    int index = 0;
    int convex_hull_index = 0;

    for (auto& box : obb_array.boxes)
    {
      pcl::PointXYZ input1;
      input1.x = box.pose.position.x;
      input1.y = box.pose.position.y;
      input1.z = box.pose.position.z;

      pcl::PointXYZ result1 = TransformPoint(input1, transform);

      cc.centroid_point.point.x = result1.x;
      cc.centroid_point.point.y = result1.y;
      cc.centroid_point.point.z = result1.z;
      cc.estimated_angle = tf::getYaw(box.pose.orientation);

      cc.dimensions.x = box.dimensions.x;
      cc.dimensions.y = box.dimensions.y;
      cc.dimensions.z = box.dimensions.z;

      cc.label = "none";
      cc.indicator_state = 3;
  
      // 2D points group
      sensor_msgs::PointCloud2 msg;
      pcl::PointCloud<pcl::PointXYZ> temp_scan;
      pcl::fromROSMsg(cloudarray_merged_[index++], temp_scan);
      pcl::PointCloud<pcl::PointXYZ> tmp;
      pcl_ros::transformPointCloud(temp_scan, tmp, transform);
      pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>(tmp));
      pcl::toROSMsg(*ptr, msg);
    
      cc.cloud = msg;
      
      // make convex_hull
      geometry_msgs::PolygonStamped object_polygon;
      object_polygon.header = cca.header;

      for(size_t k = 0; k < hull_vec_[convex_hull_index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[convex_hull_index][k % hull_vec_[convex_hull_index].size()].x;
        point.y = hull_vec_[convex_hull_index][k % hull_vec_[convex_hull_index].size()].y;
        point.z = box.pose.position.z - box.dimensions.z / 2.0;

        pcl::PointXYZ input2;
        input2.x = point.x;
        input2.y = point.y;
        input2.z = point.z;

        pcl::PointXYZ result2 = TransformPoint(input2, transform);
        
        geometry_msgs::Point32 point2;
        point2.x = result2.x;
        point2.y = result2.y;
        point2.z = result2.z;
      
        object_polygon.polygon.points.push_back(point2);
      }
      
      for(size_t k = 0; k < hull_vec_[convex_hull_index].size() + 1; k++)
      {
        geometry_msgs::Point32 point;
        point.x = hull_vec_[convex_hull_index][k % hull_vec_[convex_hull_index].size()].x;
        point.y = hull_vec_[convex_hull_index][k % hull_vec_[convex_hull_index].size()].y;
        point.z = box.pose.position.z + box.dimensions.z / 2.0;

        pcl::PointXYZ input2;
        input2.x = point.x;
        input2.y = point.y;
        input2.z = point.z;

        pcl::PointXYZ result2 = TransformPoint(input2, transform);
        
        geometry_msgs::Point32 point2;
        point2.x = result2.x;
        point2.y = result2.y;
        point2.z = result2.z;
      
        object_polygon.polygon.points.push_back(point2);
      }
      cc.convex_hull = object_polygon;

      // bounding box
      cc.bounding_box = box;

      cca.clusters.push_back(cc);
      convex_hull_index++;
    }
    
    _pub_cloud_clusters.publish(cca);
  }

  void QuadTreeSegmentationNode::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
  {
    static tf::TransformListener listener;

    int nFailedCounter = 0;
    while (1)
    {
      try
      {
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
        break;
      }
      catch (tf::TransformException& ex)
      {
        if(nFailedCounter > 2)
        {
          ROS_ERROR("%s", ex.what());
        }
        ros::Duration(1.0).sleep();
        nFailedCounter ++;
      }
    }
  }

  pcl::PointXYZ QuadTreeSegmentationNode::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
  {
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
  }

  int QuadTreeSegmentationNode::getParent(std::vector<int> &parent, int x)
  {
    if(parent[x] == x) return x;
    return parent[x] = getParent(parent, parent[x]); 
  }

  void QuadTreeSegmentationNode::unionParent(std::vector<int> &parent, int a, int b)
  {
    a = getParent(parent, a);
    b = getParent(parent, b);

    parent[b] = a;
  }

  int QuadTreeSegmentationNode::findParent(std::vector<int> &parent, int a, int b)
  {
    a = getParent(parent, a);
    b = getParent(parent, b);
    if(a == b) return 1;
    else return 0;    
  }

  jsk_recognition_msgs::BoundingBox QuadTreeSegmentationNode::estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, double longest_local_z )
  {
    jsk_recognition_msgs::BoundingBox output;
    // calc centroid point for cylinder height(z)
    pcl::PointXYZ centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;
    for (const auto& pcl_point : cluster)
    {
      centroid.x += pcl_point.x;
      centroid.y += pcl_point.y;
      centroid.z += pcl_point.z;
    }
    centroid.x = centroid.x / (double)cluster.size();
    centroid.y = centroid.y / (double)cluster.size();
    centroid.z = centroid.z / (double)cluster.size();

    // calc min and max z for cylinder length
    double min_z = (-1) * height_from_ground_to_lidar_;
    double max_z = longest_local_z;
    // for (size_t i = 0; i < cluster.size(); ++i)
    // {
    //   if (cluster.at(i).z < min_z || i == 0)
    //     min_z = cluster.at(i).z;
    //   if (max_z < cluster.at(i).z || i == 0)
    //     max_z = cluster.at(i).z;
    // }

    // calc circumscribed circle on x-y plane
    cv::Mat_<float> cv_points((int)cluster.size(), 2);
    for (size_t i = 0; i < cluster.size(); ++i)
    {
      cv_points(i, 0) = cluster.at(i).x;  // x
      cv_points(i, 1) = cluster.at(i).y;  // y
    }

    /*
    * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
    * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
    */

    // Paper : Algo.2 Search-Based Rectangle Fitting
    std::vector<std::pair<double /*theta*/, double /*q*/>> Q;
    const double max_angle = M_PI / 2.0;
    const double angle_reso = M_PI / 180.0;
    for (double theta = 0; theta < max_angle; theta += angle_reso)
    {
      Eigen::Vector2d e_1;
      e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
      Eigen::Vector2d e_2;
      e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
      std::vector<double> C_1;                   // col.5, Algo.2
      std::vector<double> C_2;                   // col.6, Algo.2
      for (const auto& point : cluster)
      {
        C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
        C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
      }
      double q = calcClosenessCriterion(C_1, C_2);  // col.7, Algo.2
      Q.push_back(std::make_pair(theta, q));        // col.8, Algo.2
    }

    double theta_star;  // col.10, Algo.2
    double max_q;
    for (size_t i = 0; i < Q.size(); ++i)
    {
      if (max_q < Q.at(i).second || i == 0)
      {
        max_q = Q.at(i).second;
        theta_star = Q.at(i).first;
      }
    }

    Eigen::Vector2d e_1_star;  // col.11, Algo.2
    Eigen::Vector2d e_2_star;
    e_1_star << std::cos(theta_star), std::sin(theta_star);
    e_2_star << -std::sin(theta_star), std::cos(theta_star);
    std::vector<double> C_1_star;  // col.11, Algo.2
    std::vector<double> C_2_star;  // col.11, Algo.2
    for (const auto& point : cluster)
    {
      C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
      C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
    }

    // col.12, Algo.2
    const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
    const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
    const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
    const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

    const double a_1 = std::cos(theta_star);
    const double b_1 = std::sin(theta_star);
    const double c_1 = min_C_1_star;
    const double a_2 = -1.0 * std::sin(theta_star);
    const double b_2 = std::cos(theta_star);
    const double c_2 = min_C_2_star;
    const double a_3 = std::cos(theta_star);
    const double b_3 = std::sin(theta_star);
    const double c_3 = max_C_1_star;
    const double a_4 = -1.0 * std::sin(theta_star);
    const double b_4 = std::cos(theta_star);
    const double c_4 = max_C_2_star;

    // calc center of bounding box
    double intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
    double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
    double intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
    double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

    // calc dimention of bounding box
    Eigen::Vector2d e_x;
    Eigen::Vector2d e_y;
    e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
    e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
    Eigen::Vector2d diagonal_vec;
    diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

    // calc yaw
    tf2::Quaternion quat;
    quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));
    output.header.frame_id = "velodyne";
    output.header.stamp = lidarTimeStamp_;
    output.pose.position.x = (intersection_x_1 + intersection_x_2) / 2.0;
    output.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
    output.pose.position.z = (longest_local_z + height_from_ground_to_lidar_) / 2 - height_from_ground_to_lidar_;
    output.pose.orientation = tf2::toMsg(quat);
    constexpr double ep = 0.001;
    output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
    output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
    output.dimensions.z = std::max((max_z - min_z), ep);

    output.dimensions.x = std::max(output.dimensions.x, ep);
    output.dimensions.y = std::max(output.dimensions.y, ep);
    
    return output;
  }

  double QuadTreeSegmentationNode::calcClosenessCriterion(const std::vector<double>& C_1, const std::vector<double>& C_2)
  {
    // Paper : Algo.4 Closeness Criterion
    const double min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
    const double min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
    const double max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

    std::vector<double> D_1;  // col.4, Algo.4
    for (const auto& c_1_element : C_1)
    {
      const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
      D_1.push_back(std::fabs(v));
    }

    std::vector<double> D_2;  // col.5, Algo.4
    for (const auto& c_2_element : C_2)
    {
      const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
      D_2.push_back(v * v);
    }

    const double d_min = 0.05;
    const double d_max = 0.50;
    double beta = 0;  // col.6, Algo.4
    for (size_t i = 0; i < D_1.size(); ++i)
    {
      const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
      beta += 1.0 / d;
    }
    return beta;
  }
}



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "quadtree_segmentation_node");

  quadtree_segmentation_node::QuadTreeSegmentationNode QTSN;

  ros::spin();

  return 0;
}
