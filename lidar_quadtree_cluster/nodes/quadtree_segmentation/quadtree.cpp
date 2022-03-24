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
 
#include <quadtree_segmentation/quadtree.hpp>

namespace quadtree
{
  std::vector< std::vector<XYI> > total_pixel_;
  jsk_recognition_msgs::BoundingBoxArray totalBox_;
  pcl::PointCloud<pcl::PointXYZ> octree_input_cloud;

  QuadTree::QuadTree() : filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>), seq(0)
  {
    // default_x_ = 0.0;
    // default_y_ = 0.0;
    // default_size_ = 100.0;
    // default_pixel_ = 513;
    boxSize_ = 0.390625;
    start_Xindex_ = 256;
    start_Yindex_ = 255;
    //box_z_ = 0.00001;
    // point_pixel_x_ = 1025;
    // point_pixel_y_ = 1025;
    // pixel_Xmax_ = -50;
    // pixel_Ymax_ = 50;

    // box_threshold_ = 0.05;
    setParam();
    
    this->children_[0] = NULL;
    this->children_[1] = NULL;
    this->children_[2] = NULL;
    this->children_[3] = NULL;

    rect_ = Box();

    haschildren_ = false;
    haspoints_ = true;

    clusteredBox_.boxes.resize(0);
    clusteredBox_.header.seq = totalBox_.header.seq;
    clusteredBox_.header.stamp = lidarTimeStamp_;
    clusteredBox_.header.frame_id = "velodyne";
  }

  QuadTree::QuadTree(Box& re)
  {

    // default_x_ = 0.0;
    // default_y_ = 0.0;
    // default_size_ = 100.0;
    // default_pixel_ = 513;
    boxSize_ = 0.390625;
    start_Xindex_ = 256;
    start_Yindex_ = 255;
    //box_z_ = 0.00001;
    // point_pixel_x_ = 1025;
    // point_pixel_y_ = 1025;
    // pixel_Xmax_ = -50;
    // pixel_Ymax_ = 50;
    // box_threshold_ = 0.05;

    setParam();

    this->children_[0] = NULL;
    this->children_[1] = NULL;
    this->children_[2] = NULL;
    this->children_[3] = NULL;

    rect_ = re;
    haschildren_ = false;
    haspoints_ = true;
  }


  QuadTree::~QuadTree()
  {

  }

  jsk_recognition_msgs::BoundingBoxArray QuadTree::quadTreeMain(sensor_msgs::PointCloud2 *in_points, sensor_msgs::PointCloud2 *pcl_points)
  {
    points_msg_ = *in_points;
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::PointCloud<pcl::PointXYZI> pcl;
    pcl::fromROSMsg(*in_points, scan);
    pcl::fromROSMsg(*pcl_points, pcl);

    double poseX = point_pixel_x_ / 2;
    double poseY = (point_pixel_y_ / 2);
    int indexX, indexY;

    initPixel();
    initTotalBox();

    // for(auto point : pcl.points)
    // {
    //   indexX = poseX + (round(point.x / boxSize_));
    //   indexY = poseY - (round(point.y / boxSize_));

    //   if(indexX > 512) indexX = 512;
    //   if(indexX < 0) indexX = 0;
    //   if(indexY > 512) indexY = 512;
    //   if(indexY < 0) indexY = 0;

    //   if(total_pixel_[indexY][indexX].maxZ < point.z)
    //     total_pixel_[indexY][indexX].maxZ = point.z;

    //   total_pixel_[indexY][indexX].count++;
    // }

    // for(auto point : scan.points)
    // {
    //   indexX = poseX + (round(point.x / boxSize_));
    //   indexY = poseY - (round(point.y / boxSize_));

    //   if(indexX > 512) indexX = 512;
    //   if(indexX < 0) indexX = 0;
    //   if(indexY > 512) indexY = 512;
    //   if(indexY < 0) indexY = 0;

    //   total_pixel_[indexY][indexX].points_vec.push_back(point);
    // }

    for(size_t i = 0; i < scan.points.size(); ++i)
    {
      indexX = poseX + (round(pcl.points[i].x / boxSize_));
      indexY = poseY - (round(pcl.points[i].y / boxSize_));

      if(indexX > 512) indexX = 512;
      if(indexX < 0) indexX = 0;
      if(indexY > 512) indexY = 512;
      if(indexY < 0) indexY = 0;

      if(total_pixel_[indexY][indexX].maxZ < scan.points[i].z)
        total_pixel_[indexY][indexX].maxZ = scan.points[i].z;

      total_pixel_[indexY][indexX].count++;
      total_pixel_[indexY][indexX].points_vec.push_back(scan.points[i]);
    }

    printTotalPixel();
    divideQuadTree2();
    makeGroup_Octree();


    total_pixel_.resize(0);
    return clusteredBox_;

  }

  void QuadTree::initPixel()
  {
    XYI xyi;
    std::vector<XYI> xyi_vec;
    point_pixel_x_ = 513;
    point_pixel_y_ = 513;
    double poseX = static_cast<double>(point_pixel_x_);
    double poseY = static_cast<double>(point_pixel_y_);
    double next_x, next_y;
    next_y = boxSize_;
    for(int i = 0; i < point_pixel_y_; ++i)
    {
      xyi_vec.resize(0);
      next_x = boxSize_;
      for(int j = 0; j < point_pixel_x_; ++j)
      {
        xyi.count = 0;
        xyi.x = -poseX + next_x;
        xyi.y = poseY - next_y;
        xyi.maxZ = (-1) * height_from_ground_to_lidar_;
        next_x = next_x + boxSize_;
        xyi.isIn_ = false;
        xyi_vec.emplace_back(xyi);
      }
      total_pixel_.emplace_back(xyi_vec);
      next_y = next_y + boxSize_;
    }
  }

  void QuadTree::printTotalPixel()
  {
    points_.colors.resize(0);
    points_.points.resize(0);

    std_msgs::ColorRGBA c;

    int id = 0;
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 1.0;
    
    for(size_t i = 0; i < total_pixel_.size(); ++i)
    {
      for(size_t k = 0; k < total_pixel_[i].size(); ++k)
      {
          if(total_pixel_[i][k].count == 0) continue;

          double x_gap = total_pixel_[int(point_pixel_x_ / 2)][int(point_pixel_y_ / 2)].x;
          double y_gap = total_pixel_[int(point_pixel_x_ / 2)][int(point_pixel_y_ / 2)].y;

          geometry_msgs::Point pt;
          pt.x = total_pixel_[i][k].x - x_gap;
          pt.y = total_pixel_[i][k].y - y_gap;
          pt.z = 0.0;

          points_.header.frame_id = "velodyne";
          points_.header.stamp = lidarTimeStamp_;
          points_.ns = "points";
          points_.action = visualization_msgs::Marker::ADD;
          points_.pose.orientation.w = 1.0;
          points_.lifetime = ros::Duration(0.1);
          points_.id = id++;
          points_.type = visualization_msgs::Marker::POINTS;
          points_.scale.x = 0.2;
          points_.scale.y = 0.2;

          points_.color.r = 0.0;
          points_.color.g = 0.0;
          points_.color.b = 1.0;
          points_.color.a = 1.0;
          points_.points.push_back(pt);
          points_.colors.push_back(c);
      }
    }
  }

  void QuadTree::initTotalBox()
  {
    totalBox_.boxes.resize(0);
    totalBox_.header.seq = 0;
    //totalBox_.header.stamp = ros::Time::now();
    totalBox_.header.frame_id = "velodyne";
  }

  void QuadTree::setParam()
  {
    height_from_ground_to_lidar_ = 2.1;
  }

  int QuadTree::divideQuadTree()
  {
    Box box = rect_;

    Box Rbox = box.getR();
    Box Lbox = box.getL();
    Box Ubox = box.getU();
    Box Dbox = box.getD();

    int urbox_pixelsize = box.getPixelSize();

    if(urbox_pixelsize < 1) return 0;

    haschildren_ = true;
    haspoints_ = false;

    children_[0] = new QuadTree(Rbox);
    children_[1] = new QuadTree(Lbox);
    children_[2] = new QuadTree(Ubox);
    children_[3] = new QuadTree(Dbox);

    if(checkChild(total_pixel_, Rbox)) children_[0]->divideQuadTree(); 
    delete children_[0];
    if(checkChild(total_pixel_, Lbox)) children_[1]->divideQuadTree(); 
    delete children_[1];
    if(checkChild(total_pixel_, Ubox)) children_[2]->divideQuadTree(); 
    delete children_[2];
    if(checkChild(total_pixel_, Dbox)) children_[3]->divideQuadTree(); 
    delete children_[3];

    return 1;
  }

  void QuadTree::setTimeStamp(const ros::Time& stamp)
  {
    lidarTimeStamp_ = stamp;
  }
  void QuadTree::divideQuadTree2()
  {
    double x_gap = total_pixel_[int(point_pixel_x_ / 2)][int(point_pixel_y_ / 2)].x;
    double y_gap = total_pixel_[int(point_pixel_x_ / 2)][int(point_pixel_y_ / 2)].y;

    for (size_t i = 0; i < total_pixel_.size(); ++i)
    {
      for (size_t k = 0; k < total_pixel_[i].size(); ++k)
      {
        if (total_pixel_[i][k].count == 0) continue;

        pcl::PointXYZ point;
        point.x = total_pixel_[i][k].x - x_gap;
        point.y = total_pixel_[i][k].y - y_gap;
        point.z = -0.3;

        octree_input_cloud.points.push_back(point);

        jsk_recognition_msgs::BoundingBox box_s;
        box_s.header.frame_id = "velodyne";
        box_s.header.seq = 0;
        box_s.header.stamp = lidarTimeStamp_;
        box_s.dimensions.x = boxSize_;
        box_s.dimensions.y = boxSize_;
        box_s.dimensions.z = total_pixel_[i][k].maxZ + height_from_ground_to_lidar_; //Try
        //box_s.dimensions.z = 0.0001;
        box_s.pose.position.x = point.x;
        box_s.pose.position.y = point.y;
        box_s.pose.position.z = (total_pixel_[i][k].maxZ + height_from_ground_to_lidar_) / 2 - height_from_ground_to_lidar_ ; // + (longest_z + 2.0) / 2;//Try
        //box_s.pose.position.z = -2.0; // + (longest_z + 2.0) / 2;//Try
        box_s.pose.orientation.x = 0.0;
        box_s.pose.orientation.y = 0.0;
        box_s.pose.orientation.z = 0.0;
        box_s.pose.orientation.w = 0.0;
        box_s.value = 1;
        box_s.label = 1;
        totalBox_.boxes.emplace_back(box_s);
        zarray_temp_.push_back(total_pixel_[i][k].maxZ);
      }
    }
    totalBox_.header.stamp = lidarTimeStamp_;
  }

  void QuadTree::makeGroup()
  {
    sensor_msgs::PointCloud2 points_group;
    pcl::PointCloud<pcl::PointXYZI> in_points;
    pcl::fromROSMsg(points_msg_, in_points);
    in_points.resize(0);
    
    int _label_;
    double longest_local_z;
    srand((unsigned)time(NULL));
    while(totalBox_.boxes.size() != 0)
    {
        longest_local_z = 0.0;
        _label_ = rand() % 255 + 1;
        grouping(0, _label_, in_points, longest_local_z);
        in_points.width = static_cast<uint32_t>(in_points.points.size());
        in_points.height = 1;
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_ptr(new pcl::PointCloud<pcl::PointXYZI>(in_points));
        pcl::toROSMsg(*in_ptr, points_group);
        in_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloudarray_.emplace_back(points_group);
        zarray_.emplace_back(longest_local_z);
        in_points.resize(0);
      }
   } 

  void QuadTree::makeGroup_Octree()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr octree_input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(octree_input_cloud));
    std::unordered_map<int, std::vector<int>> umap;
    std::vector<bool> used_check_vec;
    std::vector<int> parent;

    used_check_vec.assign(octree_input_cloud.size(), false);

    for(int i = 0; i < octree_input_cloud.points.size(); i++)
      parent.push_back(i);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(1.0);
    octree.setInputCloud(octree_input_cloud_ptr);
    octree.addPointsFromInputCloud();

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    for(size_t k = 0; k < octree_input_cloud.points.size(); ++k)
    {
      pcl::PointXYZ searchPoint;

      searchPoint = octree_input_cloud.points[k];
      used_check_vec[k] = true;
      
      double radius = 0.7;
      
      if(octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
          if(getParent(parent, k) == k) // don't have parent
          {
            if(getParent(parent, pointIdxRadiusSearch[i]) == pointIdxRadiusSearch[i]) // don't have parent
            {
              unionParent(parent, k, pointIdxRadiusSearch[i]);
            }
            else // have parent
            {
              unionParent(parent, k, getParent(parent, pointIdxRadiusSearch[i]));
              unionParent(parent, k, pointIdxRadiusSearch[i]);
            }
          }
          else  // have parent
          {
            unionParent(parent, k, getParent(parent, pointIdxRadiusSearch[i]));
            unionParent(parent, k, pointIdxRadiusSearch[i]);
          }
        }
      }
    }

    for(size_t i = 0; i < parent.size(); i++)
      umap[getParent(parent, i)].push_back(i);
  
    std::unordered_map<int, std::vector<int>>::iterator it;

    for(it = umap.begin(); it != umap.end(); it++)
    {
      sensor_msgs::PointCloud2 points_group;
      pcl::PointCloud<pcl::PointXYZI> in_points;
      
      pcl::PointXYZI point;

      double longest_local_z = -999; 

      for(size_t k = 0; k < it->second.size(); k++)
      {
        point.x = octree_input_cloud.points[it->second[k]].x;
        point.y = octree_input_cloud.points[it->second[k]].y;
        point.z = zarray_temp_[it->second[k]];
        point.intensity = 0.2;
        in_points.points.push_back(point);

        if(longest_local_z < point.z )
          longest_local_z = point.z;
      }

      in_points.width = static_cast<uint32_t>(in_points.points.size());
      in_points.height = 1;
      pcl::PointCloud<pcl::PointXYZI>::Ptr in_ptr(new pcl::PointCloud<pcl::PointXYZI>(in_points));
      pcl::toROSMsg(*in_ptr, points_group);
      in_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
      cloudarray_.emplace_back(points_group);
      zarray_.emplace_back(longest_local_z);
    }
    
    octree_input_cloud.points.resize(0);

    // detected boxes
    for(size_t i = 0; i < totalBox_.boxes.size(); ++i)
      clusteredBox_.boxes.emplace_back(totalBox_.boxes[i]);
  }

  double QuadTree::setRadius(pcl::PointXYZ searchPoint) // fix 20210103
  {
      double radius;
      
      double dist = sqrt(pow(searchPoint.x, 2) + pow(searchPoint.y, 2));

      if(dist <= 50)
        radius = 0.8;
      else if(dist <= 65)
        radius = 0.85;
      else if(dist <= 80)
        radius = 0.9;
      else
        radius = 1.0;

      return radius;
  }

  int QuadTree::getParent(std::vector<int> &parent, int x)
  {
    if(parent[x] == x) return x;
    return parent[x] = getParent(parent, parent[x]); 
  }

  void QuadTree::unionParent(std::vector<int> &parent, int a, int b)
  {
    a = getParent(parent, a);
    b = getParent(parent, b);

    if(a < b) parent[b] = a;
    else
      parent[a] = b;

    //parent[b] = a;
  }

  int QuadTree::findParent(std::vector<int> &parent, int a, int b)
  {
    a = getParent(parent, a);
    b = getParent(parent, b);
    if(a == b) return 1;
    else return 0;    
  }

  void QuadTree::grouping(int index, int box_label, pcl::PointCloud<pcl::PointXYZI>& in_points, double& longest_local_z)
  {
    jsk_recognition_msgs::BoundingBox tmp;
    tmp = totalBox_.boxes[index];
    totalBox_.boxes.erase(totalBox_.boxes.begin() + index);
    tmp.label = box_label;
    clusteredBox_.boxes.emplace_back(tmp);

    pcl::PointXYZI point;

    if (longest_local_z < tmp.dimensions.z)
      longest_local_z = tmp.dimensions.z;

    point.x = tmp.pose.position.x + (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y + (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    point.intensity = 0.2;
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x - (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y + (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    point.intensity = 0.2;
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x + (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y - (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    point.intensity = 0.2;
    in_points.points.emplace_back(point);

    point.x = tmp.pose.position.x - (tmp.dimensions.x / 2);
    point.y = tmp.pose.position.y - (tmp.dimensions.y / 2);
    point.z = tmp.pose.position.z - (tmp.dimensions.z / 2);
    point.intensity = 0.2;
    in_points.points.emplace_back(point);

    for(size_t i = 0; i < totalBox_.boxes.size(); ++i){
      if(checkDist(tmp, totalBox_.boxes[i])){
        grouping(i, box_label, in_points, longest_local_z);
          i = 0;
      }
    }
  }

  bool QuadTree::checkChild(std::vector< std::vector<XYI> >& in_pixel, Box& in_box)
  {

    int pixelCnt = 0;
    int start_X, start_Y;
    double longest_z = (-1) * height_from_ground_to_lidar_;
    double result_Z = 0, total_Z =0;

    double in_box_Xpose = in_box.getXPose();
    double in_box_Ypose = in_box.getYPose();
    double in_box_pixelsize = in_box.getPixelSize();
    double in_box_Size = in_box.getSize();
    
    start_Y = start_Yindex_ - (in_box_Ypose / boxSize_);
    start_X = start_Xindex_ + (in_box_Xpose / boxSize_);
    start_Y = start_Y - (in_box_pixelsize / 2) + 1;
    start_X = start_X - (in_box_pixelsize / 2);
    
    if(in_pixel[start_X][start_Y].isIn_ == true)
      return false;

    for(int i = start_Y; i < start_Y + in_box_pixelsize; ++i)
    {
      for(int j = start_X; j < start_X + in_box_pixelsize; ++j)
      {
        if(in_pixel[i][j].maxZ > longest_z) 
          longest_z = in_pixel[i][j].maxZ;

        pixelCnt += in_pixel[i][j].count;
        total_Z += in_pixel[i][j].z;
      }
    }
    result_Z = total_Z / pixelCnt + (longest_z + height_from_ground_to_lidar_) / 2;
    /* if point exists in box which is minimum size box, bounding box is maked */
    if (pixelCnt == 0) 
    {
      return false;
    }
    else if(checkThreshold(in_box, pixelCnt)) 
    {
      return true;
    }
    else if(!checkThreshold(in_box, pixelCnt))
    {
      in_pixel[start_X][start_Y].isIn_ = true;
      jsk_recognition_msgs::BoundingBox box_s;
      box_s.header.frame_id = "velodyne";
      box_s.header.seq = 0;
      box_s.header.stamp = lidarTimeStamp_;
      box_s.dimensions.x = in_box_Size;
      box_s.dimensions.y = in_box_Size;
      box_s.dimensions.z = longest_z + height_from_ground_to_lidar_;//Try
      //box_s.dimensions.z = 0.0001;
      box_s.pose.position.x = in_box_Xpose;
      box_s.pose.position.y = in_box_Ypose;
      box_s.pose.position.z = result_Z;// + (longest_z + 2.0) / 2;//Try
      //box_s.pose.position.z = -2.0;// + (longest_z + 2.0) / 2;//Try
      box_s.pose.orientation.x = 0.0;
      box_s.pose.orientation.y = 0.0;
      box_s.pose.orientation.z = 0.0;
      box_s.pose.orientation.w = 0.0;
      box_s.value = 1;
      box_s.label = 1;
      totalBox_.boxes.emplace_back(box_s);
      return false;
    }
    else{
        //ROS_ERROR("check children error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
        exit(1);
    }
    
  }

  bool QuadTree::checkDist(jsk_recognition_msgs::BoundingBox& in_box1, jsk_recognition_msgs::BoundingBox& in_box2)
  {
    double box1_size = in_box1.dimensions.x;    
    double box2_size = in_box2.dimensions.x;
    double box1_dist, box2_dist, box1_to_box2;
    double box1_Xpose, box1_Ypose, box2_Xpose, box2_Ypose;

    box1_Xpose = in_box1.pose.position.x;
    box1_Ypose = in_box1.pose.position.y;
    box2_Xpose = in_box2.pose.position.x;
    box2_Ypose = in_box2.pose.position.y;

    box1_dist = sqrt(pow(box1_size / 2, 2) + pow(box1_size / 2, 2));
    box2_dist = sqrt(pow(box2_size / 2, 2) + pow(box2_size / 2, 2));

    box1_to_box2 = sqrt(pow(box1_Xpose - box2_Xpose, 2) + pow(box1_Ypose - box2_Ypose, 2));

    if((box1_dist + box2_dist) + box_threshold_ >= box1_to_box2) return true; 
    else return false;
  }

  bool QuadTree::checkThreshold(Box& in_box, int in_Cnt)
  {
    int in_box_pixelsize = in_box.getPixelSize();
  #if DETECT_LEVEL_25M
    if(LEVEL0_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL0_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL1_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL1_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL2_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL2_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL3_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL3_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL4_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL4_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL5_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL5_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL6_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL6_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL7_BOX_PIXEL == in_box_pixelsize){
      if(LEVEL7_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else{
      //ROS_ERROR("check_threshold error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
      exit(1);
    }
#endif

  #if DETECT_LEVEL_50M
    if(LEVELX_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVELX_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL0_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL0_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL1_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL1_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL2_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL2_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL3_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL3_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL4_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL4_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL5_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL5_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL6_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL6_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else if(LEVEL7_BOX_PIXEL == in_box_pixelsize)
    {
      if(LEVEL7_THRESHOLD > in_Cnt) return true;
      else return false;
    }
    else
    {
      //ROS_ERROR("check_threshold error! (FUNC %s)(LINE %ld)", __FUNCTION__, __LINE__);
      exit(1);
    }
  #endif
  }

  void QuadTree::reset()
  {
    total_pixel_.resize(0);
    
    clusteredBox_.boxes.resize(0);
    totalBox_.boxes.resize(0);
    totalBox_.header.seq = seq++;
    //totalBox_.header.stamp = ros::Time::now();
    totalBox_.header.frame_id = "velodyne";
    cloudarray_.resize(0);
    zarray_.resize(0);
    zarray_temp_.resize(0);
  }

  std::vector<sensor_msgs::PointCloud2> QuadTree::getCloudArray()
  {
    return cloudarray_;
  }
  
  std::vector<double> QuadTree::getZArray()
  {
    return zarray_;
  }

  size_t QuadTree::getClusteredBoxSize()
  {
    return clusteredBox_.boxes.size();
  }

  size_t QuadTree::getCloudarraySize()
  {
    return cloudarray_.size();
  }

  visualization_msgs::Marker QuadTree::getPoints()
  {
    return points_;
  }

  Box::Box(double xpose, double ypose, double bsize, int pixel)
  {
    Xpose_ = xpose;
    Ypose_ = ypose;
    size_ = bsize;
    pixelSize_ = pixel;
  }

  Box::Box()
  {
    Xpose_ = 0.0;
    Ypose_ = 0.0;
    size_ = 100.0;
    pixelSize_ = 513;
  }

  Box::~Box()
  {

  }
  
  Box Box::getUR()
  {
      double in_x = this->Xpose_ + this->size_ / 4.0;
      double in_y = this->Ypose_ + this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getUL()
  {
      double in_x = this->Xpose_ - this->size_ / 4.0;
      double in_y = this->Ypose_ + this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getDR()
  {
      double in_x = this->Xpose_ + this->size_ / 4.0;
      double in_y = this->Ypose_ - this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getDL()
  {
      double in_x = this->Xpose_ - this->size_ / 4.0;
      double in_y = this->Ypose_ - this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  // jaemin
  Box Box::getR()
  {
      double in_x = this->Xpose_ + this->size_ / 4.0;
      double in_y = this->Ypose_;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getL()
  {
      double in_x = this->Xpose_ - this->size_ / 4.0;
      double in_y = this->Ypose_;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getU()
  {
      double in_x = this->Xpose_;
      double in_y = this->Ypose_ + this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }

  Box Box::getD()
  {
      double in_x = this->Xpose_;
      double in_y = this->Ypose_ - this->size_ / 4.0;
      double in_size = this->size_ / 2.0;
      Box box(in_x, in_y, in_size, pixelSize_ / 2);
      return box;
  }
  /////////////////////////////////////////////////

  int Box::getPixelSize()
  {
    return pixelSize_;
  }

  double Box::getXPose()
  {
    return Xpose_;
  }
  
  double Box::getYPose()
  {
    return Ypose_;
  }

  double Box::getSize()
  {
    return size_;
  }

  
}
