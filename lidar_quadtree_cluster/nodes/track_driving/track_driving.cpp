#include <track_driving/track_driving.hpp>
#include <algorithm>
#include <vector>

//trackdriving 0828 version3 = >0905 fixing
bool Compare(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
{
  if(pt1.x < pt2.x)
    return true;
  else 
    return false;
}
//std::vector<pcl::PointXYZ>

bool compare(pcl::PointXYZ pt1, pcl::PointXYZ pt2){
  if(pt1.x <pt2.x) return true;
  else return false;
} 

bool compare_dist(std::pair<geometry_msgs::Point, double> p1, std::pair<geometry_msgs::Point, double> p2)
{
  if(p1.second < p2.second) return true;
  else return false;
}

namespace track
{
  TrackDriving::TrackDriving() : private_nh_("~")
  {
    if(!private_nh_.getParam("x_limit", x_limit_)) throw std::runtime_error("fail to get x_limit");
    if(!private_nh_.getParam("y_limit", y_limit_)) throw std::runtime_error("fail to get y_limit");
    if(!private_nh_.getParam("neighbor_distance", neighbor_distance_)) throw std::runtime_error("fail to get neighbor_distance");

    sub_objects_ = nh_.subscribe("/detection/lidar_detector/objects", 1, &TrackDriving::callbackGetLidarObjects, this);

    pub_center_points_ = nh_.advertise<visualization_msgs::Marker>("/center_points", 1);
    pub_final_waypoint_rviz = nh_.advertise<visualization_msgs::Marker>("/final_waypoint_rviz", 1);
    pub_final_waypoint_ = nh_.advertise<geometry_msgs::Point>("/trackdriving_final_waypoint", 1);

    prev_left_object_.x = 1.0;
    prev_left_object_.y = 1.0;
    prev_left_object_.z = 0.0;

    prev_right_object_.x = 1.0;
    prev_right_object_.y = -1.0;
    prev_right_object_.z = 0.0;

    prev_objects_map_ = std::make_pair(savePrevObject(prev_left_object_), savePrevObject(prev_right_object_));
  }

  TrackDriving::~TrackDriving()
  {

  }
  
  void TrackDriving::callbackGetLidarObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg) // lidar_objects
  {
    // get center point
    std::vector<pcl::PointXYZ> objects_center_vec;
    std::vector<pcl::PointXYZ> all_objects;

    for(size_t i = 0; i < msg->objects.size(); ++i)
    {
      pcl::PointXYZ pt;
      pt.x = msg->objects[i].pose.position.x;
      pt.y = msg->objects[i].pose.position.y;
      pt.z = 0.0;

      all_objects.push_back(pt);
      if(pt.x < x_limit_ && pt.x > 0.0 && fabs(pt.y) < 3.0) 
        objects_center_vec.push_back(pt);
    }

    objectsClusteringJaemin(objects_center_vec, all_objects);
  }

  void TrackDriving::objectsClusteringJaemin(std::vector<pcl::PointXYZ>& objects_center_vec, std::vector<pcl::PointXYZ>& all_objects)
  {
    geometry_msgs::Point left_start_point;
    left_start_point.x = 0.0;
    left_start_point.y = 1.0;
    left_start_point.z = 0.0;

    geometry_msgs::Point right_start_point;
    right_start_point.x = 0.0;
    right_start_point.y = -1.0;
    right_start_point.z = 0.0;

    geometry_msgs::Point left_nearest_point = findNearestPoint(objects_center_vec, left_start_point, 0, all_objects);
    geometry_msgs::Point right_nearest_point = findNearestPoint(objects_center_vec, right_start_point, 1, all_objects);

    geometry_msgs::Point goal_point;
    goal_point.x = (left_nearest_point.x + right_nearest_point.x)/2.0;
    goal_point.y = (left_nearest_point.y + right_nearest_point.y)/2.0;
    goal_point.z = (left_nearest_point.z + right_nearest_point.z)/2.0;

    rviz_points_.points.resize(0);
    rviz_points_.colors.resize(0);

    printPoint(left_nearest_point, 0.0, 0.0, 1.0);
    printPoint(right_nearest_point, 0.0, 1.0, 0.0);
    printPoint(goal_point, 1.0, 0.0, 0.0);

    geometry_msgs::Point x_limit_pt;
    x_limit_pt.x = x_limit_;
    x_limit_pt.y = 1.0;
    x_limit_pt.z = 0.0;
    printPoint(x_limit_pt, 0.5, 0.5, 0.5);
    x_limit_pt.y = -1.0;
    printPoint(x_limit_pt, 0.5, 0.5, 0.5);
    geometry_msgs::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    printPoint(origin, 1.0, 1.0, 1.0);
    geometry_msgs::Point y_limit_pt;
    y_limit_pt.x = 0;
    y_limit_pt.y = 1.0;
    y_limit_pt.z = 0.0;
    printPoint(y_limit_pt, 0.5, 0.5, 0.5);
    y_limit_pt.y = -1.0;
    printPoint(y_limit_pt, 0.5, 0.5, 0.5);

    for(double i = 0.0; i < x_limit_; i+=0.3)
    {
      geometry_msgs::Point pt;
      pt.x = i;
      pt.y = 0.0;
      pt.z = 0.0;
      printPoint(pt, 0.7, 0.7, 0.0);
    }

    pub_center_points_.publish(rviz_points_);
    pub_final_waypoint_.publish(goal_point);
  }

  geometry_msgs::Point TrackDriving::findNearestPoint(std::vector<pcl::PointXYZ>& vec, geometry_msgs::Point start_point, int line_num, std::vector<pcl::PointXYZ>& all_objects)
  {
    geometry_msgs::Point pt;
    geometry_msgs::Point object_pt;

    pt = start_point;

    std::vector<std::pair<geometry_msgs::Point, double>> list;
    std::vector<std::pair<geometry_msgs::Point, double>> critical_list;

    // 0.5 <= x < 3.0, left : 0.5 < y < 3.0 , right : -3.0 < y < -0.5
    for(size_t k = 0; k < vec.size(); ++k)
    {
      if(vec[k].x < 1.7 && vec[k].x >= 1.2)
      {
        if(line_num == 0 && vec[k].y < 0.5) continue;
        if(line_num == 1 && vec[k].y > -0.5) continue;
        
        object_pt.x = vec[k].x;
        object_pt.y = vec[k].y;
        object_pt.z = vec[k].z;
        // list.push_back(std::make_pair(object_pt, fabs(vec[k].y - pt.y)));
        list.push_back(std::make_pair(object_pt, fabs(std::sqrt(std::pow(vec[k].x, 2) + std::pow(vec[k].y - pt.y, 2)))));
      }
    }
    
    
    for(size_t k = 0; k < vec.size(); ++k)
    {
      // 3.0 <= x < 6.0, -3.0 < y < 3.0 
      if(vec[k].x < x_limit_ && vec[k].x >= 1.7)
      {    
        object_pt.x = vec[k].x;
        object_pt.y = vec[k].y;
        object_pt.z = vec[k].z;
        // list.push_back(std::make_pair(object_pt, fabs(vec[k].y - pt.y)));
        critical_list.push_back(std::make_pair(object_pt, fabs(std::sqrt(std::pow(vec[k].x, 2) + std::pow(vec[k].y - pt.y, 2)))));
      }

      // 0.5 <= x < 3.0, -0.5 <= y <= 0.5 
      if(vec[k].x >= 0.5 && vec[k].x < 1.7 && fabs(vec[k].y) <= 0.7)
      {    
        object_pt.x = vec[k].x;
        object_pt.y = vec[k].y;
        object_pt.z = vec[k].z;
        // list.push_back(std::make_pair(object_pt, fabs(vec[k].y - pt.y)));
        critical_list.push_back(std::make_pair(object_pt, fabs(std::sqrt(std::pow(vec[k].x, 2) + std::pow(vec[k].y - pt.y, 2)))));
      }
    }

    std::cout << "critical_list size : " << critical_list.size() << std::endl;
    sort(list.begin(), list.end(), compare_dist);
    sort(critical_list.begin(), critical_list.end(), compare_dist);

    if(list.size() == 0)
    {
      std::cout << critical_list.size() << std::endl;
      if(critical_list.size() != 0)
      {
        std::pair<int, int> neighbor_num = getNeighborPointsNum(critical_list[0].first, all_objects);
        std::cout << neighbor_num.first << " , " << neighbor_num.second << std::endl;
        if(line_num == 0)
        {
          std::cout <<"left detection" << std::endl;
          if(neighbor_num.first < neighbor_num.second) // first : left, second : right
          {
            std::cout << "left processing" << std::endl;
            object_pt = critical_list[0].first;
            prev_left_object_ = object_pt;
            prev_objects_map_.first = savePrevObject(prev_left_object_);
          }
          else
          {
            prev_left_object_ = loadPrevObject(prev_objects_map_.first);
            return prev_left_object_;
          }
        }
        if(line_num == 1)
        {
          std::cout << "right detection" << std::endl;
          if(neighbor_num.first > neighbor_num.second) 
          {
            std::cout << "right processing" << std::endl;
            object_pt = critical_list[0].first;
            prev_right_object_ = object_pt;
            prev_objects_map_.second = savePrevObject(prev_right_object_);
          }
          else
          {
            prev_right_object_ = loadPrevObject(prev_objects_map_.second);
            return prev_right_object_;
          }
            
        }
      }
      else
      {
        if(line_num == 0) 
        {
          prev_left_object_ = loadPrevObject(prev_objects_map_.first);
          return prev_left_object_;
        }
        if(line_num == 1)
        {
          prev_right_object_ = loadPrevObject(prev_objects_map_.second);
          return prev_right_object_;
        }
      }
    }
    else
    {
      object_pt = list[0].first;
      if(line_num == 0) 
      {
        prev_left_object_ = object_pt;
        prev_objects_map_.first = savePrevObject(prev_left_object_);
      }
      if(line_num == 1)
      {
        prev_right_object_ = object_pt;
        prev_objects_map_.second = savePrevObject(prev_right_object_);
      }
    }
    
    return object_pt;
  }

  std::pair<int, int> TrackDriving::getNeighborPointsNum(geometry_msgs::Point &point, std::vector<pcl::PointXYZ>& all_objects)
  {
    std::pair<int, int> p;
    
    int left = 0;
    int right = 0;

    for(size_t i = 0; i < all_objects.size(); ++i)
    {
      if(getDistance(point, all_objects[i]) < neighbor_distance_)
      {
        if(all_objects[i].y < point.y) right++;
        if(all_objects[i].y > point.y) left++;
      }
    }
    p = std::make_pair(left, right);
    return p;
  }

  double TrackDriving::getDistance(geometry_msgs::Point &point1, pcl::PointXYZ &point2)
  {
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
  }

  void TrackDriving::printPoint(geometry_msgs::Point &point, double r, double g, double b)
  {
    visualization_msgs::Marker marker_point;

    std_msgs::ColorRGBA c;
    int id = 0;

    c.r = r;
    c.g = g;
    c.b = b;
    c.a = 1.0;

    rviz_points_.header.frame_id = "velodyne";
    rviz_points_.header.stamp = ros::Time::now();
    rviz_points_.ns = "points";
    rviz_points_.action = visualization_msgs::Marker::ADD;
    rviz_points_.pose.orientation.w = 1.0;
    rviz_points_.lifetime = ros::Duration(0.1);
    rviz_points_.id = id++;
    rviz_points_.type = visualization_msgs::Marker::POINTS;
    rviz_points_.scale.x = 0.2;
    rviz_points_.scale.y = 0.2;

    rviz_points_.color.r = 0.0;
    rviz_points_.color.g = 0.0;
    rviz_points_.color.b = 1.0;
    rviz_points_.color.a = 1.0;

    rviz_points_.points.push_back(point);
    rviz_points_.colors.push_back(c);

  }

  geometry_msgs::Point TrackDriving::savePrevObject(geometry_msgs::Point& pt)
  {
    std::cout << "savePrevObject!!" << std::endl;

    tf::StampedTransform transform;
	  GetTransformFromTF("/my_map", "/velodyne", transform); 

    pcl::PointXYZ input; 
    input.x = pt.x;
    input.y = pt.y;
    input.z = pt.z;

    pcl::PointXYZ result_pcl = TransformPoint(input, transform);

    geometry_msgs::Point result;
    result.x = result_pcl.x;
    result.y = result_pcl.y;
    result.z = result_pcl.z;

    return result;
  }

  geometry_msgs::Point TrackDriving::loadPrevObject(geometry_msgs::Point& pt)
  {
    std::cout << "loadPrevObject!!" << std::endl;
    tf::StampedTransform transform;
	  GetTransformFromTF("/velodyne", "/my_map", transform);

    pcl::PointXYZ input;
    input.x = pt.x;
    input.y = pt.y;
    input.z = pt.z;

    pcl::PointXYZ result_pcl = TransformPoint(input, transform);

    geometry_msgs::Point result;
    result.x = result_pcl.x;
    result.y = result_pcl.y;
    result.z = result_pcl.z;

    return result;
  }

  void TrackDriving::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
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

  pcl::PointXYZ TrackDriving::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
  {
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "TrackDriving");

  track::TrackDriving TD;

  ros::spin();
}