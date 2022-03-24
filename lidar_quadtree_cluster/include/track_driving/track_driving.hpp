#include <ros/ros.h>
#include <iostream>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <pcl/point_types.h>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace track
{
  class TrackDriving
  {
    public:
      TrackDriving();
      ~TrackDriving();
      void callbackGetLidarObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg); // lidar_objects
      void callbackCarVelocity(const geometry_msgs::TwistStampedConstPtr& ptr);

      void objectsClusteringJaemin(std::vector<pcl::PointXYZ>& objects_center_vec, std::vector<pcl::PointXYZ>& all_objects);

      geometry_msgs::Point findNearestPoint(std::vector<pcl::PointXYZ>& vec, geometry_msgs::Point start_point, int line_num, std::vector<pcl::PointXYZ>& all_objects);

      void printPoint(geometry_msgs::Point &point, double r, double g, double b);

      std::vector<geometry_msgs::Point> calcCenterPoint(std::vector<geometry_msgs::Point> &first_inliers, std::vector<geometry_msgs::Point> &second_inliers);
      
      std::vector<geometry_msgs::Point> getInliers(std::vector<double> &line, pcl::PointCloud<pcl::PointXYZ> &point_cloud);

      geometry_msgs::Point findNearestXmPoint(std::vector<geometry_msgs::Point>& center_points);
      std::pair<int, int> getNeighborPointsNum(geometry_msgs::Point &point, std::vector<pcl::PointXYZ>& all_objects);
      double getDistance(geometry_msgs::Point &point1, pcl::PointXYZ &point2);

      void pubTF();

      geometry_msgs::Point savePrevObject(geometry_msgs::Point& pt);
      geometry_msgs::Point loadPrevObject(geometry_msgs::Point& pt);

      void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
      pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

    private:
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      ros::Subscriber sub_objects_;
      ros::Subscriber sub_car_info_;

      ros::Publisher pub_center_points_;
      ros::Publisher pub_final_waypoint_rviz;
      ros::Publisher pub_final_waypoint_;
      ros::Publisher odomPub_;

      double x_limit_;
      double y_limit_;
      double neighbor_distance_;

      visualization_msgs::Marker points_, rviz_points_;

      geometry_msgs::Point prev_left_object_, prev_right_object_;

      std::pair<geometry_msgs::Point, geometry_msgs::Point> prev_objects_map_;
  };
}