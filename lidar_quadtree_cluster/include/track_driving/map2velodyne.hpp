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

namespace tf_publisher
{
  class Map2Velodyne
  {
    public:
      Map2Velodyne();
      ~Map2Velodyne();

      void pubTF();
      void callbackCarVelocity(const geometry_msgs::TwistStampedConstPtr& ptr);

    private:
      ros::NodeHandle nh_;
    
      ros::Subscriber sub_car_info_;
      
      // TF
      ros::Time last;
      tf::TransformBroadcaster br;

      double x_dot, y_dot, theta_dot;
      int seq;
      double x, y, theta;
      double velocity;
      double yaw;
      geometry_msgs::TwistStamped car_info_;

      Eigen::Vector2d car_pos_world;  //world frame. 
        double car_yaw_world; //world frame 
  };
}