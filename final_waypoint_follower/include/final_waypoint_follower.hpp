#include <ros/ros.h>
#include <iostream>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/Lane.h>
#include <pcl_ros/point_cloud.h>
#include <unordered_map>
#include <sensor_msgs/PointCloud2.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
#include <ctime>
#include <pcl/segmentation/segment_differences.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/registration/icp.h>
#include <memory>
#include <pcl/search/organized.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree_search.h>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include "opencv2/opencv.hpp"
// #include <pcl/segmentation/sac_segmentation.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include "ackermann_msgs/AckermannDriveStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <fstream>

namespace finalway
{
  class FinalWaypointFollower
  {
    public:
      FinalWaypointFollower();
      ~FinalWaypointFollower();
      void callbackFromWayPoints(const geometry_msgs::Point &msg);
      void callbackBackDriving(const std_msgs::BoolConstPtr& msg);
      void callbackParkingMissionEnd(const std_msgs::BoolConstPtr& msg);
      double l2norm(double x, double y);

    private:
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      ros::Subscriber sub_final_waypoint_1;
      ros::Subscriber sub_backdriving;
      ros::Subscriber sub_parking_mission_end;
      ros::Subscriber sub_distance;

      // ros::Publisher pub1_, pub2_;

      ros::Publisher ackermann_publisher;
	    ros::Publisher degree_publisher;
  
      geometry_msgs::TwistStamped ts;
      std_msgs::Float64 msg_degree;

      visualization_msgs::Marker points_;

      double velocity_;
      double prev_z_;

      bool backdriving_;
      bool parking_mission_end_;

      float dist_;
  };
}