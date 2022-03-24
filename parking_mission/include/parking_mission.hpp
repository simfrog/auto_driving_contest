#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float32.h>

namespace parking_mission
{
    class ParkingMission
    {
        public:
            ParkingMission();
            ~ParkingMission();

            std::vector<std::vector<geometry_msgs::Point>> loadParkinglotPoints();
            std::vector<std::vector<geometry_msgs::Point>> filteringParkingArea();
            void callbackParkingMission(const std_msgs::BoolConstPtr& ptr);
            void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
            void callbackGetLidarObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg);
            void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
            geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform);
            void printPoint(geometry_msgs::Point &point, double r, double g, double b);
            double getDistance(geometry_msgs::Point &point1, geometry_msgs::Point &point2);
            bool checkingEmptyArea(const autoware_msgs::DetectedObjectArrayConstPtr &lidar_objects, std::vector<geometry_msgs::Point>& candidateArea);
            void makeCheckingAreaPolygons(std::vector<geometry_msgs::Point>& candidateArea);
            bool checkPointPolygonTest(std::vector<cv::Point2f>& object_convexhull, std::vector<cv::Point2f>& target_convexhull);

        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            ros::Publisher goal_pub_;
            ros::Publisher goal_rviz_pub_;
            ros::Publisher backdriving_pub_;
            ros::Publisher parking_mission_end_pub_;
            ros::Publisher checking_area_polygons_pub_;
            ros::Publisher estop_pub_;
            ros::Publisher distance_pub_;

            ros::Subscriber mission_call_sub_;
            ros::Subscriber current_pose_sub_;
            ros::Subscriber lidar_objects_sub_;

            geometry_msgs::Point goal_;
            geometry_msgs::Point current_pose_;

            std::vector<std::vector<geometry_msgs::Point>> parkinglot_points_;
            
            bool start_;
            bool checking1;
            bool in_mission_clear_;
            bool get_latest_objects_;
            int one_parking_area_point_num_;
            int index_in_;
            int index_out_;
            bool checking2_start;
            bool checking_endpoint_;
            bool isEmptyArea;
            bool checkEmpty;
            bool waiting_mission_clear_;
            bool first_time_;
            
            std::string parkinglotpoints_file_name_;
            std::vector<geometry_msgs::Point> candidateArea;
            double checkpoint_reach_threshold_;
            visualization_msgs::Marker rviz_points_;

            ros::Time waiting_start_time_;
    };  
}
 
