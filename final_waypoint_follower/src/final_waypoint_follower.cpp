#include <final_waypoint_follower.hpp>
#include <algorithm>
#include <vector>

namespace finalway
{
  FinalWaypointFollower::FinalWaypointFollower() : private_nh_("~"), backdriving_(false), parking_mission_end_(false)
  {
    sub_final_waypoint_1 = nh_.subscribe("/trackdriving_final_waypoint", 1, &FinalWaypointFollower::callbackFromWayPoints, this);
    sub_backdriving = nh_.subscribe("/backdriving", 1, &FinalWaypointFollower::callbackBackDriving, this);
    sub_parking_mission_end = nh_.subscribe("/parking_mission_end",1, &FinalWaypointFollower::callbackParkingMissionEnd, this);
    if (!private_nh_.getParam("velocity", velocity_))    throw std::runtime_error("set velocity!");

    // pub1_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 10);
    // pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>(out_ctrl_cmd, 10);
    ackermann_publisher = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmd_erp42",100);
	  // degree_publisher = nh_.advertise<std_msgs::Float64>("degree_cmd",100);

    srand((unsigned)time(NULL));

    prev_z_ = 0.0;
  }

  FinalWaypointFollower::~FinalWaypointFollower()
  {

  }

  void FinalWaypointFollower::callbackParkingMissionEnd(const std_msgs::BoolConstPtr& msg)
  {
    std::cout << "final waypoint follower subscribe [Parking Mission End]" << std::endl;
    geometry_msgs::TwistStamped t;
    t.header.frame_id = "body";
    t.header.stamp = ros::Time::now();
    parking_mission_end_ = true;
    t.twist.linear.x = 0.0;
    t.twist.angular.z = 0.0;
    ackermann_publisher.publish(t);
    ros::shutdown();
  }

  void FinalWaypointFollower::callbackBackDriving(const std_msgs::BoolConstPtr& msg)
  {
    std::cout << "waypoint follower - backdriving" << std::endl;
    backdriving_ = msg->data;
  }
  
  void FinalWaypointFollower::callbackFromWayPoints(const geometry_msgs::Point& msg)
  {
    geometry_msgs::TwistStamped t;
    t.header.frame_id = "body";
    t.header.stamp = ros::Time::now();

    if(msg.x != 0 || msg.y != 0)
    {
      double r = l2norm(msg.x, msg.y) / (2*msg.y);
      double w = velocity_ / r;
        
      t.twist.linear.x = velocity_;
      t.twist.angular.z = w;
    }
    else
    {
      t.twist.linear.x = 0.2;
      t.twist.angular.z = 0.0;
    }

    //velocity filtering
    if(fabs(t.twist.angular.z * 180.0 / 3.141592) < 10)
      t.twist.linear.x = t.twist.linear.x;
    else if(fabs(t.twist.angular.z * 180.0 / 3.141592) < 20)
      t.twist.linear.x = t.twist.linear.x / 4 * 3;
    else if(fabs(t.twist.angular.z * 180.0 / 3.141592) < 30)
      t.twist.linear.x = t.twist.linear.x / 4 * 2;
    else if(fabs(t.twist.angular.z * 180.0 / 3.141592) < 40)
      t.twist.linear.x = t.twist.linear.x / 4 * 1;
    
    if(backdriving_ == true)
    {
      t.twist.linear.x = (-1) * t.twist.linear.x;
      t.twist.angular.z = (-1) * t.twist.angular.z;
    }

    // std::cout << "twist linear  x : " << t.twist.linear.x << std::endl;
    // std::cout << "twist angular z : " << t.twist.angular.z << " (radian) , " << t.twist.angular.z * 180.0 / 3.141592 << " (degree) " << std::endl;
    // std::cout << "==============================================" << std::endl;

    ackermann_publisher.publish(t);
  }

  double FinalWaypointFollower::l2norm(double x, double y){
        return std::sqrt(x*x + y*y);
    }

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "FinalWaypointFollower");

  finalway::FinalWaypointFollower FWF;
  
  ros::spin();
  return 0;
}