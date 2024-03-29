#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"  
#include "vector_map_msgs/PointArray.h"
#include <geometry_msgs/TwistStamped.h>
#include <thread>
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

constexpr double EPSILON = 0.00001;
class GoalInitializer{
public:
    GoalInitializer(){
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        vector_map_sub = nh.subscribe("/vector_map_info/point", 1000, &GoalInitializer::vectormap_cb, this);
        twist_sub = nh.subscribe("twist_cmd", 1000, &GoalInitializer::twist_cb, this);
    }

    void vectormap_cb(const vector_map_msgs::PointArrayConstPtr& ptr){
        auto last_point = *(ptr->data.end() - 1);
        // std::cout << "last_point: " << last_point << std::endl; //0726 추가 
        auto last_prev_point = *(ptr->data.end() - 2);

        for(int i = 0; i < ptr->data.size(); i++){
            last_point = *(ptr->data.end() - 1 -i);
            last_prev_point = *(ptr->data.end() - 3 - i);
            if(last_point.b == 0 && last_point.l == 0 && last_point.mcode1 == 0 && last_point.mcode2 == 0 && last_point.mcode3 == 0){
                break;
            }
        }
        ROS_INFO("size : %lu", ptr->data.size());
        ROS_INFO("last_point ly, bx : %lf %lf", last_point.ly, last_point.bx);
        ROS_INFO("last_prev_point ly, bx : %lf %lf", last_prev_point.ly, last_prev_point.bx);
        
        //set goalpoint poisition
        Gx = last_point.ly;
        Gy = last_point.bx;

        //calc goal point orientation
        double dx = last_point.ly - last_prev_point.ly;
        double dy = last_point.bx - last_prev_point.bx;
        double yaw = std::atan2(dy, dx);
        ROS_INFO("dx, dy, yaw : %lf %lf %lf", dx, dy, yaw);


        tf::Quaternion quat_tf = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(quat_tf, msg.pose.orientation);

        msg.pose.position.x = Gx;
        msg.pose.position.y = Gy;
        msg.pose.position.z = 0.0;

    	is_vectormap_subscribed = true;

	    std::thread(&GoalInitializer::run, this).detach();
    }

    void twist_cb(const geometry_msgs::TwistStampedConstPtr& ptr){
        if (std::fabs(ptr->twist.linear.x) < EPSILON) return;
        is_twist_subscribed = true;
        ROS_INFO("twist is subscribed");
    
    }

    void run(){
		ros::Rate loop_rate(1);
        while(ros::ok()){
            msg.header.stamp= ros::Time::now();
            msg.header.frame_id = "world";

            goal_pub.publish(msg);  // 0906주석해제
            if (is_twist_subscribed) break;
        }
        loop_rate.sleep();
        ros::shutdown();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub;
    ros::Subscriber vector_map_sub, twist_sub;
    geometry_msgs::PoseStamped msg;
	bool is_vectormap_subscribed, is_twist_subscribed;
    float Gx, Gy;
};

int main( int argc, char **argv )
{
    geometry_msgs::PoseStamped msg;
    ros::init( argc, argv, "goal_initial" );
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    
    GoalInitializer g;

	ros::spin();

    return 0;
}
