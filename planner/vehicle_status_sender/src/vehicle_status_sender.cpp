#include <ros/ros.h>
#include <autoware_msgs/VehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <erp42_msgs/erp42_to_pc.h>
#include <cmath>


constexpr double WHEELBASE = 1.12;
constexpr double EPSILON = 2; //mps
class VehicleStatusSender{
public:
    VehicleStatusSender(){
        state_pub = nh.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 1000);
        imu_sub = nh.subscribe("imu", 1000, &VehicleStatusSender::imu_cb, this);
        erp42_sub = nh.subscribe("erp42_to_pc", 1000, &VehicleStatusSender::erp42_cb, this);
    }

    void imu_cb(const sensor_msgs::ImuConstPtr& ptr){
        w_cur = ptr->angular_velocity.z;

        //calc current steering angle
        double steer_cur;
        if (std::fabs(v_cur) > EPSILON){
            double curvature = v_cur / w_cur;
		    steer_cur = std::atan(WHEELBASE / curvature); // radian
        } else steer_cur = steer_erp42; 


        //publish autoware vehcle status msg
        autoware_msgs::VehicleStatus vehicle_status_msg;
        vehicle_status_msg.header.stamp = ros::Time::now();

        vehicle_status_msg.angle = steer_cur;
        vehicle_status_msg.speed = v_cur * 3.6; //mps->kph
        state_pub.publish(vehicle_status_msg);

        ROS_INFO("v, w, steer : %lf %lf %lf", v_cur, w_cur, steer_cur * 180 / M_PI);
    }

    void erp42_cb(const erp42_msgs::erp42_to_pcConstPtr& ptr){
        v_cur = ptr->speed; //mps
        if (ptr->isBackward) v_cur *= -1;
        steer_erp42 = ptr->steer;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub, erp42_sub;
    ros::Publisher state_pub;

    double v_cur, w_cur, steer_erp42;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "vehicle_status_sender");

    VehicleStatusSender v;
    ros::spin();
    
}