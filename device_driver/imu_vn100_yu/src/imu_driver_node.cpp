#include <ros/ros.h>
#include <signal.h>
#include <imu_vn100_yu/imu_driver.h>

void mySigintHandler(int sig) //컨씨 종료 위해
{
  VN100IMU::getInstancePtr()->destroy();
  ros::shutdown();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "VN100IMU", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Rate r(100);//100hz
    /* control flow : SIGINT -> signal_handler */

    std::string port;
    double baudrate;
    try{
    if (false == nh.getParam("vn100_imu/port", port))
        throw std::runtime_error("[vn100_imu] set port!");
    if(false == nh.getParam("vn100_imu/baudrate", baudrate))
        throw std::runtime_error("[vn100_imu] set baudrate!");
        VN100IMU::createInstance(port, baudrate);
        VN100IMU* imu_core_ptr = VN100IMU::getInstancePtr();        
        while(ros::ok()){
            auto imu_msg = imu_core_ptr->read_and_parse();
            if (imu_core_ptr->ok())
                pub.publish(imu_msg);
            r.sleep();
        }
    }
    catch(serial::PortNotOpenedException){
        ROS_ERROR("imu driver : PortNotOpenedException");
    }
    catch(serial::SerialException){ 
        ROS_ERROR("imu driver : SerialException!");
    }
    catch(serial::IOException){ 
        ROS_ERROR("imu driver : IOException!");
    }
    ros::shutdown();
    return 0;    
}
