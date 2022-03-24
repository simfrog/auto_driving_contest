#include <iostream>
#include <sstream>
#include <string>
#include <imu_vn100_yu/imu_driver.h>
#include <ros/ros.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>

static double DEG2RAD = M_PI / 180;    
static int error_cnt = 0;
VN100IMU *VN100IMU::ptr = nullptr;

VN100IMU::VN100IMU() {}

sensor_msgs::Imu VN100IMU::read_and_parse(){
    size_t read_sz = imu_serial.read(buf, BUF_LEN); //store in buf, return read size
    serialReader.arrangeData(buf, read_sz);
    std::string str = serialReader.getSentence(); //read one line
    serial_read_ok = false;

    if(str == ""){
        error_cnt++;
        ROS_INFO("Read Error : cnt = %d",error_cnt);
	
    }
    else{
        std::string str_checksum = str.substr(91,2);

        unsigned char ChecksumBuf[BUF_LEN];
        strcpy( (char*) ChecksumBuf, str.c_str());
        unsigned char real_checksum = calculateChecksum(ChecksumBuf,90);
        unsigned int serial_checksum = (unsigned int)strtol(str_checksum.c_str(), NULL, 16);


        if(real_checksum != (unsigned char)serial_checksum){
            ROS_INFO("checksum error");
            return imu_msg;
        }
        
       std::string str_yaw = str.substr(7,8);
       std::string str_pitch = str.substr(16,8);
       std::string str_roll = str.substr(25,8);
       std::string str_accelX = str.substr(34,7);
       std::string str_accelY = str.substr(42,7);
       std::string str_accelZ = str.substr(50,7);
       std::string str_gyroX = str.substr(58,10);
       std::string str_gyroY = str.substr(69,10);
       std::string str_gyroZ = str.substr(80,10);
      	
       imu_msg.header.seq++;
       imu_msg.header.stamp = ros::Time::now();
       imu_msg.header.frame_id = "imu";
       imu_msg.linear_acceleration.x = std::stod(str_accelX); //m/s^2
       imu_msg.linear_acceleration.y = std::stod(str_accelY);
       imu_msg.linear_acceleration.z = std::stod(str_accelZ);
       imu_msg.angular_velocity.x = std::stod(str_gyroX); //rad/s
       imu_msg.angular_velocity.y = std::stod(str_gyroY);
       imu_msg.angular_velocity.z = std::stod(str_gyroZ);

       double roll = std::stod(str_roll) * DEG2RAD;
       double pitch = std::stod(str_pitch) * DEG2RAD;
       double yaw = std::stod(str_yaw) * DEG2RAD;
       tf2::Quaternion q;
       q.setRPY(roll, pitch, yaw);
       tf2::convert(q, imu_msg.orientation);
       serial_read_ok = true;
    }
    return imu_msg;    
}

inline unsigned char VN100IMU::calculateChecksum(unsigned char data[], unsigned int length){
    unsigned int i;
    unsigned char cksum = 0;
    for(i=1; i<length; i++){
        cksum ^= data[i];
    }
    return cksum;
}
    
bool VN100IMU::ok(){
    return serial_read_ok;
}

void VN100IMU::destroy(){
    if (ptr) 
        delete ptr;
}

VN100IMU::~VN100IMU(){
    if (imu_serial.isOpen()) 
        imu_serial.close();
}

void VN100IMU::createInstance(std::string port, double baudrate){
    ptr = new VN100IMU();
    ptr->setupImu(port, baudrate);
}

void VN100IMU::setupImu(std::string port,double baudrate){ 
    imu_serial.setPort(port);
    imu_serial.setBaudrate(baudrate);
    imu_serial.open();
    if (false == imu_serial.isOpen()){
        ROS_ERROR("Serial.open() error with port %s", port.c_str());
        throw serial::PortNotOpenedException(port.c_str());
    }
    ROS_INFO("open imu serial device success!");
}

VN100IMU* VN100IMU::getInstancePtr() {
    return ptr;
}
