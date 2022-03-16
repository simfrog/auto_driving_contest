// Software License Agreement (BSD License)
//
// Copyright (c) 2020, Taewook Park <sjrnfu12@naver.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the names of the authors nor the names of their
//    affiliated organizations may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <sstream>
#include <string>
#include <mysen_imu_driver/driver_core.h>
#include <ros/ros.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static constexpr uint8_t END_CODE1 = 0x2C, END_CODE2 = 0x03;
static size_t N_MODE5_BYTE = 32;
static size_t N_MODE6_BYTE = 30;

MySenIMU *MySenIMU::ptr = nullptr;

MySenIMU::MySenIMU() {}

sensor_msgs::Imu MySenIMU::read_and_parse(){
    /* parse only first data packet, even if there's more */
	/* assume imu is in mode 6 */
    static int read_error_cnt = 0;
    constexpr static int READ_ERROR_LIMIT = 100;
    serial_read_ok = false;

    size_t read_sz = imu_serial.read(buf, N_MODE6_BYTE);
    size_t data_len = getDataLength(buf, read_sz);

    //check for disconnect
    if (0 == data_len){
        read_error_cnt++;
        if (READ_ERROR_LIMIT == read_error_cnt){
            ROS_ERROR("too much read error!");
            this->destroy();
            throw serial::PortNotOpenedException("");
        }
        
        return imu_msg;
    
    } 
    else read_error_cnt = 0;

    static double DEG2RAD = M_PI / 180;    
    
    if (data_len == N_MODE5_BYTE){
        if (false == isValidChecksum(buf + 1, buf + 28, buf[29])) {
            ROS_INFO("checksum error");
            return imu_msg;
        }
        imu_msg.header.seq++;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        imu_msg.linear_acceleration.x   = 1.0 * (buf[1] * 256 + buf[2]) / 1000 - 5.0;
        imu_msg.linear_acceleration.x   *= 9.8;
        imu_msg.linear_acceleration.y   = 1.0 * (buf[3] * 256 + buf[4]) / 1000 - 5.0;
        imu_msg.linear_acceleration.y   *= 9.8;
        imu_msg.linear_acceleration.z   = 1.0 * (buf[5] * 256 + buf[6]) / 1000 - 5.0;
        imu_msg.linear_acceleration.z   *= 9.8;

        imu_msg.angular_velocity.x      = 1.0 * (buf[7] * 256 + buf[8]) / 100   - 320.0;
        imu_msg.angular_velocity.x      *= DEG2RAD;
        imu_msg.angular_velocity.y      = 1.0 * (buf[9] * 256 + buf[10]) / 100  - 320.0;
        imu_msg.angular_velocity.y      *= DEG2RAD;
        imu_msg.angular_velocity.z      = 1.0 * (buf[11] * 256 + buf[12]) / 100 - 320.0;
        imu_msg.angular_velocity.z      *= DEG2RAD;

        imu_msg.orientation.x           = 1.0 * (buf[19] * 256 + buf[20]) / 1000 - 2.0;
        imu_msg.orientation.y           = 1.0 * (buf[21] * 256 + buf[22]) / 1000 - 2.0;
        imu_msg.orientation.z           = 1.0 * (buf[23] * 256 + buf[24]) / 1000 - 2.0;
        imu_msg.orientation.w           = 1.0 * (buf[25] * 256 + buf[26]) / 1000 - 2.0;
        if (fabs(imu_msg.angular_velocity.z) > 1){
            ROS_ERROR("hmm %lu %lu", read_sz, data_len);
        }
        serial_read_ok = true;
    }
    else if (data_len == N_MODE6_BYTE){
        if (false == isValidChecksum(buf + 1, buf + 26, buf[27])) {
            ROS_INFO("checksum error");
            return imu_msg;
        }
        imu_msg.header.seq++;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        imu_msg.linear_acceleration.x   = 1.0 * (buf[1] * 256 + buf[2]) / 1000 - 5.0;
        imu_msg.linear_acceleration.x   *= 9.8;
        imu_msg.linear_acceleration.y   = 1.0 * (buf[3] * 256 + buf[4]) / 1000 - 5.0;
        imu_msg.linear_acceleration.y   *= 9.8;
        imu_msg.linear_acceleration.z   = 1.0 * (buf[5] * 256 + buf[6]) / 1000 - 5.0;
        imu_msg.linear_acceleration.z   *= 9.8;

        imu_msg.angular_velocity.x      = 1.0 * (buf[7] * 256 + buf[8]) / 100   - 320.0;
        imu_msg.angular_velocity.x      *= DEG2RAD;
        imu_msg.angular_velocity.y      = 1.0 * (buf[9] * 256 + buf[10]) / 100  - 320.0;
        imu_msg.angular_velocity.y      *= DEG2RAD;
        imu_msg.angular_velocity.z      = 1.0 * (buf[11] * 256 + buf[12]) / 100 - 320.0;
        imu_msg.angular_velocity.z      *= DEG2RAD;

        double roll                = 1.0 * (buf[19] * 256 + buf[20]) / 100 - 180.0;
        double pitch               = 1.0 * (buf[21] * 256 + buf[22]) / 100 - 90.0;
        double yaw                 = 1.0 * (buf[23] * 256 + buf[24]) / 100 - 180.0;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q, imu_msg.orientation);

        serial_read_ok = true;
    }
    return imu_msg;
}

size_t MySenIMU::getDataLength(uint8_t *begin, size_t read_sz){
    if (read_sz <= 3) return read_sz; //start(1B) + end(2B) >= 3
    size_t len = 0;
	uint8_t *ptr = begin + read_sz;
	if ((begin[read_sz - 1] == END_CODE2) && (begin[read_sz - 2] == END_CODE1))
		return read_sz;
	else return 0;
}

bool MySenIMU::ok(){
    return serial_read_ok;
}

inline bool MySenIMU::isValidChecksum(uint8_t* begin, uint8_t* end, uint8_t checksum){
    uint32_t sum = 0;
    size_t checksum_cnt_limit = 100;
    while(begin != end){
        sum += *begin;
        begin++;
        --checksum_cnt_limit;
        if (!checksum_cnt_limit) {
            ROS_ERROR("invalid checksum address!");
            return false;
        }
    }
    sum += *end;
    if (checksum == uint8_t(sum)) return true;
    else return false;
}

void MySenIMU::destroy(){
    if (ptr) 
        delete ptr;
}

MySenIMU::~MySenIMU(){
    if (imu_serial.isOpen()) 
        imu_serial.close();
}

void MySenIMU::createInstance(std::string port, int mode){
    ptr = new MySenIMU();
    ptr->setupImu(port, mode);
}

void MySenIMU::setupImu(std::string port, int mode){ 
    imu_serial.setPort(port);
    imu_serial.setBaudrate(115200);
    imu_serial.open();
    if (false == imu_serial.isOpen()){
        ROS_ERROR("Serial.open() error with port %s", port.c_str());
        throw serial::PortNotOpenedException(port.c_str());
    }
    ROS_INFO("open imu serial device success!");

    /* change imu mode to 5. See the manual of mySen imu */
    
    int fd = open(port.c_str(), O_RDWR | O_NONBLOCK);
    while(true){
        if (fd < 0) {
            this->destroy();
            ROS_ERROR("can not open serial file : %s", port.c_str());
            throw serial::PortNotOpenedException(port.c_str());
        }
        switch(mode){
        case 5: write(fd, "5555", 4); break;
        case 6: write(fd, "6666", 4); break;
        default:
            ROS_ERROR("invalid imu mode : %d", mode);
            this->destroy();
            throw serial::PortNotOpenedException(port.c_str());
        }
        //I don't know why, but working with serial write() doesn't work
        //so I used file write to change mode
        
        ros::Rate(100).sleep();
		if (imu_serial.available() < N_MODE5_BYTE) continue; //initial mode == 5
        size_t read_sz = imu_serial.read(buf, BUF_LEN);
		for(size_t i = 0; i < read_sz; ++i)
			printf("%x ", buf[i]);
		printf("\n");
        size_t data_len = getDataLength(buf, read_sz);
		printf("read sz, data len : %lu %lu\n", read_sz, data_len);
        bool ok = false;
        switch(mode){
        case 5: if (0 == (data_len % N_MODE5_BYTE)) ok = true; break;
        case 6: if (0 == (data_len % N_MODE6_BYTE)) ok = true; break;
        default : break;
        }
        if (ok){
            close(fd);
            break;
        }
    }
    ROS_INFO("change imu mode to %d success!", mode);
}

MySenIMU* MySenIMU::getInstancePtr() {
    return ptr;
}
