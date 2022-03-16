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

#include <ros/ros.h>
#include <signal.h>
#include <mysen_imu_driver/driver_core.h>

void mySigintHandler(int sig)
{
  MySenIMU::getInstancePtr()->destroy();
  ros::shutdown();
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "mysen_imu", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Rate r(300);//100hz
    /* control flow : SIGINT -> signal_handler */

    std::string port;
    int mode;
    if (false == nh.getParam("mysen_imu/port", port)) throw std::runtime_error("[mysen_imu] set port!");
    if (false == nh.getParam("mysen_imu/mode", mode)) throw std::runtime_error("[mysen_imu] set mode!");

    try{
        MySenIMU::createInstance(port, mode); // I don't know why, but mode 5 doesn't work well
        MySenIMU* imu_core_ptr = MySenIMU::getInstancePtr();        
        
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
