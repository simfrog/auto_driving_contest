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
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

class TwistMerger{
public:
    TwistMerger() : seq(0){
        std::string odom_topic, imu_topic, twist_topic;
        nh.param<std::string>("odom_imu_merger/odom_topic", odom_topic, "erp42_odometry");
        nh.param<std::string>("odom_imu_merger/imu_topic", imu_topic, "imu");
        nh.param<std::string>("odom_imu_merger/twist_topic", twist_topic, "can_twist");

        odom_sub = nh.subscribe(odom_topic, 100, &TwistMerger::odomCb, this);
        imu_sub = nh.subscribe(imu_topic, 100, &TwistMerger::imuCb, this);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>(twist_topic, 100);
    }

    void imuCb(const sensor_msgs::ImuConstPtr& ptr){
        imu_mutex.lock();
        imu = *ptr;
        imu_mutex.unlock();
    }
    
    void odomCb(const nav_msgs::OdometryConstPtr& ptr){
        odom = *ptr;
        pubTwist();
    }
    
    void pubTwist(){
        geometry_msgs::TwistStamped twist;
        
        twist.header.seq = seq++;
        twist.header.stamp = odom.header.stamp;
        twist.header.frame_id = "base_link";

        twist.twist.linear = odom.twist.twist.linear;
        imu_mutex.lock();
        twist.twist.angular = imu.angular_velocity;
        imu_mutex.unlock();

        twist_pub.publish(twist);
    }

private:
    ros::Subscriber odom_sub, imu_sub;
    ros::Publisher twist_pub;
    ros::NodeHandle nh;

    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;

    std::mutex imu_mutex;
    uint32_t seq;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "erp42_imu_merger");
    TwistMerger t;
    ros::spin();
}