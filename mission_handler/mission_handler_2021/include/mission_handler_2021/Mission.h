// Software License Agreement (BSD License)
//
// Copyright (c) 2020, Taewook Park <sjrnfu12@naver.com>, jaehun Kim, Sanguk Yu, Hongchal Jo
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
#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <traffic_light_msgs/Light.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <mission_handler_2021/IsPointInPolygon.h>
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <traffic_area_msgs/traffic_area.h>
#include <mission_handler_2021/traffic_light_rule.h>
#include <deque>
// #include <tf/transform_listener.h> //throw
#include <string>

//parking
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include <std_msgs/Int32.h>
#include <autoware_msgs/DetectedObjectArray.h>

//delivery
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <vector_map_msgs/PointArray.h>

//emergency_obstacle
#include <tf/tf.h>
// #define M_PI 3.141592

visualization_msgs::Marker points_estop;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Vehicle2Mission{
    Vehicle2Mission() :cur_light_updated(false), obstacle_center_points_updated(false), dv_signs_updated(false), isStart(false) {}  //yap dv_mode:0
    geometry_msgs::PoseStamped curpos;
    double linear_velocity;
    // start
    bool isStart;
    // traffic
    traffic_light_msgs::Light cur_light;
    bool cur_light_updated;
    // emergency_obstacle
    std::vector<geometry_msgs::Point32> obstacle_center_points;
    bool obstacle_center_points_updated;
    // delivery
    autoware_msgs::DetectedObjectArray dv_signs;
    bool dv_signs_updated;
};

struct Mission2Vehicle{
    Mission2Vehicle() : isMissionDone(false), isAvoidMission(false), isParkingMission(false), isEmergencyObstacle(false) {}
    bool isMissionDone;
    bool isAvoidMission;
    bool isParkingMission;
    bool isEmergencyObstacle;
    void clear() {isAvoidMission = false; isParkingMission = false; isEmergencyObstacle = false;}
};

class Mission{
public:
    Mission(ros::NodeHandle& nh, std::string mission_name) : mission_name_(mission_name) {    } //interface for constructor
    virtual Mission2Vehicle processMission(Vehicle2Mission& v2m) = 0;
    virtual void visualize() {}
    virtual void clean() {} //override if needed
    std::string to_string() const {return mission_name_;}
private:
    std::string mission_name_;
};

class MissionNone : public Mission{
public:
    MissionNone(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name) {
        max_vel_pub = nh.advertise<std_msgs::Float64>("/op_planner_max_velocity", 10);
    }
    Mission2Vehicle processMission(Vehicle2Mission& v2m) {
        if(v2m.isStart){
            std_msgs::Float64 max_vel;  
            max_vel.data = 1.0;
            max_vel_pub.publish(max_vel);
            v2m.isStart = false;
        }
        Mission2Vehicle m;
        m.isMissionDone = false; //never done
        return m;
    }
private:
    ros::Publisher max_vel_pub;    
};

/* helper functions */
static vector_map_msgs::PointArray vm_points;
// std::vector<geometry_msgs::Pose> vm_poses;
template <typename T>
static void myGetParam(ros::NodeHandle& nh, std::string param_name, T& src){
    if (!nh.getParam(param_name, src)) throw std::runtime_error(std::string() + "set " + param_name + "!!");
}
// template <typename T>
// static void mySetParam(ros::NodeHandle& nh, std::string param_name, T& src){
//     nh.setParam(param_name, src);
// }
double getCriticalDistance(double v){
    constexpr static double CAR_TIP_MARGIN = 1.4;
    return CAR_TIP_MARGIN + 0.1767*std::pow(v, 2) + 0.3508 * v - 0.04;
    //return v + CAR_TIP_MARGIN;//0.055 * std::pow(v, 4) - 0.6128 * std::pow(v, 3) + 2.4117 * std::pow(v, 2) - 3.6048 * v + 1.95 + CAR_TIP_MARGIN + SAFETY_MARGIN;
}
double Quaternion2Yaw(Vehicle2Mission& msg){
    tf::Quaternion q(
        msg.curpos.pose.orientation.x,
        msg.curpos.pose.orientation.y,
        msg.curpos.pose.orientation.z,
        msg.curpos.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll,pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
double AngleBetweenTwoAnglesPositive(const double& a1,const double& a2){
    double diff = a1 - a2;
    if(diff < 0) 
        diff = a2 - a1;
    if(diff > M_PI) 
        diff = 2*M_PI - diff;
    return diff;
}
bool isForwardWaypoint(const auto& wp, Vehicle2Mission& v2m){
    double x = v2m.curpos.pose.position.x; 
    double y = v2m.curpos.pose.position.y;
    double yaw = atan2(y - wp.y, x - wp.x);
    double angle_diff = AngleBetweenTwoAnglesPositive(Quaternion2Yaw(v2m), yaw) * 180 / M_PI;
    if(angle_diff < 90) 
        return true;
    else 
        return false;
}
double distanceBetweenTwoPoints(geometry_msgs::Point p1, geometry_msgs::Point p2){
    double dist = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    return dist;
}
int getClosestWaypointIdx(geometry_msgs::Point current_pose){
    double dist, min_dist=9999999;
    int min_idx, idx=0;
    for (vector_map_msgs::Point& p : vm_points.data){
        geometry_msgs::Point pt;
        pt.x = p.ly;
        pt.y = p.bx;
        dist = distanceBetweenTwoPoints(pt, current_pose);
        if(dist < min_dist){
            min_dist = dist;
            min_idx = idx;
        }
        idx++;
    }
    return min_idx;
}

// =================================================== 신호등미션 시작 ===================================================
class MissionTrafficLight: public Mission{
public:
    MissionTrafficLight(ros::NodeHandle& nh, std::string mission_name): 
        Mission(nh, mission_name), isDone(false), isRed(false), stop_once(false)
    {
        /* load params */
        std::string traffic_type;
        myGetParam(nh, to_string() + "/n_hole", n_hole_);
        myGetParam(nh, to_string() + "/type", traffic_type);
        myGetParam(nh, to_string() + "/section", section_);
        myGetParam(nh, to_string() + "/traffic_velocity", traffic_velocity);
        myGetParam(nh, to_string() + "/driving_velocity", driving_velocity);
        myGetParam(nh, to_string() + "/safety_margin", safety_margin);
        myGetParam(nh, to_string() + "/max_waiting_time", max_waiting_time);

        std::vector<double> stopline_point_vec_tmp;
        myGetParam(nh, to_string() + "/stopline_points", stopline_point_vec_tmp);
        geometry_msgs::Point p1_stopline, p2_stopline;
        p1_stopline.x = stopline_point_vec_tmp[0]; p1_stopline.y = stopline_point_vec_tmp[1];
        p2_stopline.x = stopline_point_vec_tmp[2]; p2_stopline.y = stopline_point_vec_tmp[3];

        /* initializer */
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        stopline_pub = nh.advertise<visualization_msgs::Marker>("cur_stopline", 10);
        traffic_light_pub = nh.advertise<visualization_msgs::Marker>("traffic_light_state", 10);
        if (traffic_type == "STRAIGHT") target_light_state = TrafficLightState::STRAIGHT;
        else if (traffic_type == "LEFT") target_light_state = TrafficLightState::LEFT;
        else throw std::runtime_error(std::string() + "unknown traffic light type : " + traffic_type);

        traffic_area_pub = nh.advertise<traffic_area_msgs::traffic_area>("traffic_area_info", 10);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        /* calc line's equation's coefficient : ax - y + (y1 - ax1) = 0
                                                 a   b         c            */
        if (std::fabs(p1_stopline.x - p2_stopline.x) < 0.00001){ //vertical line
            a = 1;
            b = 0;
            c = -p1_stopline.x;
        } 
        else { //normal line including horizontal
            a = (p1_stopline.y - p2_stopline.y) / (p1_stopline.x - p2_stopline.x); //slope
            b = -1; //slope related
            c = p1_stopline.y - a*p1_stopline.x;
        }

        //initiate stopline visualization
        stopline_marker.header.frame_id = "map";
        stopline_marker.header.stamp = ros::Time::now();
        stopline_marker.ns = "stopline";
        stopline_marker.action = visualization_msgs::Marker::ADD;
        stopline_marker.pose.orientation.w = 1.0;
        stopline_marker.type = visualization_msgs::Marker::LINE_LIST;
        stopline_marker.scale.x = 0.5;
        stopline_marker.color.r = 1.0;
        stopline_marker.color.g = 1.0;
        stopline_marker.color.b = 1.0;
        stopline_marker.color.a = 0.8;
        stopline_marker.id = 0;

        stopline_marker.points.push_back(p1_stopline); stopline_marker.points.push_back(p2_stopline);

        pub_estop_points = nh.advertise<visualization_msgs::Marker>("/estop_point", 1);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        //pub n traffic light hole
        traffic_area_msgs::traffic_area t_info;
        t_info.n_hole = n_hole_;
        t_info.section = section_;
        traffic_area_pub.publish(t_info);

        std_msgs::Float64 max_vel;
        max_vel.data = traffic_velocity;
        max_vel_pub.publish(max_vel);

        Mission2Vehicle m;      
        m.isMissionDone = false;  //추가
        if (false == isDone)  {
            if (v2m.cur_light_updated) {
                v2m.cur_light_updated = false;
            }
            else {
                m.isMissionDone = false; 
                return m;
            }

        //update current traffic light
            if (v2m.cur_light.light == traffic_light_msgs::Light::STRANGE){
                m.isMissionDone = false; 
                return m;
            }
            auto traffic_light_state_cur = traffic_light_rule.getPromisingState(v2m.cur_light);
        //for visualize
            if (traffic_light_state_cur == TrafficLightState::STRAIGHT) cur_light_string = "STRAIGHT";
            else if (traffic_light_state_cur == TrafficLightState::STRAIGHT_AND_LEFT) cur_light_string = "S & L";
            else if (traffic_light_state_cur == TrafficLightState::LEFT) cur_light_string = "LEFT";
            else if (traffic_light_state_cur == TrafficLightState::YELLOW) cur_light_string = "YELLOW";
            else if (traffic_light_state_cur == TrafficLightState::RED) cur_light_string = "RED";
            else if (traffic_light_state_cur == TrafficLightState::STRANGE) cur_light_string = "";
            else throw std::runtime_error("impossible situation happened in traffic light handler");

        //calc dist to stopline
            double x = v2m.curpos.pose.position.x;
            double y = v2m.curpos.pose.position.y;
            double dist_to_stopline = std::fabs(a*x + b*y + c) / std::sqrt(a*a + b*b);
            
        //decide vehicle behaviour
            // std::cout << v2m.linear_velocity << std::endl;
        // // ROS_WARN("dist to stopline, critical + safety : %lf %lf", dist_to_stopline, getCriticalDistance(v2m.linear_velocity) + safety_margin);
        //     if (dist_to_stopline < (getCriticalDistance(v2m.linear_velocity) + safety_margin)){
        //         if(stop_once == false){
        //             std::cout << "Let's estop!!" << std::endl;
        //             stop_once = true;
        //             waiting_start_time = ros::Time::now();
        //             msg.data = true;
        //             estop_pub.publish(msg);

        //             std_msgs::ColorRGBA c;

        //             int id = 0;
        //             c.r = 1.0;
        //             c.g = 0.0;
        //             c.b = 0.0;
        //             c.a = 1.0;
        //             geometry_msgs::Point pt;
        //             pt.x = x;
        //             pt.y = y;
        //             pt.z = 0.0;
        //             points_estop.header.frame_id = "/map";
        //             points_estop.header.stamp = ros::Time::now();
        //             points_estop.ns = "points";
        //             points_estop.action = visualization_msgs::Marker::ADD;
        //             points_estop.pose.orientation.w = 1.0;
        //             points_estop.lifetime = ros::Duration();
        //             points_estop.id = id++;
        //             points_estop.type = visualization_msgs::Marker::POINTS;
        //             points_estop.scale.x = 1.0;
        //             points_estop.scale.y = 1.0;

        //             points_estop.color.r = 0.0;
        //             points_estop.color.g = 0.0;
        //             points_estop.color.b = 1.0;
        //             points_estop.color.a = 1.0;

        //             points_estop.points.push_back(pt);
        //             points_estop.colors.push_back(c);
        //             pub_estop_points.publish(points_estop);
        //         }
        //     }

        //     if((ros::Time::now()-waiting_start_time).toSec() >= max_waiting_time){
        //         msg.data = false;
        //         estop_pub.publish(msg);
        //         m.isMissionDone = true;   
        //     }
            //추가
            
            
            
            
            if ((dist_to_stopline < (getCriticalDistance(v2m.linear_velocity) + safety_margin)) || isRed){
                std::cout << "traffic_light_state_cur : " << cur_light_string << std::endl;
                // std::cout << "traffic type : " << traffic_type << std::endl;
                if ((traffic_light_state_cur == target_light_state) || //able to go
                    (traffic_light_state_cur == TrafficLightState::STRAIGHT_AND_LEFT)){
                    std_msgs::Bool b;
                    b.data = false;
                    estop_pub.publish(b);

                    max_vel.data = driving_velocity;
                    max_vel_pub.publish(max_vel);
                    isDone = true;
                }
                else { // can not go
                    if (isRed == false) 
                        waiting_start_time = ros::Time::now();

                    isRed = true;
                    std_msgs::Bool b;
                    std::cout << "Let's estop!!" << std::endl;

                    std_msgs::ColorRGBA c;

                    int id = 0;
                    c.r = 1.0;
                    c.g = 0.0;
                    c.b = 0.0;
                    c.a = 1.0;
                    geometry_msgs::Point pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = 0.0;
                    points_estop.header.frame_id = "/map";
                    points_estop.header.stamp = ros::Time::now();
                    points_estop.ns = "points";
                    points_estop.action = visualization_msgs::Marker::ADD;
                    points_estop.pose.orientation.w = 1.0;
                    points_estop.lifetime = ros::Duration();
                    points_estop.id = id++;
                    points_estop.type = visualization_msgs::Marker::POINTS;
                    points_estop.scale.x = 1.0;
                    points_estop.scale.y = 1.0;

                    points_estop.color.r = 0.0;
                    points_estop.color.g = 0.0;
                    points_estop.color.b = 1.0;
                    points_estop.color.a = 1.0;

                    points_estop.points.push_back(pt);
                    points_estop.colors.push_back(c);
                    pub_estop_points.publish(points_estop);


                    b.data = true;
                    estop_pub.publish(b);    
                }
            }
            std::cout << "time : " << (ros::Time::now() - waiting_start_time).toSec() << std::endl;
            //force traffic handler to be done when RedLight maintained too long
            if (isRed && (ros::Time::now() - waiting_start_time).toSec() >= max_waiting_time)
                isDone = true;
        }

        m.isMissionDone = isDone;   
        return m;
    }

    void visualize() {
        //stopline
        stopline_marker.header.stamp = ros::Time::now();
        stopline_pub.publish(stopline_marker);

        //traffic light
        visualization_msgs::Marker cur_light_marker;
        cur_light_marker.header.frame_id = "base_link";
        cur_light_marker.header.stamp = ros::Time::now();
        cur_light_marker.ns = "traffic_light_state";
        cur_light_marker.id = 0;
        cur_light_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        cur_light_marker.action = visualization_msgs::Marker::ADD;
        cur_light_marker.pose.orientation.x = 0.0;
        cur_light_marker.pose.orientation.y = 0.0;
        cur_light_marker.pose.orientation.z = 0.0;
        cur_light_marker.pose.orientation.w = 1.0;
        cur_light_marker.scale.z = 3;
        
        cur_light_marker.pose.position.x = 3.0;
        cur_light_marker.pose.position.y = 0;
    
        if (cur_light_string == "STRAIGHT" || cur_light_string == "S & L" || cur_light_string == "LEFT"){
            cur_light_marker.color.a = 1.0;
            cur_light_marker.color.r = 0.0;
            cur_light_marker.color.g = 1.0;
            cur_light_marker.color.b = 0.0;
        } else if (cur_light_string == "YELLOW"){
            cur_light_marker.color.a = 1.0;
            cur_light_marker.color.r = 1.0;
            cur_light_marker.color.g = 1.0;
            cur_light_marker.color.b = 0.0;
        } else if (cur_light_string == "RED"){
            cur_light_marker.color.a = 1.0;
            cur_light_marker.color.r = 1.0;
            cur_light_marker.color.g = 0.0;
            cur_light_marker.color.b = 0.0;
            std::string remain_time_str = std::string() + 
                "(" + std::to_string((ros::Time::now() - waiting_start_time).toSec()) + ")";
            cur_light_string = cur_light_string + remain_time_str;
        } else {}
        cur_light_marker.text = cur_light_string;
        traffic_light_pub.publish(cur_light_marker);
    }
    void clean() {
        std_msgs::Bool b;
        b.data = false;
        estop_pub.publish(b);

        std_msgs::Float64 max_vel;
        max_vel.data = driving_velocity;
        max_vel_pub.publish(max_vel);
        // mySetParam(nh, "/op_common_params/maxVelocity", max_vel.data);

        stopline_marker.header.stamp = ros::Time::now();        
        stopline_marker.points.clear();
        stopline_pub.publish(stopline_marker);
        visualize();
    }

    double a, b, c; //stopline distance calculation members
    bool isDone;
    ros::Publisher estop_pub;
    ros::Publisher traffic_area_pub;
    ros::Publisher max_vel_pub;
    ros::Publisher pub_estop_points;

    double traffic_velocity;
    double driving_velocity;
    double safety_margin;
    TrafficLightState target_light_state;
    int n_hole_;
    int section_;
    TrafficLightRule traffic_light_rule;
    double max_waiting_time;
    ros::Time waiting_start_time;
    ros::NodeHandle nh;

    //visualizer 
    ros::Publisher stopline_pub;
    ros::Publisher traffic_light_pub;
    visualization_msgs::Marker stopline_marker;
    std::string cur_light_string;
    bool isRed;
    bool stop_once;
    std_msgs::Bool msg;

    
        
};
// =================================================== 신호등미션 끝 ===================================================

// =================================================== 배달미션 시작 ===================================================
enum{DETECTION_1=0, PICK_UP, DRIVE, DETECTION_2, DELIVERY, DONE};
class MissionDelivery: public Mission{
public:
    MissionDelivery(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name), stop_once(false), waiting_start_time(0)
    {
        myGetParam(nh, to_string() + "/searching_velocity", s_v);
        myGetParam(nh, to_string() + "/driving_velocity", d_v);
        myGetParam(nh, to_string() + "/epsilon_goal_current", epsilon);
        myGetParam(nh, to_string() + "/epsilon_goal_current_a", epsilon_a);
        myGetParam(nh, to_string() + "/max_waiting_time", max_waiting_time);
        myGetParam(nh, to_string() + "/level", level);
        myGetParam(nh, to_string() + "/not_use_pickup_detecton", not_use_pickup_detection);
        myGetParam(nh, to_string() + "/pickup_sign_number", pickup_sign_number);
        myGetParam(nh, to_string() + "/dv_mode", dv_mode);
        
        max_vel_pub = nh.advertise<std_msgs::Float64>("/op_planner_max_velocity", 10);
        // max_vel_pub = nh.advertise<std_msgs::Float64>("/op_common_params/maxVelocity", 10); //아몰랑!해보기!
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        pub_stop_goal_a = nh.advertise<visualization_msgs::Marker>("/stop_goal_a", 1);
        pub_stop_goal_b = nh.advertise<visualization_msgs::Marker>("/stop_goal_b", 1);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        Mission2Vehicle m;
        m.isMissionDone = false;
            /* way1 */
        /* current mode classification */
std::cout << "[*] dv_mode: " << dv_mode << std::endl;

        /* (0)pick-up sign detection */
        if(dv_mode == DETECTION_1){
            //publish max velocity
            max_vel.data = s_v;
            max_vel_pub.publish(max_vel);
            for(auto& sign: v2m.dv_signs.objects){
// std::cout << "inside for, label : " << sign.label << std::endl;
                if(sign.label == "A1" || sign.label == "A2" || sign.label == "A3"){
std::cout << "[1] I found " << sign.label << " !!" << std::endl;
                    dv_goal_num = sign.label.erase(0,1); // get sign number
std::cout << "[1] dv_goal_num: " << dv_goal_num << std::endl;
                    p_sign = sign.pose.position; // get sign position
                    int idx = getClosestWaypointIdx(p_sign);
std::cout << "[1] closest inx: " << idx << std::endl;
                    p_goal.x = vm_points.data[idx].ly; // get pick-up position
                    p_goal.y = vm_points.data[idx].bx;

                    visualization_msgs::Marker points_;
                    std_msgs::ColorRGBA c;

                    int id = 0;
                    c.r = 1.0;
                    c.g = 0.0;
                    c.b = 0.0;
                    c.a = 1.0;
                    geometry_msgs::Point pt;
                    pt.x = p_goal.x;
                    pt.y = p_goal.y;
                    pt.z = 0.0;
                    points_.header.frame_id = "/map";
                    points_.header.stamp = ros::Time::now();
                    points_.ns = "points";
                    points_.action = visualization_msgs::Marker::ADD;
                    points_.pose.orientation.w = 1.0;
                    points_.lifetime = ros::Duration();
                    points_.id = id++;
                    points_.type = visualization_msgs::Marker::POINTS;
                    points_.scale.x = 1.0;
                    points_.scale.y = 1.0;

                    points_.color.r = 0.0;
                    points_.color.g = 0.0;
                    points_.color.b = 1.0;
                    points_.color.a = 1.0;

                    points_.points.push_back(pt);
                    points_.colors.push_back(c);
                    pub_stop_goal_a.publish(points_);
                    
// std::cout << "before dv_mode++ (0)" << std::endl;
                    dv_mode++;    // next mode
                    break;
                }
            }
        }
        /* (1)pick-up, (4) delivery */
        else if(dv_mode == PICK_UP || dv_mode == DELIVERY){   
        // else if(v2m.dv_mode == 1 || v2m.dv_mode == 3){   
            //publish max velocity
            max_vel.data = s_v;
            max_vel_pub.publish(max_vel);
            double dist = distanceBetweenTwoPoints(v2m.curpos.pose.position, p_goal);

            if(stop_once == false && dist <= epsilon){
                stop_once = true;
                std::cout << "stop !!" << std::endl;
                waiting_start_time = ros::Time::now();
                msg.data = true;
                estop_pub.publish(msg);        
            }

            if((ros::Time::now() - waiting_start_time).toSec() >= max_waiting_time && stop_once == true){
                msg.data = false;
                estop_pub.publish(msg);
// std::cout << "before dv_mode++ (1),(3)" << std::endl;
                dv_mode++;  // next mode
            }
        }
         /* (2)driving */
        else if(dv_mode == DRIVE){
            //publish max velocity
            max_vel.data = d_v;
            max_vel_pub.publish(max_vel);
                /* way1:camera detection */
            // for(auto& sign: v2m.dv_signs.objects){
            //     // if(v2m.dv_goal_num == atoi(sign.label.erase(0,1))){
            //     if(sign.label == "A1" || sign.label == "A2" || sign.label == "A3"){                       
            //         v2m.dv_mode++;    // next mode
            //         break;
            //     }
            // }

                /* way2:location */
            // if(isForwardWaypoint(v2m.p_sign, v2m)){
                // double dist = distanceBetweenTwoPoints(v2m.curpos.pose.position, v2m.p_sign);
                // if(stop_once == false && dist <= epsilon_a){
                    // v2m.dv_mode++;    // next mode
                // }
            // }

            double dist = distanceBetweenTwoPoints(v2m.curpos.pose.position, p_sign);
            
            if(dist > epsilon_a && level == 0) level++;

            if(dist < epsilon_a && level == 1) level++;

            if(dist > epsilon_a && level == 2) dv_mode++;

            std::cout << "level : " << level << std::endl;

        }
        /* (3)delivery sign detection */
        else if(dv_mode == DETECTION_2){
        // else if(v2m.dv_mode == 2){
            //publish max velocity
            max_vel.data = s_v;
            max_vel_pub.publish(max_vel);   
            for(auto& sign: v2m.dv_signs.objects){
                if(sign.label == "B1" || sign.label == "B2" || sign.label == "B3"){
std::cout << "[2] I found " << sign.label << " !!" << std::endl;
                    if(not_use_pickup_detection == true){dv_goal_num = pickup_sign_number;}
                    if(dv_goal_num == sign.label.erase(0,1)){ // get sign number
                    // if("1" == sign.label.erase(0,1)){ // get sign number
std::cout << "[2] dv_goal_num: " << dv_goal_num << std::endl;
                        p_sign = sign.pose.position; // get sign position
                        int idx = getClosestWaypointIdx(p_sign);
std::cout << "[2] closest inx: " << idx << std::endl;
                        p_goal.x = vm_points.data[idx].ly; // get pick-up position
                        p_goal.y = vm_points.data[idx].bx;

                        visualization_msgs::Marker points_;
                        std_msgs::ColorRGBA c;

                        int id = 0;
                        c.r = 0.0;
                        c.g = 0.0;
                        c.b = 1.0;
                        c.a = 1.0;
                        geometry_msgs::Point pt;
                        pt.x = p_goal.x;
                        pt.y = p_goal.y;
                        pt.z = 0.0;
                        points_.header.frame_id = "map";
                        points_.header.stamp = ros::Time::now();
                        points_.ns = "points";
                        points_.action = visualization_msgs::Marker::ADD;
                        points_.pose.orientation.w = 1.0;
                        points_.lifetime = ros::Duration();
                        points_.id = id++;
                        points_.type = visualization_msgs::Marker::POINTS;
                        points_.scale.x = 1.0;
                        points_.scale.y = 1.0;

                        points_.color.r = 0.0;
                        points_.color.g = 0.0;
                        points_.color.b = 1.0;
                        points_.color.a = 1.0;

                        points_.points.push_back(pt);
                        points_.colors.push_back(c);
                        pub_stop_goal_b.publish(points_);

                        stop_once = false;
// std::cout << "before dv_mode++ (2)" << std::endl;
                        dv_mode++;  // next mode
                        break;
                    }
                }
            }
        }
        else if(dv_mode == DONE){
        //     MissionTrafficLight* dt;
        //     dt = new MissionTrafficLight(nh,"delivery+trafficLight");
        //     dt->processMission(v2m);
        //     delete dt;
        //     // v2m.dv_mode++;   //구역전환 메세지 안뜨면 넣어서 해보기 !!!
        // }

        // else if(v2m.dv_mode == 6){ 
        //     m.isMissionDone = true;
            std::cout << "isMissionDone~~" << std::endl;
        }
        else std::cout << "\'dv_mode\' is strange: " << dv_mode << std::endl;
        // float c_x = v2m.curpos.pose.position.x;
        // float c_y = v2m.curpos.pose.position.y;

        // std_msgs::Float64 d;
        // d.data = sqrt(pow((c_x-89.4124),2) + pow((c_y-39.0177),2));             // parkinglot
        // // d.data = sqrt(pow((c_x-90.0555),2) + pow((c_y-32.0468),2));             // parkinglot_end
        // //m.data = sqrt(pow((c_y-28.3824),2) + pow((c_x-32.8092),2));             // soongsil_straight

        // if (stop_once == false && d.data <= epsilon){
        //     stop_once = true;
        //     std::cout << "stop !!" << std::endl;
        //     waiting_start_time = ros::Time::now();
        //     msg.data = true;
        //     estop_pub.publish(msg);
            
        // }

        // if ((ros::Time::now() - waiting_start_time).toSec() >= max_waiting_time){
        //     msg.data = false;
        //     estop_pub.publish(msg);
        // }

        //         /* way2*/
        //    if(v2m.dv_round == 0) std::cout << "dv_round is not checked yet!!" << std::endl;
        //    if(v2m.dv_round == 1){

        //    }
        //    if(v2m.dv_round == 2){

        //    }
        return m;
    }
    void clean(){
        std_msgs::Float64 max_vel;
        max_vel.data = d_v;
        max_vel_pub.publish(max_vel);
        // mySetParam(nh, "/op_common_params/maxVelocity", max_vel.data);

        msg.data = false;
        estop_pub.publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher max_vel_pub;
    ros::Publisher estop_pub;
    ros::Publisher pub_stop_goal_a;
    ros::Publisher pub_stop_goal_b;
    // ros::Subscriber sub_goal_pose; //goal에서 멈출 때
    std_msgs::Float64 max_vel;
    std_msgs::Bool msg;
    double d_v;
    double s_v;
    ros::Time waiting_start_time;
    double max_waiting_time;
    int epsilon;
    int epsilon_a;
    bool stop_once;
    double prev_dist;
    int level;
    int dv_mode; // (0)pick-up sign detection, (1)pick-up, (2)driving, (3)delivery sign detection, (4)delivery (5)finished
    std::string pickup_sign_number;
    bool not_use_pickup_detection;
    geometry_msgs::Point p_goal;
    geometry_msgs::Point p_sign;
    std::string dv_goal_num;
};
// =================================================== 배달미션 끝 ===================================================

// =================================================== 정적장애물 미션 시작 ===================================================
class MissionObstacleAvoidance: public Mission{
public:
    MissionObstacleAvoidance(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name)
    {
        myGetParam(nh, to_string() + "/avoidance_velocity", a_v);
        myGetParam(nh, to_string() + "/driving_velocity", d_v);
        // myGetParam(nh, to_string() + "/avoidance_distance", avoidance_distance);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        // avoidance_distance_pub = nh.advertise<std_msgs::Float64>("op_planner_avoidance_distance", 10);
    }
    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        // lock.lock(); //0901
        //publish max velocity
        std_msgs::Float64 max_vel;
        max_vel.data = a_v;
        max_vel_pub.publish(max_vel);
        // mySetParam(nh, "/op_common_params/maxVelocity", max_vel.data);

        //publish avoidance distance
        // std_msgs::Float64 a_dist;
        // a_dist.data = avoidance_distance;
        // avoidance_distance_pub.publish(a_dist);

        Mission2Vehicle m;
        m.isMissionDone = false; //never done
        m.isAvoidMission = true;
        // lock.unlock(); //0901
        return m;
    }
    void clean(){
        std_msgs::Float64 max_vel;
        max_vel.data = d_v;
        max_vel_pub.publish(max_vel);
        // mySetParam(nh, "/op_common_params/maxVelocity", max_vel.data);
    }
private:
    ros::Publisher max_vel_pub;    
    // ros::Publisher avoidance_distance_pub;
    // double avoidance_distance;
    double a_v, d_v;
    std::mutex lock;    //0901
};
// =================================================== 정적장애물 미션 끝 ===================================================

// =================================================== 동적장애물 미션 시작 ===================================================
//jw
class MissionEmergencyObstacle: public Mission{
public:
    MissionEmergencyObstacle(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name), detected_e_obj(false)
    {
        std::vector<double> poly_raw;
        myGetParam(nh, to_string() + "/poly", poly_raw); //must have even length
        for(size_t i = 0 ; i < poly_raw.size() / 2; ++i){
            geometry_msgs::Point32 p;
            p.x = poly_raw[2*i];
            p.y = poly_raw[2*i + 1];
            road_poly.points.push_back(p);
        }// 이 미션의 poly구역을 저장받음

        // std::vector<double> poly_raw_endLine;
        // myGetParam(nh, to_string() + "/poly_endLine", poly_raw_endLine); //must have even length
        // for(size_t i = 0 ; i < poly_raw_endLine.size() / 2; ++i){
        //     geometry_msgs::Point32 p_endLine;
        //     p_endLine.x = poly_raw_endLine[2*i];
        //     p_endLine.y = poly_raw_endLine[2*i + 1];
        //     road_poly_endLine.points.push_back(p_endLine);
        // }// 이 미션의 끝라인 poly구역을 저장받음

        myGetParam(nh, to_string() + "/driving_velocity", driving_velocity);    //current미션을 지우고 다음 미션으로 넘길 때, clean()하면서 넘기는 속도   
        myGetParam(nh, to_string() + "/dynamic_obj_velocity", dynamic_obj_velocity);    //현재 미션 수행하면서 장애물 피하는 제한속도  
        // myGetParam(nh, to_string() + "/detection_limit_x", detection_limit_x);
        // myGetParam(nh, to_string() + "/detection_limit_y", detection_limit_y);
        myGetParam(nh, to_string() + "/detection_limit_radius", detection_limit_radius);
        
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);   //dynamic_obj_velocity를 넣어주게 됨
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        obj_info_pub = nh.advertise<visualization_msgs::MarkerArray>("e_obj_candidate", 10); 
        //rollout_num_pub = nh.advertise<std_msgs::Float64>("op_planner_rollout_num", 10);

        nhh = nh;        
        //score_deq.resize(MINIMUM_SCORE * 2);  //throw
        //std::fill_n(score_deq.begin(), MINIMUM_SCORE * 2, false); //throw
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        Mission2Vehicle m2v;
        m2v.isMissionDone = false;
        m2v.isEmergencyObstacle = true;

        /* publish intended velocity */
        std_msgs::Float64 vel_msg;
        vel_msg.data = dynamic_obj_velocity;
        max_vel_pub.publish(vel_msg);
        // mySetParam(nhh, "/op_common_params/maxVelocity", vel_msg.data);
// std::cout << "debug1: max_velocity: " << vel_msg.data << std::endl;
        
        /* reduce rollout num to 0 */
        // std_msgs::Float64 num_msg;
        // num_msg.data = 0;
        // rollout_num_pub.publish(num_msg);

        /* check if emergency obstacle emerged */
        if (false == v2m.obstacle_center_points_updated) return m2v;    //장애물 중심점 update
        else v2m.obstacle_center_points_updated = false;    //obstacle_center_points_updated 됐을 때ㅇㅇ 다시 false로 초기화

std::cout << "obstacle checking start !! \n" ;

        for(const auto& p : v2m.obstacle_center_points){
            car_x = v2m.curpos.pose.position.x; 
            car_y = v2m.curpos.pose.position.y;
            car_pose.x = car_x;
            car_pose.y = car_y;
            car_pose.z = v2m.curpos.pose.position.z;

            detected_e_obj = false;
            /* calculte distance between obstacle and car */
            double dist = std::sqrt(pow((p.x - car_x), 2) + pow((p.y - car_y), 2));
            
            // 장애물이 차의 관심영역 원 안에 있는지
            if (dist < detection_limit_radius){
                std::cout << "obstacle is in CIRCLE ROI of car @ \n" ;
                if (isForwardWaypoint(p, v2m)){
                    std::cout << "obstacle is infront of car @ \n" ;
                    // 장애물이 미션구역에 있는지
                    if(true == isPointInPolygon(p.x, p.y, road_poly)){
                            detected_e_obj = true;  //"동적 장애물이 앞에 있다"
                            std::cout << "I found e_obj @@ \n";
                            break;
                    }
                }
            }

            // //장애물이 차보다 앞에 있는지
            // if(p.x > car_x){ 
            //     std::cout << "obstacle is infront of car @ \n" ; 
            //     //장애물이 차의 관심영역에 있는지
            //     if(fabs(p.x - car_x) < detection_limit_x && fabs(p.y - car_y) < detection_limit_y){ 
            //         std::cout << "obstacle is in ROI of car @@ \n" ;
            //         //장애물이 미션구역에 있는지
            //         if(true == isPointInPolygon(p.x, p.y, road_poly)){
            //             detected_e_obj = true;  //"동적 장애물이 앞에 있다"
            //             std::cout << "I found e_obj @@@ \n";
            //             break;
            //         }
            //     }
            // }

            //debug2. 발견된 장애물의 위험정보를 벡터에 저장
            EobjInfo e;
            e.p = p;
            e.car_p = car_pose;
            e.isDangerous = detected_e_obj;
            e_obj_info_vec.push_back(e);
        }
            
        /* check if emergency obstacle emerged */
        if (true == detected_e_obj){
            msg.data = true;
            estop_pub.publish(msg);
        } 
        else {
            msg.data = false;
            estop_pub.publish(msg); //estop 해제
        } 
        /* check if car is at the end of mission polygon */
        // if (true == isPointInPolygon(car_x, car_y, road_poly_endLine)){
        //     msg.data = false;
        //     estop_pub.publish(msg); //걍 한번 더

        //     std_msgs::Float64 vel_msg;
        //     vel_msg.data = driving_velocity;
        //     max_vel_pub.publish(vel_msg); //미션끝나고 주행속도 발행
        //     std::cout << "finished velocity: " << vel_msg.data << std::endl;
        //     m2v.isMissionDone = true;  
        // }
        return m2v;
    }

    void clean(){  
        std_msgs::Float64 max_vel;
        max_vel.data = driving_velocity;
        max_vel_pub.publish(max_vel);
        std::cout << "clean velocity: " << max_vel.data << std::endl;

        msg.data = false;
        estop_pub.publish(msg);
    }

    void visualize(){
        int id = 0;
        visualization_msgs::MarkerArray markers_e;
        for(const auto& e : e_obj_info_vec){
            visualization_msgs::Marker marker_e;
            marker_e.header.frame_id = "map";
            marker_e.header.stamp = ros::Time::now();
            marker_e.ns = "parking_cars_candidate";
            marker_e.id = id++;
            marker_e.type = visualization_msgs::Marker::CUBE;
            marker_e.action = visualization_msgs::Marker::ADD;
            marker_e.pose.position.x  =   e.p.x;
            marker_e.pose.position.y  =   e.p.y;
            marker_e.pose.position.z  =   e.p.z;
            marker_e.pose.orientation.x = 0.0;
            marker_e.pose.orientation.y = 0.0;
            marker_e.pose.orientation.z = 0.0;
            marker_e.pose.orientation.w = 1.0;
            marker_e.scale.x = 0.8;
            marker_e.scale.y = 0.8;
            marker_e.scale.z = 1.5;
            marker_e.color.a = 0.8;            
            if (e.isDangerous) marker_e.color.r = 1.0;
            else marker_e.color.g = 1.0;
            marker_e.color.b = 0.0;
            //marker_e.text = std::to_string(e.gap_deg);
            marker_e.lifetime = ros::Duration(0.1);
            markers_e.markers.push_back(marker_e);
        }
        obj_info_pub.publish(markers_e);
    }

    struct EobjInfo{
        geometry_msgs::Point32 p;
        geometry_msgs::Point32 car_p;
        bool isDangerous;
    };
    
private:
    //ros::Time stop_start_time; //throw
    ros::NodeHandle nhh;
    ros::Publisher max_vel_pub;    
    ros::Publisher estop_pub;
    ros::Publisher obj_info_pub;
    ros::Publisher rollout_num_pub;
    double driving_velocity;
    // double detection_limit_x;
    // double detection_limit_y;
    double detection_limit_radius;
    double dynamic_obj_velocity;
    std_msgs::Bool msg;
    bool detected_e_obj;
    double car_x;
    double car_y;
    geometry_msgs::Point32 car_pose;
    // geometry_msgs::PoseStamped car_pose;
    std::mutex lock;
    //std::deque<bool> score_deq; //throw
    geometry_msgs::Polygon road_poly;
    geometry_msgs::Polygon road_poly_endLine;
    //tf::TransformListener listener; //throw
    std::vector<EobjInfo> e_obj_info_vec;
    //constexpr static size_t MINIMUM_SCORE = 3; //object must be detected or not detected more than MINIMUM_SCORE times successively //throw
    //constexpr static double ANGLE_GAP_LIMIT = 30; //deg //throw
};
// =================================================== 동적장애물 미션 끝 ===================================================

// =================================================== 주차미션 시작 ===================================================
class MissionParking : public Mission{
public:
    MissionParking(
        ros::NodeHandle& nh, 
        std::string mission_name): Mission(nh, mission_name), isEnd(false)
    {
         /* load params */
        // myGetParam(nh, to_string() + "/seeking_velocity", seeking_velocity);
        myGetParam(nh, to_string() + "/driving_velocity", driving_velocity);
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        parking_start_pub = nh.advertise<std_msgs::Bool>("/parking_mission_call", 10);
        parking_end_sub = nh.subscribe("parking_mission_end", 10, &MissionParking::parking_endCB, this);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        if(v2m.isStart){
            std_msgs::Float64 max_vel;  
            max_vel.data = 1.0;
            max_vel_pub.publish(max_vel);
            v2m.isStart = false;
        }
        Mission2Vehicle m;
        m.isParkingMission = true;
        m.isMissionDone = false;
        // m.isMissionDone = true;        /* parking mission is end*/
        if(isEnd == true){
            m.isParkingMission = false;
            m.isMissionDone = true;
            std_msgs::Float64 max_vel;  
            max_vel.data = driving_velocity;
            max_vel_pub.publish(max_vel);
            std::cout << "driving_velocity_pub : " << max_vel.data << std::endl;
            return m;
        }
        std_msgs::Bool msg;
        msg.data = true;
        parking_start_pub.publish(msg);
        return m;
    }

    void parking_endCB(const std_msgs::Bool msg){
        if(msg.data == true){
            isEnd = true;
        }
    }

    void clean() {
        std_msgs::Bool estop;
        estop.data = false;
        estop_pub.publish(estop);

        std_msgs::Float64 max_vel;
        max_vel.data = driving_velocity;
        max_vel_pub.publish(max_vel);
    }
private:
    ros::Publisher estop_pub;
    ros::Publisher max_vel_pub;
    ros::Publisher parking_start_pub; 
    ros::Subscriber parking_end_sub; 
    bool isEnd;
    double driving_velocity;
    // double seeking_velocity;
};
// =================================================== 주차미션 끝 ===================================================

