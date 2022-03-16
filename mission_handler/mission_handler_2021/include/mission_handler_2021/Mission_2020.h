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
#include <tf/transform_listener.h>
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



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Vehicle2Mission{
    Vehicle2Mission() :cur_light_updated(false), obstacle_center_points_updated(false) {} 
    geometry_msgs::PoseStamped curpos;
    double linear_velocity;

    traffic_light_msgs::Light cur_light;
    bool cur_light_updated;
    std::vector<geometry_msgs::Point32> obstacle_center_points;
    bool obstacle_center_points_updated;
};

struct Mission2Vehicle{
    Mission2Vehicle() : isMissionDone(false), isAvoidMission(false), isParkingMission(false) {}
    bool isMissionDone;
    bool isAvoidMission;
    bool isParkingMission;
    void clear() {isAvoidMission = false; isParkingMission = false;}
};

class Mission{
public:
    Mission(ros::NodeHandle& nh, std::string mission_name) : mission_name_(mission_name) {} //interface for constructor
    virtual Mission2Vehicle processMission(Vehicle2Mission& v2m) = 0;
    virtual void visualize() {}
    virtual void clean() {} //override if needed
    std::string to_string() const {return mission_name_;}
private:
    std::string mission_name_;
};

class MissionNone : public Mission{
public:
    MissionNone(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name) {}
    Mission2Vehicle processMission(Vehicle2Mission& v2m) {
        Mission2Vehicle m;
        m.isMissionDone = false; //never done
        return m;
    }
};
/* helper functions */
template <typename T>
static void myGetParam(ros::NodeHandle& nh, std::string param_name, T& src){
    if (!nh.getParam(param_name, src)) throw std::runtime_error(std::string() + "set " + param_name + "!!");
}

double getCriticalDistance(double v){
    constexpr static double CAR_TIP_MARGIN = 1.4;
    return CAR_TIP_MARGIN + 0.1767*std::pow(v, 2) + 0.3508 * v - 0.04;
    //return v + CAR_TIP_MARGIN;//0.055 * std::pow(v, 4) - 0.6128 * std::pow(v, 3) + 2.4117 * std::pow(v, 2) - 3.6048 * v + 1.95 + CAR_TIP_MARGIN + SAFETY_MARGIN;
}

class MissionObstacleAvoidance: public Mission{
public:
    MissionObstacleAvoidance(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name)
    {
        myGetParam(nh, to_string() + "/avoidance_velocity", a_v);
        myGetParam(nh, to_string() + "/driving_velocity", d_v);
        myGetParam(nh, to_string() + "/avoidance_distance", avoidance_distance);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        avoidance_distance_pub = nh.advertise<std_msgs::Float64>("op_planner_avoidance_distance", 10);
    }
    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        //publish max velocity
        std_msgs::Float64 max_vel;
        max_vel.data = a_v;
        max_vel_pub.publish(max_vel);

        //publish avoidance distance
        std_msgs::Float64 a_dist;
        a_dist.data = avoidance_distance;
        avoidance_distance_pub.publish(a_dist);

        Mission2Vehicle m;
        m.isMissionDone = false; //never done
        m.isAvoidMission = true;
        return m;
    }
    void clean(){
        std_msgs::Float64 max_vel;
        max_vel.data = d_v;
        max_vel_pub.publish(max_vel);
    }
private:
    ros::Publisher max_vel_pub;    
    ros::Publisher avoidance_distance_pub;
    double avoidance_distance;
    double a_v, d_v;
};

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
        }
        myGetParam(nh, to_string() + "/driving_velocity", driving_velocity);
        myGetParam(nh, to_string() + "/dynamic_obj_velocity", dynamic_obj_velocity);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        obj_info_pub = nh.advertise<visualization_msgs::MarkerArray>("e_obj_candidate", 10);
        
        score_deq.resize(MINIMUM_SCORE * 2);
        std::fill_n(score_deq.begin(), MINIMUM_SCORE * 2, false);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        Mission2Vehicle m2v;
        m2v.isMissionDone = false;

        std_msgs::Float64 vel_msg;
        vel_msg.data = dynamic_obj_velocity;
        max_vel_pub.publish(vel_msg);

        /* check if emergency obstacle emerged */
        bool is_obstacle_detected = false;
        if (false == v2m.obstacle_center_points_updated) return m2v;
        else v2m.obstacle_center_points_updated = false;

        //get car's yaw
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform); //Time(0) finds the latest tf
        }
        catch (tf::TransformException ex){
            ROS_ERROR("no tf from map to baselink : %s",ex.what());
        }
        double yaw_car_rad = tf::getYaw(transform.getRotation());

        e_obj_info_vec.resize(0);
        for(const auto& p : v2m.obstacle_center_points){

            //calc angle between (car's angle) with (obstacle's xy - car's xy)
            double car_x = v2m.curpos.pose.position.x; double car_y = v2m.curpos.pose.position.y;

            double yaw_obstacle_minus_car_rad = std::atan2(p.y - car_y, p.x - car_x);
            double yaw_gap_deg = std::fabs(yaw_car_rad - yaw_obstacle_minus_car_rad) * 180 / M_PI;

            //consider emergency obstacle to be emerged if point is in front of the car
            bool isDangerous =  (yaw_gap_deg < ANGLE_GAP_LIMIT) && (true == isPointInPolygon(p.x, p.y, road_poly));
            if (isDangerous && (false == is_obstacle_detected)) is_obstacle_detected = true;

            //debug
            EobjInfo e;
            e.p = p;
            e.isDangerous = isDangerous;
            e.gap_deg = yaw_gap_deg;
            e_obj_info_vec.push_back(e);
        }

        /* scoring */
        score_deq.push_back(is_obstacle_detected);
        score_deq.pop_front();

        /* decide if obj is in front of vehicle */
        bool isObjExist = std::count(score_deq.begin(), score_deq.end(), true) >= MINIMUM_SCORE ;
        if (false == detected_e_obj){
            if (isObjExist){
                std_msgs::Bool msg;
                msg.data = true;
                estop_pub.publish(msg);
                detected_e_obj = true;
                stop_start_time = ros::Time::now();//tw
            }
        } else {
            if (isObjExist){
                std_msgs::Bool msg;
                msg.data = true;
                estop_pub.publish(msg);

                if ((ros::Time::now() - stop_start_time).toSec() >= 30){
                    m2v.isMissionDone = true;
                }
            }
            else {
                std_msgs::Bool msg;
                msg.data = false;
                estop_pub.publish(msg);

                std_msgs::Float64 vel_msg;
                vel_msg.data = driving_velocity;
                max_vel_pub.publish(vel_msg);

                m2v.isMissionDone = true;
            }   
        }
        return m2v;
    }

    void clean(){
        std_msgs::Float64 max_vel;
        max_vel.data = driving_velocity;
        max_vel_pub.publish(max_vel);

        std_msgs::Bool msg;
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
            marker_e.text = std::to_string(e.gap_deg);
            marker_e.lifetime = ros::Duration(0.1);
            markers_e.markers.push_back(marker_e);
        }
        obj_info_pub.publish(markers_e);
    }

    struct EobjInfo{
        geometry_msgs::Point32 p;
        bool isDangerous;
        double gap_deg;
    };

    
private:
    ros::Time stop_start_time;
    ros::Publisher max_vel_pub;    
    ros::Publisher estop_pub;
    ros::Publisher obj_info_pub;
    double driving_velocity;
    std::deque<bool> score_deq;
    geometry_msgs::Polygon road_poly;
    bool detected_e_obj;
    tf::TransformListener listener;
    double dynamic_obj_velocity;
    std::vector<EobjInfo> e_obj_info_vec;

    constexpr static size_t MINIMUM_SCORE = 3; //object must be detected or not detected more than MINIMUM_SCORE times successively
    constexpr static double ANGLE_GAP_LIMIT = 30; //deg
};

//jw
class MissionDelivery: public Mission{
public:
    MissionDelivery(ros::NodeHandle& nh, std::string mission_name) : Mission(nh, mission_name)
    {
        myGetParam(nh, to_string() + "/driving_velocity", d_v);
        myGetParam(nh, to_string() + "/epsilon_goal_current", epsilon);
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        //publish max velocity
        std_msgs::Float64 max_vel;
        max_vel.data = d_v;
        max_vel_pub.publish(max_vel);

        float c_x = v2m.curpos.pose.position.x;
        float c_y = v2m.curpos.pose.position.y;

        std_msgs::Float64 d;
        d.data = sqrt(pow((c_y-27.5905),2) + pow((c_x-88.8855),2));             // parkinglot
        //m.data = sqrt(pow((c_y-28.3824),2) + pow((c_x-32.8092),2));             // soongsil_straight
        if (stop_once == false && d.data <= epsilon){
            lock.lock();
            stop_once = true;
            std::cout << "stop !!" << std::endl;
            msg.data = true;
            estop_pub.publish(msg);
            sleep(5);       // 5초정지
            msg.data = false;
            estop_pub.publish(msg);
            lock.unlock();
        }
        else{
            lock.lock();
            msg.data = false;
            estop_pub.publish(msg);
            lock.unlock();
        }

        Mission2Vehicle m;
        m.isMissionDone = false; //never done
        return m;
    }
    void clean(){
        std_msgs::Float64 max_vel;
        max_vel.data = d_v;
        max_vel_pub.publish(max_vel);

        msg.data = false;
        estop_pub.publish(msg);
    }

private:
    ros::Publisher max_vel_pub;
    ros::Publisher estop_pub;
    ros::Subscriber curposeSub;
    ros::Subscriber sub_goal_pose;
    double d_v;
    int epsilon;
    std::mutex lock;
    std_msgs::Bool msg;
    bool stop_once;
};


class MissionParking : public Mission{
public:
    MissionParking(
        ros::NodeHandle& nh, 
        std::string mission_name):
            Mission(nh, mission_name), found_goal_area(false), found_closest_point_in_poly(false), parking_action_client("move_base", true)
    {
         /* load params */
        myGetParam(nh, to_string() + "/seeking_velocity", seeking_velocity);
        myGetParam(nh, to_string() + "/driving_velocity", driving_velocity);
        myGetParam(nh, to_string() + "/parking_time", parking_time);

        int n_goal;
        myGetParam(nh, to_string() + "/n_goal", n_goal);

        std::vector<double> goal_pose_vec_raw, goal_area_vec_raw;
        myGetParam(nh, to_string() + "/goal_pose", goal_pose_vec_raw);
        myGetParam(nh, to_string() + "/goal_area", goal_area_vec_raw);
        for(int i = 0 ; i < n_goal; ++i){
            geometry_msgs::PoseStamped goal, recovery;
            goal.header.frame_id = "map";
            goal.pose.position.x        = goal_pose_vec_raw[8*i];
            goal.pose.position.y        = goal_pose_vec_raw[8*i + 1];
            goal.pose.orientation.z     = goal_pose_vec_raw[8*i + 2];
            goal.pose.orientation.w     = goal_pose_vec_raw[8*i + 3];
            recovery.header.frame_id = "map";
            recovery.pose.position.x    = goal_pose_vec_raw[8*i + 4];
            recovery.pose.position.y    = goal_pose_vec_raw[8*i + 5];
            recovery.pose.orientation.z = goal_pose_vec_raw[8*i + 6];
            recovery.pose.orientation.w = goal_pose_vec_raw[8*i + 7];
            goal_pose_vec.push_back(goal); recovery_pose_vec.push_back(recovery);

            geometry_msgs::Polygon poly;
            for(int j = 0 ; j < 4; ++j){
                geometry_msgs::Point32 p;
                p.x = goal_area_vec_raw[8*i + 2*j];
                p.y = goal_area_vec_raw[8*i + 2*j + 1];
                poly.points.push_back(p);
            }
            goal_area_poly_vec.push_back(poly);
        }

        //wait for move_base server
        while(!parking_action_client.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        std::string goal_parking_topic;
        if (!nh.getParam("erp42_parking_goal_topic", goal_parking_topic)) 
            throw std::runtime_error("set erp42_parking_goal_topic!");
        max_vel_pub = nh.advertise<std_msgs::Float64>("op_planner_max_velocity", 10);
        estop_pub = nh.advertise<std_msgs::Bool>("erp42_estop", 10);
        goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_parking_topic, 10);
        state_pub = nh.advertise<visualization_msgs::Marker>("parking_state", 10);
        parking_cars_candidate_pub = nh.advertise<visualization_msgs::MarkerArray>("parking_cars_candidate", 10);
        goal_area_vec_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("parking_areas_debug", 10);

        state = STATE::SEEKING;

        //init empty place searching members
        parking_occupied_check_vec.resize(goal_area_poly_vec.size());
        std::fill_n(parking_occupied_check_vec.begin(), parking_occupied_check_vec.size(), PARKING_OCCUPIED_COUNTER);
        //if any parking place is perceived as empty place more than 'PARKING_OCCUPIED_COUNTER' times, 
        //car thinks that place as empty

        closest_dist_memory.resize(goal_area_poly_vec.size());
        std::fill_n(closest_dist_memory.begin(), closest_dist_memory.size(), MAX_DIST_TO_PARKING_POLY);
        closest_point_in_poly_idx_vec.resize(goal_area_poly_vec.size());
    }

    std::pair<bool, int> updateAndFindParkingPlace(Vehicle2Mission& v2m)
    /* update value : (bool found, int empty_place_idx) */
    {
        //executed only once
        //find closest point in polygon 
        if (false == found_closest_point_in_poly){
            double closest_dist_to_poly = MAX_DIST_TO_PARKING_POLY;//10000m
            
            for(size_t idx_poly = 0 ; idx_poly < goal_area_poly_vec.size(); ++idx_poly){
                auto& poly = goal_area_poly_vec[idx_poly];
                for(size_t idx_point = 0; idx_point < poly.points.size(); ++idx_point){
                    auto& p = poly.points[idx_point];
                    double x = p.x; double y = p.y;
                    double dist = std::sqrt(
                        std::pow(p.x - v2m.curpos.pose.position.x, 2) +
                        std::pow(p.y - v2m.curpos.pose.position.y, 2)
                    );
                    if (dist < closest_dist_to_poly) {
                        closest_dist_to_poly = dist;
                        closest_point_in_poly_idx_vec[idx_poly] = idx_point;
                    }
                }
            }
            found_closest_point_in_poly = true;
        }//debug done
        
        if (v2m.obstacle_center_points_updated) //works only when "transformed_obb_arr" is published
            v2m.obstacle_center_points_updated = false;
        else return std::make_pair(false, -1);
        parking_cars_candidate = v2m.obstacle_center_points;

        //update empty place          
        for(size_t idx_poly = search_start_idx ; idx_poly < goal_area_poly_vec.size(); ++idx_poly){
            auto & parking_poly = goal_area_poly_vec[idx_poly];

            //extract closest distance from polygon
            //in parking mission, every polygon is closer then MAX_DIST_TO_PARKING_POLY. SO it's ok
            auto& closest_point_in_poly = parking_poly.points[closest_point_in_poly_idx_vec[idx_poly]];
            double closest_dist_to_poly = std::sqrt(
                std::pow(closest_point_in_poly.x - v2m.curpos.pose.position.x, 2) +
                std::pow(closest_point_in_poly.y - v2m.curpos.pose.position.y, 2)
            );
            
            //if closest_dist_to_poly becomes bigger, we can think the car passes the polygon
            //so we can update search_start_idx

            if (closest_dist_to_poly > (closest_dist_memory[idx_poly] + 0.2)) {
                search_start_idx = idx_poly + 1;
                continue; //start checking from next poly
            }
            
            //update closest distance
            closest_dist_memory[idx_poly] = closest_dist_to_poly;

            //check if vehicle is in poly when it's close
            if (closest_dist_to_poly < PARKING_SEARCH_START_DISTANCE){ //search start
                bool is_car_detected_in_cur_poly = false;
                for(const auto& p : v2m.obstacle_center_points){
                    double x_car_cand = p.x;
                    double y_car_cand = p.y;
                    if (true == isPointInPolygon(x_car_cand, y_car_cand, parking_poly)){
                        is_car_detected_in_cur_poly = true;
                        break;
                    }
                }
                if (false == is_car_detected_in_cur_poly)
                    parking_occupied_check_vec[idx_poly]--; //see comment in constructor
                else parking_occupied_check_vec[idx_poly]++ ;
            } else break;//since dist_to_parking_poly becomes bigger, we can skip extra searching if one poly
            //is far from car
        }
        //find empty place to park
        for(size_t i = 0 ; i < parking_occupied_check_vec.size(); ++i){
            if (parking_occupied_check_vec[i] <= 0) return std::make_pair(true, i);//found empty place
        }
        return std::make_pair(false, -1);//not found empty place
    }

    void visualize(){
        //publish state marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "parking";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z  =   1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 2;
        marker.color.a = 1.0;
        marker.color.r = 0.9;
        marker.color.g = 0.1;
        marker.color.b = 0.0;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
    
        marker.text = state_string;
        state_pub.publish(marker);

        //publish parking car candidate positions
        visualization_msgs::MarkerArray marker_parking_cars;
        int id = 0;
        for(const auto& p : parking_cars_candidate){
            visualization_msgs::Marker marker_car;
            marker_car.header.frame_id = "map";
            marker_car.header.stamp = ros::Time::now();
            marker_car.ns = "parking_cars_candidate";
            marker_car.id = id++;
            marker_car.type = visualization_msgs::Marker::SPHERE;
            marker_car.action = visualization_msgs::Marker::ADD;
            marker_car.pose.position.x  =   p.x;
            marker_car.pose.position.y  =   p.y;
            marker_car.pose.position.z  =   p.z;
            marker_car.pose.orientation.x = 0.0;
            marker_car.pose.orientation.y = 0.0;
            marker_car.pose.orientation.z = 0.0;
            marker_car.pose.orientation.w = 1.0;
            marker_car.scale.x = 2;
            marker_car.scale.y = 2;
            marker_car.scale.z = 2;
            marker_car.color.a = 0.8;
            marker_car.color.r = 0.1;
            marker_car.color.g = 0.9;
            marker_car.color.b = 0.0;
            marker_parking_cars.markers.push_back(marker_car);
        }
        parking_cars_candidate_pub.publish(marker_parking_cars);

        //publish goal areas            
        jsk_recognition_msgs::PolygonArray polys;
        polys.header.frame_id = "map";
        polys.header.stamp = ros::Time::now();
        for(auto& poly : goal_area_poly_vec){
            geometry_msgs::PolygonStamped poly_real;
            poly_real.header.frame_id = "map";
            poly_real.header.stamp = ros::Time::now();
            poly_real.polygon = poly;
            polys.polygons.push_back(poly_real);
        }
        goal_area_vec_pub.publish(polys);
    }

    Mission2Vehicle processMission(Vehicle2Mission& v2m){
        Mission2Vehicle m;
        m.isMissionDone = false;

        double dt;
        std_msgs::Float64 max_vel;
        std_msgs::Bool estop;
        
        if (state == STATE::SEEKING){
            state_string = "[Parking]Seeking";
            //set max vel
            max_vel.data = seeking_velocity;
            max_vel_pub.publish(max_vel);

            //seeking logic....
            auto [found, empty_place_idx] = updateAndFindParkingPlace(v2m);
            if (false == found) return m;
            found_goal_area = true;
            target_goal_idx = empty_place_idx;
            //seeking logic done...

            //publish goal to navigation stack...
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose = goal_pose_vec[target_goal_idx];
            parking_action_client.sendGoal(goal);
            ROS_WARN("sending parking goal to navigation stack...");
            //publish goal to navigation stack done
            state = STATE::PARKING;
            m.isParkingMission = true;
            ROS_WARN("switch to STATE::PARKING");
        }
        else if (state == STATE::PARKING){
            state_string = "[Parking]Parking";
            m.isParkingMission = true;        
            //wait for navigation stack's completion notifying
            if (parking_action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) return m;
            //wait for navigation stack's completion notifying end...
            estop.data = true;
            estop_pub.publish(estop);
            parking_start_time = ros::Time::now();
            state = STATE::WAITING;
            ROS_WARN("switch to STATE::WAITING");
        }
        else if (state == STATE::WAITING){
            state_string = "[Parking]Waiting";
            m.isParkingMission = true;        
            dt = (ros::Time::now() - parking_start_time).toSec();
            ROS_WARN("dt : %lf", dt);
            if (dt < parking_time) return m;
            estop.data = false;
            estop_pub.publish(estop);
            state = STATE::RECOVERING;
            //send recovery goal
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose = recovery_pose_vec[target_goal_idx];
            parking_action_client.sendGoal(goal);
            ROS_WARN("switch to STATE::RECOVERING");
        }
        else if (state == STATE::RECOVERING){
            state_string = "[Parking]Recovering";
            m.isParkingMission = true;
            //wait for navigation stack's completion notifying
            if (parking_action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) return m;
            //wait for navigation stack's completion notifying end...
            m.isParkingMission = false;
            m.isMissionDone = true;
            
            //switch to original speed
            std_msgs::Float64 max_vel;
            max_vel.data = driving_velocity;
            max_vel_pub.publish(max_vel);
        }
        state_string = "[Parking]Done";
        visualize();
        return m;
    }
    void clean() {
        std_msgs::Bool estop;
        estop.data = false;
        estop_pub.publish(estop);

        std_msgs::Float64 max_vel;
        max_vel.data = driving_velocity;
        max_vel_pub.publish(max_vel);

        parking_action_client.cancelAllGoals();
    }
private:
    ros::Publisher goal_pose_pub;
    ros::Publisher estop_pub;
    ros::Publisher max_vel_pub; 
    ros::Publisher state_pub;
    ros::Publisher goal_area_vec_pub;
    ros::Publisher parking_cars_candidate_pub;    

    std::vector<geometry_msgs::PoseStamped> goal_pose_vec, recovery_pose_vec;
    std::vector<geometry_msgs::Polygon> goal_area_poly_vec;
    double seeking_velocity, parking_time, driving_velocity;
    enum class STATE {SEEKING, PARKING, WAITING, RECOVERING};
    STATE state;
    int target_goal_idx;
    ros::Time parking_start_time;
    bool updated_parking_obb_box;
    std::vector<geometry_msgs::Point32> parking_cars_candidate;
    MoveBaseClient parking_action_client;
    std::string state_string;
    bool found_goal_area;

    //empty place searching members
    constexpr static int PARKING_OCCUPIED_COUNTER = 3;
    std::vector<double> closest_dist_memory; // to update search_start_idx
    std::vector<int> parking_occupied_check_vec;
    constexpr static double MAX_DIST_TO_PARKING_POLY = 10000; //10km
    constexpr static double PARKING_SEARCH_START_DISTANCE = 5;
    size_t search_start_idx = 0;
    bool found_closest_point_in_poly;
    std::vector<size_t> closest_point_in_poly_idx_vec;

};


class MissionTrafficLight: public Mission{
public:
    MissionTrafficLight(ros::NodeHandle& nh, std::string mission_name): 
        Mission(nh, mission_name), isDone(false), isRed(false)
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
        if (false == isDone)  {
            if (v2m.cur_light_updated) v2m.cur_light_updated = false;
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
            //ROS_WARN("dist to stopline, critical + safety : %lf %lf", dist_to_stopline, getCriticalDistance(v2m.linear_velocity) + safety_margin);
            if ((dist_to_stopline < (getCriticalDistance(v2m.linear_velocity) + safety_margin)) || isRed){
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
                    b.data = true;
                    estop_pub.publish(b);    
                }
            }

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
    double traffic_velocity;
    double driving_velocity;
    double safety_margin;
    TrafficLightState target_light_state;
    int n_hole_;
    int section_;
    TrafficLightRule traffic_light_rule;
    double max_waiting_time;
    ros::Time waiting_start_time;

    //visualizer 
    ros::Publisher stopline_pub;
    ros::Publisher traffic_light_pub;
    visualization_msgs::Marker stopline_marker;
    std::string cur_light_string;
    bool isRed;
    
        
};
