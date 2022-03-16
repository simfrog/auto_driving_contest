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

#include <ros/ros.h>
#include <mission_handler_2021/Mission.h>
#include <mission_handler_2021/IsPointInPolygon.h>
#include <vector>
#include <mutex>
#include <thread>
//#include <jsk_recognition_msgs/PolygonArray.h> //내가 추가
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_msgs/DetectedObjectArray.h> //add by hc 
#include <autoware_msgs/CloudClusterArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

struct MissionInfo{
    int mission_identifier;
    geometry_msgs::Polygon area;
    Mission* mission_ptr;
};

class MissionManager{
public:
    void genPolyMsg(geometry_msgs::Polygon& poly_ret, std::vector<double> ppoly, const char* name){
        static int label;
        static int ll;

        polyArr.header.frame_id = "map";
        polyArr.header.seq = 1;
        polyArr.header.stamp = ros::Time::now(); //일단...

        geometry_msgs::PolygonStamped poly;

        poly.header.frame_id = "map";
		poly.header.seq = 1;
		poly.header.stamp = ros::Time::now();

        int size = ppoly.size();
        int x,y;
        for(int i = 0; i < size; i += 2){
            geometry_msgs::Point32 p;
            p.x = ppoly[i];
			p.y = ppoly[i+1];
			p.z = 0;
            x = p.x;
            y = p.y;
            poly.polygon.points.emplace_back(p);
        }

        polyArr.polygons.emplace_back(poly);
        polyArr.labels.emplace_back(label++);
        polyArr.likelihood.emplace_back(ll);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z=1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
//      marker.scale.x = 10;
//      marker.scale.y = 10;
        marker.scale.z = 5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.pose.position.x = x;
        marker.pose.position.y = y;

        marker.text = name;
        markera.markers.emplace_back(marker);

        //return value
        poly_ret = poly.polygon;
    }

    void initMissions(){
        //must be aligned with .yaml file
        auto createMissionHandler = [&](std::string mission_name, int mission_type)-> Mission* {
            if (mission_type == 0) //parking
                return new MissionParking(nh, mission_name);
            else if (mission_type == 1) // obstacle_avoidance
                return new MissionObstacleAvoidance(nh, mission_name);
            else if (mission_type == 2) // traffic light
                return new MissionTrafficLight(nh, mission_name);
            else if (mission_type == 3) //emergency obstacle
                return new MissionEmergencyObstacle(nh, mission_name);
            else if (mission_type == 4) //delivery
                return new MissionDelivery(nh, mission_name);
            else
                throw std::runtime_error(std::string() + "unknown mission type : " + std::to_string(mission_type));
        };

        std::vector<std::string> mission_names;
        std::vector<int> mission_types;

        if (!nh.getParam("mission_names", mission_names)) throw std::runtime_error("set mission_names!");
        if (!nh.getParam("mission_types", mission_types)) throw std::runtime_error("set mission_types!");

        size_t n_mission = mission_names.size();

        //init MissionNone
        MissionInfo info;
        info.mission_identifier = 0;
        info.mission_ptr = new MissionNone(nh, "MissionNone");
        mission_infos.push_back(info);

        //init Missions
        std::vector<double> poly_vec;
        geometry_msgs::Polygon poly;
        for(size_t i = 0 ; i < n_mission; ++i){
            std::string mission_name = mission_names[i];
            int mission_type = mission_types[i];
    		if(!nh.getParam(mission_name + "/poly", poly_vec)) throw std::runtime_error( std::string() + "set " + mission_name + "/poly!!!");
            genPolyMsg(poly, poly_vec, mission_name.c_str());

            info.mission_identifier = mission_infos.back().mission_identifier + 1;
            info.mission_ptr = createMissionHandler(mission_name, mission_type);
            info.area = poly;

            mission_infos.push_back(info);
        }
        
        //init mission handling variables
        mission_identifier_cur = 0;
        mission_none = &mission_infos[0]; 
        cur_mission = &mission_infos[mission_identifier_cur];
    }

    MissionManager(){
        // 210722 추가
        // b_x = 0;
        // b_y = 0;
        // b_z = 0;

        //init mission instances
        initMissions();
        isStart = true;
        // if(!nh.getParam("publish_object_topic_name", publish_objects_topic_name_))                
        //     throw std::runtime_error("set publish_object_topic_name");
        //init ros members
        //pub
        poly_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("mission_poly_visualizer",10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("mission_text_visualizer",10);
        // objects_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects",10);
        objects_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects_erp42",10); //jw
        erp42_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd_erp42", 10);////////////debug
        // tracked_obj_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_tracked_obj_viz", 10);
        //sub
        curposeSub = nh.subscribe("current_pose", 10, &MissionManager::curposeCB, this);
        curTwistSub = nh.subscribe("current_velocity", 10, &MissionManager::curTwistCB, this);
        // objects_sub = nh.subscribe("detection/tracked_objects", 10, &MissionManager::objects_infoCB,this);//need to change autoware code
        traffic_light_sub = nh.subscribe("traffic_light_erp42", 10, &MissionManager::traffic_lightCB, this);
        twist_cmd_sub = nh.subscribe("twist_cmd", 10, &MissionManager::twist_cmdCB, this);
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &MissionManager::cmd_velCB, this);
        twist_purepursuit_sub = nh.subscribe("twist_raw_pure_pursuit", 10, &MissionManager::twist_cmd_purepursuitCB, this);
        objects_center_points_sub = nh.subscribe("tracked_objects", 10, &MissionManager::obstacle_center_points_CB_quadtree, this);
        clouds_center_points_sub = nh.subscribe("/detection/lidar_detector/cloud_clusters", 10, &MissionManager::clouds_center_points_CB_quadtree, this); //실험해보기
        // objects_center_points_sub = nh.subscribe("transformed_obb_boxes", 10, &MissionManager::obstacle_center_points_CB_quadtree, this);
        vm_points_sub = nh.subscribe("/vector_map_info/point", 1, &MissionManager::callbackGetVMPoints,  this);
        traffic_sign_sub = nh.subscribe("/fusion_objects", 10, &MissionManager::delivery_signCB, this);
        
        //210722_jw
        // sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &MissionManager::callbackGetGoalPose, this); 
 
        //init thread
        std::thread(&MissionManager::missionProcesingThread, this).detach();
        std::thread(&MissionManager::missionAreaDebugThread, this).detach();
    }
    // 210722_jw
    // void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
    // {
    //     //geometry_msgs::Point pt(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //     //goal_compare_base = pt;
    //     b_x = msg->pose.position.x;
    //     b_y = msg->pose.position.y; 
    //     b_z = msg->pose.position.z;
    // }

    void missionAreaDebugThread(){
        ros::Rate r(10);
        while(ros::ok()){
            v2m_lock.lock();//I'm lazy to make new lock for each thread..
            poly_pub.publish(polyArr);
            marker_pub.publish(markera);
            cur_mission->mission_ptr->visualize();
            v2m_lock.unlock();
            r.sleep();
        }
    }

    void missionProcesingThread(){
        ros::Rate loop_rate(50);
        //std::cout << "2"  << std::endl;

        /* reset mission processing variables */
        //not use mission handler
        bool not_use_mission_handler;
        if (!nh.getParam("not_use_mission_handler", not_use_mission_handler)) throw std::runtime_error("set not_use_mission_handler!");
        
        //std::cout << "3"  << std::endl;
        //mission_activate_ary
        std::vector<bool> mission_activate_ary_wo_none;
        if (!nh.getParam("mission_activate_ary", mission_activate_ary_wo_none)) throw std::runtime_error("set mission_activate_ary!");
        std::vector<bool> mission_activate_ary;
        mission_activate_ary.push_back(true); //mission_none -> .yaml file does not include flag for missionNone
        mission_activate_ary.insert(mission_activate_ary.end(), mission_activate_ary_wo_none.begin(), mission_activate_ary_wo_none.end());

        //isMissionDoneVec
        std::vector<bool> isMissionDoneVec;
        isMissionDoneVec.resize(mission_infos.size());
        std::fill_n(isMissionDoneVec.begin(), isMissionDoneVec.size(), false);

        while(ros::ok()){
            if (not_use_mission_handler){
        //std::cout << "4"  << std::endl;
                loop_rate.sleep();
                continue;
            }
            
            v2m_lock.lock();
            m2v_lock.lock();
            
            //process mission
            if (mission_activate_ary[mission_identifier_cur]){
            // std::cout << "aa"  << std::endl;
                if(isStart){
                    isStart = false;
                    v2m.isStart = true;
                }
                m2v = cur_mission->mission_ptr->processMission(v2m);
            }

            //find current mission
            //assume there's no intersaction between mission area
            int mission_identifier_localizing = 0;
        //std::cout << "6"  << std::endl;
            for(size_t i = 0 ; i < mission_infos.size(); ++i){
                if (true == isPointInPolygon(v2m.curpos.pose.position.x, 
                    v2m.curpos.pose.position.y, 
                    mission_infos[i].area))
                {
                    if(isMissionDoneVec[i] == false)
                        mission_identifier_localizing = (int)i;
                    break;
                }
            }
            
            //find next mission
            int mission_idx_next = 0;
            bool changed = false;
        //std::cout << "7"  << std::endl;
            if (mission_identifier_localizing != mission_identifier_cur) {
                changed = true;
                mission_idx_next = mission_identifier_localizing; //default is mission none
            }
        //std::cout << "8"  << std::endl;
            if ((cur_mission != mission_none) && m2v.isMissionDone) {
                mission_idx_next = 0; //mission none
                changed = true;
            }

            //clean current mission and change to next
        //std::cout << "9"  << std::endl;
            if (changed){
                // if (mission_idx_next == 0 && mission_identifier_cur == 1) isMissionDoneVec[mission_identifier_cur] = true;

                cur_mission->mission_ptr->clean();                
                mission_identifier_cur = mission_idx_next;
                cur_mission = &mission_infos[mission_identifier_cur];

                    /* delivery_mission way2 */
                // if(cur_mission->mission_ptr->to_string() == "pick-up" || cur_mission->mission_ptr->to_string() == "delivery") v2m.dv_round++; //jw

                m2v.clear();
                ROS_WARN("switch to mission[%d] : %s", mission_identifier_cur, cur_mission->mission_ptr->to_string().c_str());
            }
            
            m2v_lock.unlock();
            v2m_lock.unlock(); //no need to lock v2m anymore

            loop_rate.sleep();
        }
        //std::cout << "10"  << std::endl;
    }
    void curTwistCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        //std::cout << "curTwistCB" <<std::endl;
        v2m_lock.lock();
        v2m.linear_velocity = ptr->twist.linear.x;
        v2m_lock.unlock();
    }

    void curposeCB(const geometry_msgs::PoseStampedConstPtr& msg){
        //std::cout << "curposeCB" <<std::endl;
        v2m_lock.lock();        
        v2m.curpos = *msg;
        v2m_lock.unlock();
    }   

    visualization_msgs::MarkerArray genVizMsg(const autoware_msgs::DetectedObjectArray& d_obj){
        visualization_msgs::MarkerArray m_ary;
        
        int id = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "nsns";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        for(const auto& p : d_obj.objects.back().convex_hull.polygon.points){
            geometry_msgs::Point p_msg;
            p_msg.x = p.x;
            p_msg.y = p.y;
            marker.points.push_back(p_msg);
        }
        marker.scale.x = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        m_ary.markers.push_back(marker);
        return m_ary;
    }

    //traffic light
    void traffic_lightCB(const traffic_light_msgs::LightConstPtr& ptr){
        //std::cout << "traffic_lightCB" <<std::endl;
        v2m_lock.lock();
        v2m.cur_light = *ptr;
        v2m.cur_light_updated = true;
        v2m_lock.unlock();
    }

    void twist_cmdCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        //std::cout << "twist_cmdCB" <<std::endl;
        m2v_lock.lock();
        bool isParkingMission = m2v.isParkingMission;
        bool isAvoidMission = m2v.isAvoidMission;
        m2v_lock.unlock();

        if (false == isParkingMission && false == isAvoidMission) publish_erp42_twist_cmd(*ptr);
    }

    void cmd_velCB(const geometry_msgs::TwistConstPtr& ptr){
        //std::cout << "cmdvelCB" <<std::endl;
        m2v_lock.lock();
        bool isParkingMission = m2v.isParkingMission;
        m2v_lock.unlock();
        if (true == isParkingMission) {
            geometry_msgs::TwistStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.twist = *ptr;
            publish_erp42_twist_cmd(msg);
        }
    }
    void twist_cmd_purepursuitCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        //std::cout << "twist_cmd_purepursuitCB" <<std::endl;
        m2v_lock.lock();
        bool isAvoidMission = m2v.isAvoidMission;
        m2v_lock.unlock();

        if (true == isAvoidMission) publish_erp42_twist_cmd(*ptr);
    }

    void publish_erp42_twist_cmd(const geometry_msgs::TwistStamped& msg){       
        //std::cout << "publish_erp42_twist_cmd" <<std::endl;
        erp42_twist_pub.publish(msg);
    }

    void obstacle_center_points_CB_quadtree(const autoware_msgs::DetectedObjectArrayConstPtr& ptr){  
        // std::cout << "obstacle_center_points_CB_quadtree" <<std::endl;
        objects_pub.publish(ptr);
        double x;
        double y;
        geometry_msgs::Point32 p;
        std::vector<geometry_msgs::Point32> obstacle_center_points;
        for(auto& box : ptr->objects){
            double x = box.pose.position.x;
            double y = box.pose.position.y;
            p.x = x; p.y = y;
            obstacle_center_points.push_back(p);
        }

        v2m_lock.lock();
        v2m.obstacle_center_points_updated = true;
        v2m.obstacle_center_points = obstacle_center_points;
        v2m_lock.unlock();
    }

    void clouds_center_points_CB_quadtree(const autoware_msgs::CloudClusterArrayConstPtr& ptr){
        // autoware_msgs::CloudCluster c;
        // std::vector<autoware_msgs::CloudCluster>
        // objects_pub.publish(ptr);
        // double x;
        // double y;
        // geometry_msgs::Point32 p;
        // std::vector<geometry_msgs::Point32> obstacle_center_points;
        // for(auto& box : ptr->clusters){
        //     double x = box.centroid_point.point.x;
        //     double y = box.centroid_point.point.y;
        //     p.x = x; p.y = y;
        //     obstacle_center_points.push_back(p);
        // }

        // v2m_lock.lock();
        // v2m.obstacle_center_points_updated = true;
        // v2m.obstacle_center_points = obstacle_center_points;
        // v2m_lock.unlock();
    }

    void callbackGetVMPoints(const vector_map_msgs::PointArrayConstPtr& ptr){
        //std::cout << "1"  << std::endl;
        vm_points = *ptr;
        // for(auto p: ptr)
        // auto last_point = *(ptr->data.end() - 1);
        // // //std::cout << "last_point: " << last_point << std::endl; //0726 추가 
        // auto last_prev_point = *(ptr->data.end() - 2);

        // for(int i = 0; i < ptr->data.size(); i++){
        //     last_point = *(ptr->data.end() - 1 -i);
        //     last_prev_point = *(ptr->data.end() - 3 - i);
        //     if(last_point.b == 0 && last_point.l == 0 && last_point.mcode1 == 0 && last_point.mcode2 == 0 && last_point.mcode3 == 0){
        //         break;
        //     }
        // }
        // ROS_INFO("size : %lu", ptr->data.size());
        // ROS_INFO("last_point ly, bx : %lf %lf", last_point.ly, last_point.bx);
        // ROS_INFO("last_prev_point ly, bx : %lf %lf", last_prev_point.ly, last_prev_point.bx);
        
        // //set goalpoint poisition
        // Gx = last_point.ly;
        // Gy = last_point.bx;

        // //calc goal point orientation
        // double dx = last_point.ly - last_prev_point.ly;
        // double dy = last_point.bx - last_prev_point.bx;
        // double yaw = std::atan2(dy, dx);
        // ROS_INFO("dx, dy, yaw : %lf %lf %lf", dx, dy, yaw);


        // tf::Quaternion quat_tf = tf::createQuaternionFromYaw(yaw);
        // tf::quaternionTFToMsg(quat_tf, msg.pose.orientation);

        // msg.pose.position.x = Gx;
        // msg.pose.position.y = Gy;
        // msg.pose.position.z = 0.0;

    	// is_vectormap_subscribed = true;

	    // std::thread(&GoalInitializer::run, this).detach();
    }

    //delivery sign
    void delivery_signCB(const autoware_msgs::DetectedObjectArray& msg){
        v2m_lock.lock();
        
        /* (1)map frame */
        tf::StampedTransform transform; // add
        //std::cout << "1"  << std::endl;
	    GetTransformFromTF("/map", "/velodyne", transform); // add
        //std::cout << "2"  << std::endl;

        autoware_msgs::DetectedObjectArray detected_objects;
        detected_objects.header = msg.header;
        detected_objects.header.frame_id = "/map"; // add

        size_t id = 1;

        for (auto& object : msg.objects)
        {
            geometry_msgs::PolygonStamped object_polygon;
            object_polygon.header = msg.header;

            autoware_msgs::DetectedObject detected_object;
            detected_object.id = id++;
            detected_object.header = msg.header;
            detected_object.header.frame_id = "/map"; // add
            detected_object.label = object.label;
// std::cout << "delivery-signCB/label : " << object.label << std::endl;

            detected_object.indicator_state = 3;
            // detected_object.velocity.linear.x = 0.0;
            // detected_object.velocity.linear.y = 0.0;
            // detected_object.velocity.linear.z = 0.0;
            // detected_object.velocity_reliable = false;
            detected_object.pose_reliable = true;


            pcl::PointXYZ input1; // add
            input1.x = object.pose.position.x;// add
            input1.y = object.pose.position.y;// add
            input1.z = object.pose.position.z;// add

            pcl::PointXYZ result1 = TransformPoint(input1, transform);// add

            //detected_object.pose = object.pose;
            detected_object.pose.position.x = result1.x;
            detected_object.pose.position.y = result1.y;
            detected_object.pose.position.z = result1.z;
            detected_object.pose.orientation = object.pose.orientation;
            detected_object.dimensions = object.dimensions;

            detected_object.valid = true;
            detected_objects.objects.push_back(detected_object);
        }

        v2m.dv_signs = detected_objects;
        /* (1)velodyne frame */
        // v2m.dv_signs_v = detected_objects;
// std::cout << "dv_signs.size : " << detected_objects.objects.size() << std::endl;
        v2m.dv_signs_updated = true;
        v2m_lock.unlock();
//std::cout << "3"  << std::endl;
    }

    pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
    {
        tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
        tf::Vector3 tf_point_t = in_transform * tf_point;
        return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
    }

    void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
    {
        static tf::TransformListener listener;

        int nFailedCounter = 0;
        while (1)
        {
            try
            {
                listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
                break;
            }
            catch (tf::TransformException& ex)
            {
                if(nFailedCounter > 2)
                {
                ROS_ERROR("%s", ex.what());
                }
                ros::Duration(1.0).sleep();
                nFailedCounter ++;
            }
        }
    }
private:
    //mission related members
    MissionInfo* cur_mission;
    MissionInfo* mission_none;
    std::vector<MissionInfo> mission_infos;
    int mission_identifier_cur;
  
    //재훈코드
    jsk_recognition_msgs::PolygonArray polyArr;
    visualization_msgs::MarkerArray markera; //jaehoon

    //concurrency members
    Vehicle2Mission v2m;
    Mission2Vehicle m2v;
    std::mutex v2m_lock;
    std::mutex m2v_lock;

    //ros members
    //sub
    ros::Subscriber traffic_light_sub; //traffic light detection
    ros::Subscriber curposeSub;
    ros::Subscriber curTwistSub;
    ros::Subscriber objects_sub;  //avoid obstacle
    ros::Subscriber twist_cmd_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber twist_purepursuit_sub;
    ros::Subscriber objects_center_points_sub;
    ros::Subscriber clouds_center_points_sub;
    ros::Subscriber vm_points_sub;
    ros::Subscriber traffic_sign_sub;
    // std::string publish_objects_topic_name_;
    // ros::Subscriber sub_goal_pose;
    //pub
    ros::Publisher erp42_twist_pub;
    ros::Publisher poly_pub;
    ros::Publisher marker_pub;
    ros::Publisher objects_pub; //avoid obstacle
    ros::Publisher tracked_obj_pub;
    //nh
    ros::NodeHandle nh;
    bool isStart;
};

int main(int argc, char *argv[]){
    //std::cout << "main" << std::endl;
    ros::init(argc, argv, "mission_manager");
    MissionManager m;
    ros::spin();
}