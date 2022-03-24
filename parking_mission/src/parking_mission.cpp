#include <parking_mission.hpp>

namespace parking_mission
{
    ParkingMission::ParkingMission() : private_nh_("~"), start_(false), checking1(false),in_mission_clear_(false), get_latest_objects_(false), index_in_(0), index_out_(1), checking2_start(false), checking_endpoint_(false), isEmptyArea(false), checkEmpty(false)
    {
        if(!private_nh_.getParam("parkinglotpoints_file_name", parkinglotpoints_file_name_))       throw std::runtime_error("fail to get parkinglotpoints_file_name");
        if(!private_nh_.getParam("checkpoint_reach_threshold", checkpoint_reach_threshold_))       throw std::runtime_error("fail to get checkpoint_reach_threshold");
        if(!private_nh_.getParam("one_parking_area_point_num", one_parking_area_point_num_))       throw std::runtime_error("fail to get one_parking_area_point_num");

        mission_call_sub_ = nh_.subscribe("/parking_mission_call", 1, &ParkingMission::callbackParkingMission, this);
        current_pose_sub_ = nh_.subscribe("/current_pose", 1, &ParkingMission::callbackGetCurrentPose, this);
        lidar_objects_sub_ = nh_.subscribe("/detection/lidar_detector/objects", 1, &ParkingMission::callbackGetLidarObjects, this);

        goal_pub_ = nh_.advertise<geometry_msgs::Point>("/trackdriving_final_waypoint", 1);
        goal_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>("/parking_waypoint_rviz", 1);
        backdriving_pub_ = nh_.advertise<std_msgs::Bool>("/backdriving", 1);
        parking_mission_end_pub_ = nh_.advertise<std_msgs::Bool>("/parking_mission_end", 1);
        checking_area_polygons_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/checking_area_polygons", 1);
        estop_pub_ = nh_.advertise<std_msgs::Bool>("/erp42_estop", 10);
        distance_pub_ = nh_.advertise<std_msgs::Float32>("/distance", 1);

        parkinglot_points_ = loadParkinglotPoints();

        std::cout << "parkinglot_points area : " << parkinglot_points_.size() << std::endl;

        for(size_t i = 0; i < parkinglot_points_.size(); ++i)
        {
            for(size_t k = 0; k < parkinglot_points_[i].size(); ++k)
            {
                std::cout << parkinglot_points_[i][k] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "parking Mission setting complete" << std::endl;

        waiting_mission_clear_ = false;
        first_time_ = true;
    }   

    ParkingMission::~ParkingMission()
    {

    }

    void ParkingMission::callbackParkingMission(const std_msgs::BoolConstPtr& ptr)
    {
        // std::cout << "callback ParkingMission!" << std::endl;
        start_ = true;
    }

    void ParkingMission::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_pose_.x = msg->pose.position.x;
        current_pose_.y = msg->pose.position.y;
        current_pose_.z = 0.0;
    }

    void ParkingMission::callbackGetLidarObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg)
    {
        if(start_ == false)
        {
            //std::cout << "Parking Mission is not yet" << std::endl;
            return;
        }

        if(in_mission_clear_ == false)
        {
            if(get_latest_objects_ == false && checking2_start == false )
            {
                if(checking1 == false)
                {
                    std::vector<std::vector<geometry_msgs::Point>> selectedParkingArea = filteringParkingArea();
                    if(selectedParkingArea.size() == 0)
                    {
                        std::cout << "we cannot find ParkingArea in forward" << std::endl;
                        return;
                    }
                    candidateArea = selectedParkingArea[0];
                    checking1 = true;
                }

                tf::StampedTransform transform;
                GetTransformFromTF("/velodyne", "/map", transform); 

                // move first area
                geometry_msgs::Point pt = TransformPoint(candidateArea[candidateArea.size() - 1], transform);
                goal_pub_.publish(pt);
                
                for(size_t i = 0; i < parkinglot_points_.size(); ++i)
                {
                    for(size_t k = 0;  k < parkinglot_points_[i].size(); ++k)
                    {
                        printPoint(parkinglot_points_[i][k], 0.5, 0.5, 0.5);
                    }
                }
                printPoint(candidateArea[candidateArea.size() - 1], 1.0, 0.0, 0.0);
                goal_rviz_pub_.publish(rviz_points_);
                rviz_points_.points.resize(0);
                rviz_points_.colors.resize(0);

                // std::cout << candidateArea[candidateArea.size() - 1].x << " , " << candidateArea[candidateArea.size() - 1].y << std::endl;
                // std::cout << current_pose_.x << " , " << current_pose_.y << std::endl;
                // std::cout << " remain distance : " << getDistance(candidateArea[candidateArea.size() - 1], current_pose_) << std::endl;

                // reach first area
                if(getDistance(candidateArea[candidateArea.size() - 1], current_pose_) < checkpoint_reach_threshold_) 
                {
                    std::cout <<"parking_mission - reached checkPoint!" << std::endl;
                    checking2_start = true;
                }
                else
                {
                    std_msgs::Float32 dist;
                    dist.data = getDistance(candidateArea[candidateArea.size() - 1], current_pose_);
                    // distance_pub_.publish(dist);
                    std::cout << "parking_mission - moving checkPoint!" << std::endl;
                    return;
                }       
            }
            
            if(checking2_start == false) return;
            
            if(get_latest_objects_ == false)
            {
                get_latest_objects_ = true;
                return;
            }    
            
            std::cout << "parking_mission - checking area object" << std::endl;
            // checking
            if(checkEmpty == false)
            {
                makeCheckingAreaPolygons(candidateArea);
                isEmptyArea = checkingEmptyArea(msg, candidateArea);
                if(isEmptyArea == true)
                    checkEmpty = true;
            }              

            if(isEmptyArea == true)
            {
                tf::StampedTransform transform;
                GetTransformFromTF("/velodyne", "/map", transform); 

                std::cout << "isEmptyArea!! go to next point "<< index_in_ << " of " << candidateArea.size() << "candidateArea" << std::endl;

                // move first area
                geometry_msgs::Point pt = TransformPoint(candidateArea[candidateArea.size() - index_in_ - 2], transform);
                goal_pub_.publish(pt);
                for(size_t i = 0; i < parkinglot_points_.size(); ++i)
                {
                    for(size_t k = 0;  k < parkinglot_points_[i].size(); ++k)
                    {
                        printPoint(parkinglot_points_[i][k], 0.5, 0.5, 0.5);
                    }
                }
                printPoint(candidateArea[candidateArea.size() - index_in_ - 2], 0.5, 0.0, 0.0);

                goal_rviz_pub_.publish(rviz_points_);
                rviz_points_.points.resize(0);
                rviz_points_.colors.resize(0);
                // std::cout <<"parking_mission - moving checkPoint!" << std::endl;
                std::cout << " remain distance : " << getDistance(candidateArea[candidateArea.size() - index_in_ - 2], current_pose_) << std::endl;
                // reach first area
                if(getDistance(candidateArea[candidateArea.size() - index_in_ - 2], current_pose_) < checkpoint_reach_threshold_) 
                {
                    std::cout <<"parking_mission - reached checkPoint!" << std::endl;
                    index_in_++;
                    if(candidateArea.size() - index_in_ - 2 == -1)
                    {
                        in_mission_clear_ = true;
                    }
                    
                }
            }
            else
            {
                std::cout << "is no EmptyArea!! start to find other candidateArea" << std::endl;
                get_latest_objects_ = false;
                checking1 = false;
                checking2_start = false;
            }
        }
        
        if(!in_mission_clear_) return;

        if(!waiting_mission_clear_)
        {
            if(first_time_)
            {
                waiting_start_time_ = ros::Time::now();
                first_time_ = false;
            }
                
            std_msgs::Bool estop;
            estop.data = true;
            estop_pub_.publish(estop);
            std::cout << (ros::Time::now() - waiting_start_time_).toSec() << std::endl;
            if((ros::Time::now() - waiting_start_time_).toSec() >= 10)
                waiting_mission_clear_ = true;
        }
        else
        {
            std_msgs::Bool estop;
            estop.data = false;
            estop_pub_.publish(estop);
        }
        

        if(checking_endpoint_ == false)
        {
            std::cout << "out mission start!!" << std::endl;
            std_msgs::Bool back;
            back.data = true;
            backdriving_pub_.publish(back);

            tf::StampedTransform transform;
            GetTransformFromTF("/velodyne", "/map", transform); 

            // move first area
            geometry_msgs::Point pt = TransformPoint(candidateArea[index_out_], transform);
            goal_pub_.publish(pt);
            for(size_t i = 0; i < parkinglot_points_.size(); ++i)
                {
                    for(size_t k = 0;  k < parkinglot_points_[i].size(); ++k)
                    {
                        printPoint(parkinglot_points_[i][k], 0.5, 0.5, 0.5);
                    }
                }
                            printPoint(candidateArea[index_out_], 0.5, 0.0, 0.0);

            goal_rviz_pub_.publish(rviz_points_);
            rviz_points_.points.resize(0);
            rviz_points_.colors.resize(0);
            std::cout <<"parking_mission - moving checkPoint!" << std::endl;

            // reach first area
            if(getDistance(candidateArea[index_out_], current_pose_) < checkpoint_reach_threshold_) 
            {
                std::cout <<"parking_mission - reached checkPoint!" << std::endl;
                index_out_++;
                if(index_out_ == candidateArea.size())
                    checking_endpoint_ = true;
            }
        }

        if(checking_endpoint_ == false) return;

        std_msgs::Bool end;
        end.data = true;
        parking_mission_end_pub_.publish(end);  
        ros::shutdown();
    }

    std::vector<std::vector<geometry_msgs::Point>> ParkingMission::loadParkinglotPoints()
    {
        std::vector<std::vector<geometry_msgs::Point>> total_parkinglot_points;
        std::vector<geometry_msgs::Point> vec;

        std::fstream fs;
        fs.open(parkinglotpoints_file_name_, std::ios::in);
        fs << std::fixed << std::setprecision(10);

        int count = 0;

        std::string x, y, z;

        while(!fs.eof())
        {
            if(count % 3 == 0)
                getline(fs, x, ',');
            else if(count % 3 == 1)
                getline(fs, y, ',');
            else if(count % 3 == 2)
            {
                getline(fs, z, '\n');
                z = "0.0";

                geometry_msgs::Point pt;
                pt.x = stod(x);
                pt.y = stod(y);
                pt.z = stod(z);

                vec.push_back(pt);
            }
            count++;

            if(vec.size() == one_parking_area_point_num_)
            {
                total_parkinglot_points.push_back(vec);
                vec.resize(0);
            }
        }

        return total_parkinglot_points;
    }
    
    std::vector<std::vector<geometry_msgs::Point>> ParkingMission::filteringParkingArea()
    {
        std::vector<std::vector<geometry_msgs::Point>> filtered_area;

        tf::StampedTransform transform;
	    GetTransformFromTF("/velodyne", "/map", transform); 

        for(size_t i = 0; i < parkinglot_points_.size(); ++i)
        {
            geometry_msgs::Point pt = TransformPoint(parkinglot_points_[i][parkinglot_points_[i].size()-1], transform);

            if(pt.x < checkpoint_reach_threshold_ + 0.5) continue;

            filtered_area.push_back(parkinglot_points_[i]);
        }

        return filtered_area;
    }

    void ParkingMission::makeCheckingAreaPolygons(std::vector<geometry_msgs::Point>& candidateArea)
    {
        jsk_recognition_msgs::PolygonArray checking_area_polygons;
        checking_area_polygons.header.frame_id = "/map";
        checking_area_polygons.header.stamp = ros::Time::now();

        for(size_t i = 0; i < candidateArea.size(); ++i)
        {
            geometry_msgs::PolygonStamped checking_area_polygon;
            checking_area_polygon.header.frame_id = "/map";
            checking_area_polygon.header.stamp = ros::Time::now();
            
            geometry_msgs::Point32 point;
            point.x = candidateArea[i].x + 0.7;
            point.y = candidateArea[i].y + 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x;
            point.y = candidateArea[i].y + 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x - 0.7;
            point.y = candidateArea[i].y + 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x - 0.7;
            point.y = candidateArea[i].y;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x - 0.7;
            point.y = candidateArea[i].y - 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x;
            point.y = candidateArea[i].y - 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x + 0.7;
            point.y = candidateArea[i].y - 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x + 0.7;
            point.y = candidateArea[i].y;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            point.x = candidateArea[i].x + 0.7;
            point.y = candidateArea[i].y + 0.7;
            point.z = 0.0;
            checking_area_polygon.polygon.points.push_back(point);

            checking_area_polygons.polygons.push_back(checking_area_polygon);
        }

        checking_area_polygons_pub_.publish(checking_area_polygons);
    }

    bool ParkingMission::checkingEmptyArea(const autoware_msgs::DetectedObjectArrayConstPtr &lidar_objects, std::vector<geometry_msgs::Point>& candidateArea)
    {
        int clear_target = 0;
        tf::StampedTransform transform;
	    GetTransformFromTF("/map", "/velodyne", transform);

        for(size_t m = 0; m < candidateArea.size() - 1; m++)
        {
            int count = 0;
            for(size_t i = 0; i < lidar_objects->objects.size(); ++i)
            { 
                geometry_msgs::Point pt;
                pt.x = lidar_objects->objects[i].pose.position.x;
                pt.y = lidar_objects->objects[i].pose.position.y;
                pt.z = lidar_objects->objects[i].pose.position.z;

                geometry_msgs::Point map_object_pose_pt = TransformPoint(pt, transform);

                if(getDistance(map_object_pose_pt, current_pose_) > 50) continue;

                std::vector<cv::Point2f> object_convexhull;
                std::vector<cv::Point2f> target_convexhull;

                // make_object_polygon
                for(size_t k = 0; k < lidar_objects->objects[i].convex_hull.polygon.points.size() / 2; ++k)
                {
                    geometry_msgs::Point velodyne_frame_pt;
                    velodyne_frame_pt.x = lidar_objects->objects[i].convex_hull.polygon.points[k].x;
                    velodyne_frame_pt.y = lidar_objects->objects[i].convex_hull.polygon.points[k].y;
                    velodyne_frame_pt.z = lidar_objects->objects[i].convex_hull.polygon.points[k].z;

                    geometry_msgs::Point map_object_pt = TransformPoint(velodyne_frame_pt, transform);
                    cv::Point2f temp;
                    temp.x = map_object_pt.x;
                    temp.y = map_object_pt.y;

                    object_convexhull.push_back(temp);
                }

                // make_target_polygon
                cv::Point2f temp;
                temp.x = candidateArea[m].x + 0.5;
                temp.y = candidateArea[m].y + 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x;
                temp.y = candidateArea[m].y + 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x - 0.5;
                temp.y = candidateArea[m].y + 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x - 0.5;
                temp.y = candidateArea[m].y;
                target_convexhull.push_back(temp);
                
                temp.x = candidateArea[m].x - 0.5;
                temp.y = candidateArea[m].y - 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x;
                temp.y = candidateArea[m].y - 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x + 0.5;
                temp.y = candidateArea[m].y - 0.5;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x + 0.5;
                temp.y = candidateArea[m].y;
                target_convexhull.push_back(temp);

                temp.x = candidateArea[m].x + 0.5;
                temp.y = candidateArea[m].y + 0.5;
                target_convexhull.push_back(temp);

                bool ok = checkPointPolygonTest(object_convexhull, target_convexhull);                

                if(ok == false) // go to next level
                    return false;
            }
        }
        
        return true;
    }

    bool ParkingMission::checkPointPolygonTest(std::vector<cv::Point2f>& object_convexhull, std::vector<cv::Point2f>& target_convexhull)
    {
        for(size_t k = 0; k < target_convexhull.size(); ++k)
        {
            int location = cv::pointPolygonTest(object_convexhull, target_convexhull[k], false);

            if(location >= 0) // inside = 1 , on convexhull = 0, outside = -1
                return false;
        }

        for(size_t k = 0; k < object_convexhull.size(); ++k)
        {
            int location = cv::pointPolygonTest(target_convexhull, object_convexhull[k], false);

            if(location >= 0) // inside = 1 , on convexhull = 0, outside = -1
                return false;
        }

        return true;
    }

    void ParkingMission::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
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
    
    geometry_msgs::Point ParkingMission::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
    {
        tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
        tf::Vector3 tf_point_t = in_transform * tf_point;
        geometry_msgs::Point pt;
        pt.x = tf_point_t.x();
        pt.y = tf_point_t.y();
        pt.z = tf_point_t.z();
        return pt;
    }

    void ParkingMission::printPoint(geometry_msgs::Point &point, double r, double g, double b)
    {
        std_msgs::ColorRGBA c;
        int id = 0;

        c.r = r;
        c.g = g;
        c.b = b;
        c.a = 1.0;

        rviz_points_.header.frame_id = "/map";
        rviz_points_.header.stamp = ros::Time::now();
        rviz_points_.ns = "points";
        rviz_points_.action = visualization_msgs::Marker::ADD;
        rviz_points_.pose.orientation.w = 1.0;
        rviz_points_.lifetime = ros::Duration(0.1);
        rviz_points_.id = id++;
        rviz_points_.type = visualization_msgs::Marker::POINTS;
        rviz_points_.scale.x = 0.5;
        rviz_points_.scale.y = 0.5;

        rviz_points_.color.r = 0.0;
        rviz_points_.color.g = 0.0;
        rviz_points_.color.b = 1.0;
        rviz_points_.color.a = 1.0;

        rviz_points_.points.push_back(point);
        rviz_points_.colors.push_back(c);

    }

    double ParkingMission::getDistance(geometry_msgs::Point &point1, geometry_msgs::Point &point2)
    {
        return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
    }
}
 

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "parking_mission");

  parking_mission::ParkingMission PM;

  ros::spin();

  return 0;
}
