#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>


namespace camera_info_publisher{
    class CameraInfoPublisher{
        public:
            CameraInfoPublisher();
            ~CameraInfoPublisher();

            void run();
            void setParameters();

        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            ros::Publisher pub_camera_info_;
            
            double width_;
            double height_;
            double c_x_;
            double c_y_;
            double f_x_;
            double f_y_;
            double max_f_;
            
            std::vector<double> calib_mat_k_;
            std::vector<double> calib_mat_r_;
            std::vector<double> calib_mat_p_;
            std::vector<double> calib_mat_d_;

            boost::array<double, 9> mat_K_;
            boost::array<double, 9> mat_R_;
            boost::array<double, 12> mat_P_;

    };
}