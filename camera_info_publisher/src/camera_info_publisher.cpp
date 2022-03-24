#include <camera_info_publisher.hpp>

namespace camera_info_publisher
{
    CameraInfoPublisher::CameraInfoPublisher() : private_nh_("~")
    {
        pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);


        calib_mat_k_.assign(9, 0);
        calib_mat_r_.assign(9, 0);
        calib_mat_p_.assign(12, 0);

        if(!private_nh_.getParam("width", width_))                 throw std::runtime_error("fail to get width");
        if(!private_nh_.getParam("height", height_))               throw std::runtime_error("fail to get height");
        if(!private_nh_.getParam("c_x", c_x_))                     throw std::runtime_error("fail to get c_x");
        if(!private_nh_.getParam("c_y", c_y_))                     throw std::runtime_error("fail to get c_y");
        if(!private_nh_.getParam("f_x", f_x_))                     throw std::runtime_error("fail to get f_x");
        if(!private_nh_.getParam("f_y", f_y_))                     throw std::runtime_error("fail to get f_y");
        if(!private_nh_.getParam("max_f", max_f_))                 throw std::runtime_error("fail to get max_f");
        
        // if(!private_nh_.getParam("calib_mat_K", calib_mat_k_))    throw std::runtime_error("fail to get calib_mat_k");
        // if(!private_nh_.getParam("calib_mat_R", calib_mat_r_))    throw std::runtime_error("fail to get calib_mat_r");
        // if(!private_nh_.getParam("calib_mat_P", calib_mat_p_))    throw std::runtime_error("fail to get calib_mat_p");
        // if(!private_nh_.getParam("calib_mat_D", calib_mat_d_))    throw std::runtime_error("fail to get calib_mat_d");
        
        

        mat_K_ = {max_f_, 0, c_x_, 0, max_f_, c_y_, 0, 0, 1};

        mat_R_ = {1, 0, 0, 0, 1, 0, 0, 0, 1};                                     
    
        mat_P_ = {max_f_, 0, c_x_, 0, 0, max_f_, c_y_, 0, 0, 0, 1, 0};

        calib_mat_d_.push_back(0.1726);  
        calib_mat_d_.push_back(-2.348617);
        calib_mat_d_.push_back(0.011128);
        calib_mat_d_.push_back(0.009115);
    }

    CameraInfoPublisher::~CameraInfoPublisher()
    {

    }

    void CameraInfoPublisher::run()
    {
        sensor_msgs::CameraInfo CI;

        CI.header.frame_id = "camera";
        CI.header.stamp = ros::Time::now();
        
        CI.height = height_;
        CI.width = width_;

        CI.K = mat_K_;
        CI.R = mat_R_;
        CI.P = mat_P_;
        CI.D = calib_mat_d_;

        CI.binning_x = 0;
        CI.binning_y = 0;

        pub_camera_info_.publish(CI);
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "camera_info_publisher");

  camera_info_publisher::CameraInfoPublisher CIP;

  ros::Rate loop(100);

  while(ros::ok())
  {
      CIP.run();
      loop.sleep();
  }

  return 0;
}
