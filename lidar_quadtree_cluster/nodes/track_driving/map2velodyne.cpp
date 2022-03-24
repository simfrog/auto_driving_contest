#include <track_driving/map2velodyne.hpp>

namespace tf_publisher
{
  Map2Velodyne::Map2Velodyne() : seq(0), x(0.0), y(0.0), theta(0.0)
  {
    sub_car_info_ = nh_.subscribe("/current_velocity", 1, &Map2Velodyne::callbackCarVelocity, this);
    last = ros::Time::now();
  }

  Map2Velodyne::~Map2Velodyne()
  {

  }

  void Map2Velodyne::callbackCarVelocity(const geometry_msgs::TwistStampedConstPtr& ptr)
  {
    car_info_ = *ptr;

    velocity = car_info_.twist.linear.x;
    theta_dot = car_info_.twist.angular.z;
    // std::cout << "velocity : " << velocity << " , " << "theta_dot : " << theta_dot << std::endl;
  }

  void Map2Velodyne::pubTF()
  {
    ros::Time now = ros::Time::now();
    double dt = (now - last).toSec();
    
    // calc odometry 
    theta += theta_dot * dt;
    x_dot = velocity * cos(theta);
    y_dot = velocity * sin(theta);
    x += x_dot * dt;
    y += y_dot * dt;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, now, "/my_map", "/velodyne"));

    last = now;
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tf_publisher");
  tf_publisher::Map2Velodyne MRO;
  ros::Rate loop_rate(100); // dt is always 100ms

  while(ros::ok()){
      ros::spinOnce();
      MRO.pubTF();
      loop_rate.sleep();
  }
  return 0;
}