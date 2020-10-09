#ifndef RANGESENSORPROCESSOR_HPP
#define RANGESENSORPROCESSOR_HPP

#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <pid_ros_deb/FoilboatState.h>

class RangeSensorProcessor
{
public:
  RangeSensorProcessor(ros::NodeHandle nh);
  ~RangeSensorProcessor();

  bool init();
  void run();

private:
  void onState(const pid_ros_deb::FoilboatState::ConstPtr statePtr);
  void onLaser(const sensor_msgs::LaserScan::ConstPtr laserPtr);
  void onFloat(const std_msgs::Float64::ConstPtr floatPtr);
  void publishMsg(double data);

  ros::NodeHandle n;
  ros::Publisher pose_pub;
  ros::Subscriber state_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber float_sub;

  float sensor_covariance;

  pid_ros_deb::FoilboatState::ConstPtr last_state{new pid_ros_deb::FoilboatState};
};

#endif