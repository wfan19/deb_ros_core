#ifndef FOILBOAT_CONTROLLER_HPP
#define FOILBOAT_CONTROLLER_HPP

#include <map>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <dynamic_reconfigure/server.h>
#include <pid_ros_deb/ControllerConfig.h>

#include <pid_ros_deb/FoilboatTarget.h>
#include <pid_ros_deb/FoilboatControl.h>
#include <pid_ros_deb/FoilboatState.h>

#include "PIDFF.hpp"
#include "PIDWrapper.hpp"

using namespace std;
class FoilboatController
{
public:
  FoilboatController(ros::NodeHandle nh);
  ~FoilboatController();

  bool init();
  void start();
  void control(const ros::TimerEvent &event);

private:
  void onImu(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void onLaser(const sensor_msgs::LaserScan::ConstPtr& laserPtr);
  void onZEstimate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr);
  void onOdom(const nav_msgs::Odometry::ConstPtr& odomPtr);
  void onTarget(const pid_ros_deb::FoilboatTarget::ConstPtr& targetPtr);

  void onConfig(pid_ros_deb::ControllerConfig &config, uint32_t level);

  PIDFF::PIDConfig convertPIDMapParamToStruct(map<string, float> pidConfigMap);

  ros::NodeHandle n;
  ros::Rate controller_frequency;
  int controller_mode{0};

  ros::Subscriber imu_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber z_estimate_sub;
  ros::Subscriber odom_sub;

  ros::Subscriber target_sub;
  ros::Publisher control_pub;
  ros::Publisher state_pub;

  tf2::Quaternion lastOrientation;
  pid_ros_deb::FoilboatState last_state;
  ros::Time last_laser_time;

  pid_ros_deb::FoilboatTarget::ConstPtr lastTarget{new pid_ros_deb::FoilboatTarget};

  PIDWrapper controller_pid;

  PIDFF::PIDConfig roll_controller_config;
  PIDFF::PIDConfig pitch_controller_config;
  PIDFF::PIDConfig altitude_controller_config;
  PIDFF::PIDConfig altitude_rate_controller_config;

};

#endif