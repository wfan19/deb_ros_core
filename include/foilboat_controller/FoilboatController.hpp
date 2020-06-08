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
#include <foilboat_controller/GainsConfig.h>

#include <foilboat_controller/FoilboatTarget.h>
#include <foilboat_controller/FoilboatControl.h>
#include <foilboat_controller/FoilboatState.h>

#include <foilboat_controller/PIDFF.hpp>
#include <foilboat_controller/PIDWrapper.hpp>

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
  void onTarget(const foilboat_controller::FoilboatTarget::ConstPtr& targetPtr);

  void onPIDConfig(foilboat_controller::GainsConfig &config, uint32_t level);

  PIDFF::PIDConfig convertPIDMapParamToStruct(map<string, float> pidConfigMap);

  ros::NodeHandle n;
  ros::Rate controller_frequency;

  ros::Subscriber imu_sub;
  ros::Subscriber laser_sub;
  ros::Subscriber z_estimate_sub;
  ros::Subscriber odom_sub;

  ros::Subscriber target_sub;
  ros::Publisher control_pub;
  ros::Publisher state_pub;

  tf2::Quaternion lastOrientation;
  foilboat_controller::FoilboatState last_state;
  ros::Time last_laser_time;

  foilboat_controller::FoilboatTarget::ConstPtr lastTarget{new foilboat_controller::FoilboatTarget};

  PIDWrapper controller_pid;

};

#endif