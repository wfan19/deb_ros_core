#ifndef FOILBOAT_CONTROLLER_HPP
#define FOILBOAT_CONTROLLER_HPP

#include <map>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <foilboat_controller/PIDFF.hpp>
#include <foilboat_controller/FoilboatTarget.h>
#include <foilboat_controller/FoilboatControl.h>

using namespace std;
class FoilboatController
{
public:
  FoilboatController(ros::NodeHandle nh);
  ~FoilboatController();

private:
  void onImu(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void onLaser(const sensor_msgs::LaserScan::ConstPtr& laserPtr);
  void onTarget(const foilboat_controller::FoilboatTarget::ConstPtr& targetPtr);
  void control();

  PIDFF::PIDConfig convertPIDMapParamToStruct(map<string, float> pidConfigMap);

  ros::NodeHandle n;

  ros::Time lastControlTime;

  ros::Subscriber imu_sub;
  ros::Subscriber laser_sub;

  ros::Subscriber target_sub;
  ros::Publisher control_pub;

  sensor_msgs::Imu::ConstPtr lastIMUMsg{new sensor_msgs::Imu};
  tf2::Quaternion lastOrientation;
  sensor_msgs::LaserScan::ConstPtr lastLaser{new sensor_msgs::LaserScan};
  foilboat_controller::FoilboatTarget::ConstPtr lastTarget{new foilboat_controller::FoilboatTarget};

  PIDFF roll_controller;
  PIDFF pitch_controller;

};

#endif