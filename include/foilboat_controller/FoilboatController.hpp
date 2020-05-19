#ifndef FOILBOAT_CONTROLLER_HPP
#define FOILBOAT_CONTROLLER_HPP

#include <map>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

#include <foilboat_controller/PIDFF.hpp>

using namespace std;
class FoilboatController
{
public:
  FoilboatController(ros::NodeHandle nh, int rate);
  ~FoilboatController();

  void run();

private:
  void onImu(sensor_msgs::Imu::ConstPtr& imuPtr);
  void onLaser(sensor_msgs::LaserScan::ConstPtr& laserPtr);
  void control();

  PIDFF::PIDConfig convertPIDMapParamToStruct(map<string, float> pidConfigMap);

  ros::NodeHandle n;
  ros::Rate controllerRate;
  sensor_msgs::Imu::ConstPtr lastIMUMsg;
  sensor_msgs::LaserScan::ConstPtr lastLaser;

  PIDFF roll_controller;
  PIDFF pitch_controller;

};

#endif