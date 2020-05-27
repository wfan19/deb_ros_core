#ifndef RANGESENSORPROCESSOR_HPP
#define RANGESENSORPROCESSOR_HPP

#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <foilboat_controller/FoilboatState.h>

class RangeSensorProcessor
{
public:
  RangeSensorProcessor(ros::NodeHandle nh);
  ~RangeSensorProcessor();

  void run();

private:
  void onState(const foilboat_controller::FoilboatState::ConstPtr statePtr);
  void onLaser(const sensor_msgs::LaserScan::ConstPtr laserPtr);

  ros::NodeHandle n;
  ros::Publisher pose_pub;
  foilboat_controller::FoilboatState::ConstPtr last_state{new foilboat_controller::FoilboatState};
};

#endif