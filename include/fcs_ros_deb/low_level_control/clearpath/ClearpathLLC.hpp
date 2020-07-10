#ifndef SRC_CLEARPATH_LLC_HPP
#define SRC_CLEARPATH_LLC_HPP

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <fcs_ros_deb/FoilboatControl.h>

using namespace std;
class ClearpathLLC
{
public:
  ClearpathLLC(ros::NodeHandle *n);
  ~ClearpathLLC();

  int init();

private:
  void onControl(const fcs_ros_deb::FoilboatControl::ConstPtr);
  ros::NodeHandle n;
};

#endif //SRC_CLEARPATH_LLC_HPP
