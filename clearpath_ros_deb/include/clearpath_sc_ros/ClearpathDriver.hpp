#ifndef SRC_CLEARPATHDRIVER_HPP
#define SRC_CLEARPATHDRIVER_HPP

#include <string>
#include <vector>

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <clearpath_sc_ros/Axis.hpp>

using namespace sFnd;
class ClearpathDriver{
public:
  ClearpathDriver(ros::NodeHandle nh);
  ~ClearpathDriver();

  int init();

private:
  ros::NodeHandle n;

  SysManager mSysManager;
  vector<Axis*> axes_list;

  int port_counter{0};
};

#endif //SRC_CLEARPATHDRIVER_HPP
