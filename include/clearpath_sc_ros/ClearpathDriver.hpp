#ifndef SRC_CLEARPATHDRIVER_HPP
#define SRC_CLEARPATHDRIVER_HPP

#include <string>
#include <vector>

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace sFnd;
class ClearpathDriver{
public:
  ClearpathDriver(ros::NodeHandle nh);
  ~ClearpathDriver();

  int init();

private:
  void onMotorCommand(std_msgs::Float64::ConstPtr floatMsg);

  ros::NodeHandle n;

  SysManager mSysManager;

  int port_counter;

  ros::Subscriber motor_sub;
};

#endif //SRC_CLEARPATHDRIVER_HPP
