#ifndef SRC_AXIS_HPP
#define SRC_AXIS_HPP

#include <thread>

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace sFnd;
using namespace std;

class Axis
{
private:
  thread position_thread;
  thread main_thread;

public:
  Axis(string axis_name, ros::NodeHandle* driver_nh, INode* node);
  ~Axis();

  int start();

private:

  ros::NodeHandle n;
  INode *mNode;

  void mainLoop();

  double position_target{0};
  void positionLoop();

  ros::Publisher position_state_pub;

  ros::Subscriber position_target_sub;

  void onPositionTarget(const std_msgs::Float64::ConstPtr target_msg);
};

#endif //SRC_AXIS_HPP
