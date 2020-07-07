#ifndef SRC_AXIS_HPP
#define SRC_AXIS_HPP

#include <thread>

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <clearpath_sc_ros/ServoState.h>
#include <clearpath_sc_ros/ServoConfig.h>

using namespace sFnd;
using namespace std;

class Axis
{
private:
  thread status_thread;
  thread position_thread;

public:
  Axis(string axis_name, ros::NodeHandle* driver_nh, INode* node);
  ~Axis();

  int start();

private:
  // Node handles
  ros::NodeHandle n; // ROS node
  INode *mNode;      // Clearpath node (servo)

  // Status publishing loop
  void statusLoop();

  // Position control loop
  double position_target{0};
  void positionLoop();

  ros::Publisher servo_state_pub;
  double last_error_code;

  ros::Subscriber position_target_sub;

  void onPositionTarget(const std_msgs::Float64::ConstPtr target_msg);
};

#endif //SRC_AXIS_HPP
