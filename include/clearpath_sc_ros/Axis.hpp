#ifndef SRC_AXIS_HPP
#define SRC_AXIS_HPP

#include <thread>

#include "pubSysCls.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <clearpath_sc_ros/ServoState.h>
#include <clearpath_sc_ros/ServoConfig.h>
#include <clearpath_sc_ros/GetConfig.h>
#include <clearpath_sc_ros/HomeAxis.h>
#include <clearpath_sc_ros/ClearAlerts.h>

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
  string axis_name;

  int min_position;
  int max_position;

  // Status publishing loop
  void statusLoop();

  // Position control loop
  double position_target{0};
  void positionLoop();

  ros::Publisher servo_state_pub;
  double last_error_code;

  ros::Subscriber position_target_sub;
  void onPositionTarget(const std_msgs::Float64::ConstPtr target_msg);

  ros::ServiceServer getConfig_service;
  bool getConfig(
      clearpath_sc_ros::GetConfig::Request &req,
      clearpath_sc_ros::GetConfig::Response &res
  );

  ros::ServiceServer homeAxis_service;
  bool homeAxis(
      clearpath_sc_ros::HomeAxis::Request &req,
      clearpath_sc_ros::HomeAxis::Response &res
  );

  ros::ServiceServer clearAlert_service;
  bool clearAlert(
    clearpath_sc_ros::ClearAlerts::Request &req,
    clearpath_sc_ros::ClearAlerts::Response &res
  );
};

#endif //SRC_AXIS_HPP
