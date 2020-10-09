#ifndef SRC_CLEARPATH_LLC_HPP
#define SRC_CLEARPATH_LLC_HPP

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <pid_ros_deb/FoilboatControl.h>

using namespace std;
class ClearpathLLC
{
public:
  ClearpathLLC(ros::NodeHandle *n);
  ~ClearpathLLC();

  int init();

private:
  void onControl(const pid_ros_deb::FoilboatControl::ConstPtr control_msg);

  ros::NodeHandle n;
  ros::Subscriber control_sub;
  ros::Publisher left_flap_pub;
  ros::Publisher right_flap_pub;
  ros::Publisher left_elevator_pub;
  ros::Publisher right_elevator_pub;

  bool using_elevator = false;

  int servo_encoder_cpr{800};
  double servo_to_flap_ratio{142}; // Radians to rotations
  double servo_to_elevator_ratio{1000}; // Radians to rotations
  int left_flap_servo_midpoint{0}; // Encoder counts
  int right_flap_servo_midpoint{0};
  int left_elevator_servo_midpoint{-8500}; // Encoder counts
  int right_elevator_servo_midpoint{-6300};
};

#endif //SRC_CLEARPATH_LLC_HPP
