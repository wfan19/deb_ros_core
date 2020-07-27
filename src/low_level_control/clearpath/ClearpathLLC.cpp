#include "fcs_ros_deb/low_level_control/clearpath/ClearpathLLC.hpp"

using namespace std;
ClearpathLLC::ClearpathLLC(ros::NodeHandle *n)
{
  this->n = *n;
}

ClearpathLLC::~ClearpathLLC()
{

}

int ClearpathLLC::init()
{
  control_sub = n.subscribe("foilboat/control", 100, &ClearpathLLC::onControl, this);

  left_flap_pub = n.advertise<std_msgs::Float64>("clearpath/servo0/position_target", 1, 100);
  right_flap_pub = n.advertise<std_msgs::Float64>("clearpath/servo1/position_target", 1, 100);
  left_elevator_pub = n.advertise<std_msgs::Float64>("clearpath/servo2/position_target", 1, 100);
  right_elevator_pub = n.advertise<std_msgs::Float64>("clearpath/servo3/position_target", 1, 100);
  return 0;
}

void ClearpathLLC::onControl(const fcs_ros_deb::FoilboatControl::ConstPtr control_msg)
{
  double left_flap_out, right_flap_out, left_elevator_out, right_elevator_out;
  std_msgs::Float64 left_flap_msg, right_flap_msg, left_elevator_msg, right_elevator_msg;

  left_flap_out = control_msg->leftFlap * servo_to_flap_ratio * servo_encoder_cpr + left_flap_servo_midpoint;
  right_flap_out = control_msg->rightFlap * servo_to_flap_ratio * servo_encoder_cpr + right_flap_servo_midpoint;

  left_elevator_out = control_msg->leftElevator * servo_to_elevator_ratio * servo_encoder_cpr + left_elevator_servo_midpoint;
  right_elevator_out = control_msg->rightElevator * servo_to_elevator_ratio * servo_encoder_cpr + right_elevator_servo_midpoint;

  left_flap_msg.data = left_flap_out;
  right_flap_msg.data = right_flap_out;
  left_elevator_msg.data = left_elevator_out;
  right_elevator_msg.data = right_elevator_out;
}