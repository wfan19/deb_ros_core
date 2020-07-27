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
  string control_topic_name;
  if(n.getParam("clearpath_llc/control_topic", control_topic_name))
  {
    control_sub = n.subscribe("foilboat/control", 100, &ClearpathLLC::onControl, this);
  }
  else
  {
    ROS_ERROR("[ClearpathLLC] Missing required parameter clearpath_llc/control_topic");
    return -1;
  }

  string left_flap_numb, right_flap_numb, left_elevator_numb, right_elevator_numb;
  if(n.getParam("clearpath_llc/left_flap/number", left_flap_numb)
    && n.getParam("clearpath_llc/right_flap/number", right_flap_numb)
    && n.getParam("clearpath_llc/left_flap/midpiont", left_flap_servo_midpoint)
    && n.getParam("clearpath_llc/right_flap/midpoint", right_flap_servo_midpoint))
  {
    left_flap_pub = n.advertise<std_msgs::Float64>("clearpath/servo" + left_flap_numb + "/position_target",1, 100);
    right_flap_pub = n.advertise<std_msgs::Float64>("clearpath/servo" + right_flap_numb + "/position_target", 1, 100);
  }
  else
  {
    ROS_ERROR("[ClearpathLLC] Missing required parameters for clearpath_llc/left_flap or clearpath_llc/right_flap");
    return -1;
  }

  if(n.getParam("clearpath_llc/left_elevator/number", left_flap_numb)
     && n.getParam("clearpath_llc/right_elevator/number", right_flap_numb)
     && n.getParam("clearpath_llc/left_elevator/midpoint", left_elevator_servo_midpoint)
     && n.getParam("clearpath_llc/right_elevator/midpoint", right_elevator_servo_midpoint))
  {
    left_elevator_pub = n.advertise<std_msgs::Float64>("clearpath/servo2/position_target", 1, 100);
    right_elevator_pub = n.advertise<std_msgs::Float64>("clearpath/servo3/position_target", 1, 100);
  }
  else
  {
    ROS_ERROR("[ClearpathLLC] Missing parameters for clearpath_llc/left_flap or clearpath_llc/right_flap, running LLC without elevators");
  }

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