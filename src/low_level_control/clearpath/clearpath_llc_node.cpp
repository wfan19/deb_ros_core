#include "ros/ros.h"

#include "fcs_ros_deb/low_level_control/clearpath/ClearpathLLC.hpp"

using namespace std;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "clearpath_llc_node");
  ros::NodeHandle n;

  ClearpathLLC low_level_controller = ClearpathLLC(&n);

  return 0;
}