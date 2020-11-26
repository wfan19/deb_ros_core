#include "ros/ros.h"

#include <lowlevel_ros_deb/low_level_control/clearpath/ClearpathLLC.hpp>

using namespace std;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "clearpath_llc_node");
  ros::NodeHandle n;

  ClearpathLLC low_level_controller = ClearpathLLC(&n);
  if(low_level_controller.init() != 0)
  {
    return -1;
  }
  ros::spin();
  return 0;
}