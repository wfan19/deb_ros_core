#include "ros/ros.h"

#include <pid_ros_deb/controller/FoilboatController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_ros_deb_node");
  ros::NodeHandle n;

  FoilboatController mFoilboatController(n);
  bool init_status = mFoilboatController.init();
  if(init_status)
  {
    mFoilboatController.start();
  }
  return EXIT_SUCCESS;
}