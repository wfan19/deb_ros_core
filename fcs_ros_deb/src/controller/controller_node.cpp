#include "ros/ros.h"

#include <fcs_ros_deb/controller/FoilboatController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fcs_ros_deb_node");
  ros::NodeHandle n;

  FoilboatController mFoilboatController(n);
  bool init_status = mFoilboatController.init();
  if(init_status)
  {
    mFoilboatController.start();
  }
  return EXIT_SUCCESS;
}