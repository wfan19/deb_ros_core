#include "ros/ros.h"

#include <foilboat_controller/controller/FoilboatController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foilboat_controller_node");
  ros::NodeHandle n;

  FoilboatController mFoilboatController(n);
  bool init_status = mFoilboatController.init();
  if(init_status)
  {
    mFoilboatController.start();
  }
  return EXIT_SUCCESS;
}