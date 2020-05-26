#include "ros/ros.h"

#include <foilboat_controller/FoilboatController.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "foilboat_controller_node");
  ros::NodeHandle n;

  FoilboatController mFoilboatController(n);
  bool init_status = mFoilboatController.init();
  if(init_status)
  {
    mFoilboatController.control();
  }
  return EXIT_SUCCESS;
}