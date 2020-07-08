#include "ros/ros.h"

#include <foilboat_controller/range_sensor_wrapper/RangeSensorProcessor.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_sensor_wrapper_node");
  ros::NodeHandle n;

  RangeSensorProcessor mRangeSensorProcessor(n);
  if(mRangeSensorProcessor.init())
  {
    mRangeSensorProcessor.run();
  }
  
  return EXIT_SUCCESS;
}