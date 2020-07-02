#include "clearpath_sc_ros/ClearpathDriver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Clearpath");
  ros::NodeHandle n("clearpath");

  ClearpathDriver mClearpathDriver(n);

  mClearpathDriver.init();

  ros::Rate r(10);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}