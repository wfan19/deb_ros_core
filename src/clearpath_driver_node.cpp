#include "clearpath_sc_ros/ClearpathDriver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clearpath");
  ros::NodeHandle n("clearpath");

  ClearpathDriver mClearpathDriver(n);

  int initialization_status = mClearpathDriver.init();
  if(initialization_status < 0)
    ros::shutdown();

  ros::Rate r(10);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}