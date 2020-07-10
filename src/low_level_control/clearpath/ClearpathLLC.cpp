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

  return 0;
}

void ClearpathLLC::onControl(const fcs_ros_deb::FoilboatControl::ConstPtr)
{
}