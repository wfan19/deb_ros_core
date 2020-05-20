#include "geometry_msgs/Quaternion.h"

#include <foilboat_controller/FoilboatController.hpp>

using namespace std;
FoilboatController::FoilboatController(ros::NodeHandle nh, int rate):
  controllerRate(rate)
{
  this->n = nh;

  map<std::string, float> roll_controller_param_map, pitch_controller_param_map;
  
  if(n.getParam("foilboat_controller/roll_controller/pidff", roll_controller_param_map))
  {
    ROS_INFO("Found roll_controller pidff config");
    PIDFF::PIDConfig roll_controller_config = convertPIDMapParamToStruct(roll_controller_param_map);
    roll_controller = PIDFF(roll_controller_config);
  }
  else
  {
    ROS_INFO("Failed to find param /roll_controller/pidff");
  }

  // Initialize pitch controller PID
  if(n.getParam("foilboat_controller/pitch_controller/pidff", pitch_controller_param_map))
  {
    ROS_INFO("Found pitch_controller pidff config");
    PIDFF::PIDConfig pitch_controller_config = convertPIDMapParamToStruct(pitch_controller_param_map);
    pitch_controller = PIDFF(pitch_controller_config);
  }
  else
  {
    ROS_INFO("Failed to find param /pitch_controller/pidff");
  }

  string imu_topic_name, laser_topic_name, target_topic_name, control_topic_name;
  // Initialize IMU subscriber
  if(n.getParam("foilboat_controller/imu_topic", imu_topic_name))
  {
    imu_sub = n.subscribe(imu_topic_name, 1000, &FoilboatController::onImu, this);
    ROS_INFO("IMU subscriber initialized");
  }

  // Initialize laser subscriber
  if(n.getParam("foilboat_controller/laser_topic", laser_topic_name))
  {
    laser_sub = n.subscribe(laser_topic_name, 1000, &FoilboatController::onLaser, this);
    ROS_INFO("Laser subscriber initialized");
  }

  // Initialize target subscriber
  if(n.getParam("foilboat_controller/target_topic", target_topic_name))
  {
    target_sub = n.subscribe(target_topic_name, 1000, &FoilboatController::onTarget, this);
  }

  if(n.getParam("foilboat_controller/control_topic", control_topic_name))
  {
    control_pub = n.advertise<foilboat_controller::FoilboatControl>(control_topic_name, 100);
  }
}

FoilboatController::~FoilboatController()
{
}

void FoilboatController::control()
{
  foilboat_controller::FoilboatControl controlOut;
  // Rotation matrix math is stinky
  tf2::Matrix3x3 quaternion_rotation_matrix(lastOrientation);
  float last_roll, last_pitch, last_yaw;
  quaternion_rotation_matrix.getRPY(last_roll, last_pitch, last_yaw)

  float roll_error = lastTarget->rollTarget - last_roll;
  rollControl = roll_controller.update(roll_error, lastTarget->rollTarget, ros::Time::now().toSec());
  controlOut.rightFoil = 0 + rollControl;
  controlOut.leftFoil = 0 - rollControl;

  controlOut.elevatorFoil = 0;

}

void FoilboatController::onImu(const sensor_msgs::Imu::ConstPtr& imuPtr)
{
  geometry_msgs::Quaternion orientation = imuPtr->orientation;
  lastOrientation = tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  this->lastIMUMsg = imuPtr;
  control();
}

void FoilboatController::onLaser(const sensor_msgs::LaserScan::ConstPtr& laserPtr)
{
  this->lastLaser = laserPtr;
  control();
}

void FoilboatController::onTarget(const foilboat_controller::FoilboatTarget::ConstPtr& targetPtr)
{
  this->lastTarget = targetPtr;
  control();
}

PIDFF::PIDConfig FoilboatController::convertPIDMapParamToStruct(map<string, float> pidConfigMap)
{
  map<string, float>::iterator configIterator;
  PIDFF::PIDConfig config_out;
  for (configIterator = pidConfigMap.begin(); configIterator != pidConfigMap.end(); configIterator++)
  {
    // ROS_INFO("Reading PID parameter <%s, %f>", configIterator->first.c_str(), configIterator->second);
    
    // TODO: This is hideous. Refactor when you get the chance
    if(configIterator->first == "p")
      config_out.kp = configIterator->second;
    else if(configIterator->first == "i")
      config_out.ki = configIterator->second;
    else if(configIterator->first == "d")
      config_out.kd = configIterator->second;
    else if(configIterator->first == "ff")
      config_out.kff = configIterator->second;
    else if(configIterator->first == "i_min")
      config_out.imin = configIterator->second;
    else if(configIterator->first == "i_max")
      config_out.imax = configIterator->second;
    else if(configIterator->first == "min")
      config_out.min = configIterator->second;
    else if(configIterator->first == "max")
      config_out.max = configIterator->second;
    else
      ROS_ERROR("Unknown parameter %s", configIterator->first.c_str());

  }
  return config_out;
}