#include <cmath> // Needed for isnan() and cos()

#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"

#include <foilboat_controller/FoilboatController.hpp>

using namespace std;
FoilboatController::FoilboatController(ros::NodeHandle nh):
  controller_frequency{1}
{
  this->n = nh;
}

FoilboatController::~FoilboatController()
{
}

bool FoilboatController::init()
{
  // TODO: Rewrite all this initialization garbage
  // Something like initParams(string paramName, function callback){}
 
  map<std::string, float> roll_controller_param_map, pitch_controller_param_map, altitude_controller_param_map;
  if(n.getParam("foilboat_controller/roll_controller/pidff", roll_controller_param_map))
  {
    ROS_INFO("Found roll_controller pidff config");
    PIDFF::PIDConfig roll_controller_config = convertPIDMapParamToStruct(roll_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::roll, roll_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /roll_controller/pidff");
    return false;
  }

  // Initialize pitch controller PID
  if(n.getParam("foilboat_controller/pitch_controller/pidff", pitch_controller_param_map))
  {
    ROS_INFO("Found pitch_controller pidff config");
    PIDFF::PIDConfig pitch_controller_config = convertPIDMapParamToStruct(pitch_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::pitch, pitch_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /pitch_controller/pidff");
    return false;
  }

  // Initialize altitude controller PID
  if(n.getParam("foilboat_controller/altitude_controller/pidff", altitude_controller_param_map))
  {
    ROS_INFO("Found altitude_controller pidff config");
    PIDFF::PIDConfig altitude_controller_config = convertPIDMapParamToStruct(altitude_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude, altitude_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/altitude_cnontroller/pidff");
    return false;
  }

  string imu_topic_name, laser_topic_name, target_topic_name, control_topic_name;
  // Initialize IMU subscriber
  if(n.getParam("foilboat_controller/imu_topic", imu_topic_name))
  {
    imu_sub = n.subscribe(imu_topic_name, 1000, &FoilboatController::onImu, this);
    ROS_INFO("IMU subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/imu_topic");
    return false;
  }

  // Initialize laser subscriber
  if(n.getParam("foilboat_controller/laser_topic", laser_topic_name))
  {
    laser_sub = n.subscribe(laser_topic_name, 1000, &FoilboatController::onLaser, this);
    ROS_INFO("Laser subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/laser_topic");
    return false;
  }

  // Initialize target subscriber
  if(n.getParam("foilboat_controller/target_topic", target_topic_name))
  {
    target_sub = n.subscribe(target_topic_name, 1000, &FoilboatController::onTarget, this);
    ROS_INFO("Target subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/target_topic");
    return false;
  }

  if(n.getParam("foilboat_controller/control_topic", control_topic_name))
  {
    control_pub = n.advertise<foilboat_controller::FoilboatControl>(control_topic_name, 100);
    ROS_INFO("Control publisher initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/control_topic");
    return false;
  }

  int frequency;
  if(n.getParam("foilboat_controller/controller_frequency", frequency))
  {
    controller_frequency = ros::Rate(frequency);
    ROS_INFO("Controller frequency initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/controller_frequency");
    return false;
  }
  
  // Advertise the state publisher
  state_pub = n.advertise<foilboat_controller::FoilboatState>("/plane_ros/state", 100);

  return true;
}

void FoilboatController::start()
{
  // Initialize dynamic reconfigure server, and bind onPIDConfig to it as a callback
  dynamic_reconfigure::Server<foilboat_controller::GainsConfig> dynamic_reconfigure_server;
  dynamic_reconfigure::Server<foilboat_controller::GainsConfig>::CallbackType onPIDConfig_callback;
  onPIDConfig_callback = boost::bind(&FoilboatController::onPIDConfig, this, _1, _2);
  dynamic_reconfigure_server.setCallback(onPIDConfig_callback);
  
  ros::Timer control_timer = n.createTimer(controller_frequency, &FoilboatController::control, this);
  ros::spin();
}

void FoilboatController::control(const ros::TimerEvent &event)
{
  foilboat_controller::FoilboatControl controlOut;

  // Run control loop
    // TODO: Rotation matrix math is stinky
  tf2::Matrix3x3 quaternion_rotation_matrix(lastOrientation);
  double last_roll, last_pitch, last_yaw;
  quaternion_rotation_matrix.getRPY(last_roll, last_pitch, last_yaw);

  // Only run control loop if we have valid a target and pose estimate
  if(
    !isnan(last_roll) &&
    !isnan(last_pitch) &&
    !isnan(last_yaw) &&
    !isnan(lastTarget->pitchTarget) &&
    !isnan(lastTarget->rollTarget)
  )
  {
    foilboat_controller::FoilboatState current_state;
    current_state.roll = last_roll;
    current_state.pitch = last_pitch;
    if(lastLaser->ranges[0] < 35 && lastLaser->ranges[0] > 0)
    {
      current_state.altitude = lastLaser->ranges[0] * cos(last_pitch) * cos(last_roll);
    }

    foilboat_controller::FoilboatState::ConstPtr current_state_ptr(new foilboat_controller::FoilboatState(current_state));
    
    controlOut = controller_pid.control(lastTarget, current_state_ptr, ros::Time::now().toSec());

    ROS_INFO("Control out: Right foil: %f, Left Foil: %f, Elevator: %f", controlOut.rightFoil, controlOut.leftFoil, controlOut.elevatorFoil);

    state_pub.publish(current_state);
    control_pub.publish(controlOut);
  }
  else
  {
    // ROS_INFO("Either your roll, pitch, yaw, pitchTarget, or rollTarget are NaN");
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f, pitchTarget: %f, rollTarget: %f",
      last_roll, last_pitch, last_yaw, lastTarget->pitchTarget, lastTarget->rollTarget);
  }
}

// IMU topic subcriber callback
void FoilboatController::onImu(const sensor_msgs::Imu::ConstPtr& imuPtr)
{
  ROS_INFO("onIMU");
  geometry_msgs::Quaternion orientation = imuPtr->orientation;
  lastOrientation = tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  this->lastIMUMsg = imuPtr;
}

// Laser sensor topic subscriber callback
void FoilboatController::onLaser(const sensor_msgs::LaserScan::ConstPtr& laserPtr)
{
  this->lastLaser = laserPtr;
}


// Plane pose/altitude target topic subscriber callback
void FoilboatController::onTarget(const foilboat_controller::FoilboatTarget::ConstPtr& targetPtr)
{
  this->lastTarget = targetPtr;
}

// Dynamic reconfigure callback
void FoilboatController::onPIDConfig(foilboat_controller::GainsConfig &config, uint32_t level)
{
  controller_pid.resetIntegrators();

  ROS_INFO("=============== PID Configuration update: ===============");
  ROS_INFO("Pitch controller config: P: %f, I: %f, D: %f",
            config.p_pitch,
            config.i_pitch,
            config.d_pitch);
  PIDWrapper::PIDGains pitch_gains(config.p_pitch, config.i_pitch, config.d_pitch);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::pitch, pitch_gains);

  ROS_INFO("Roll controller config: P: %f, I: %f, D: %f",
            config.p_roll,
            config.i_roll,
            config.d_roll);
  PIDWrapper::PIDGains roll_gains(config.p_roll, config.i_roll, config.d_roll);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::roll, roll_gains);
}

// Convert an array of PIDFF controller parameters
// into a PIDFF::PIDConfig configuration struct
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