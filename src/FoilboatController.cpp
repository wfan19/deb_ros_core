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
 
  map<std::string, float> roll_controller_param_map, altitude_rate_controller_param_map, altitude_controller_param_map;
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
  if(n.getParam("foilboat_controller/altitude_rate_controller/pidff", altitude_rate_controller_param_map))
  {
    ROS_INFO("Found altitude_rate pidff config");
    PIDFF::PIDConfig altitude_rate_controller_config = convertPIDMapParamToStruct(altitude_rate_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude_rate, altitude_rate_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /altitude_rate_controller/pidff");
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
    ROS_ERROR("Failed to find param /foilboat_controller/altitude_controller/pidff");
    return false;
  }

  string odom_topic_name, imu_topic_name, z_estimate_topic_name, laser_topic_name, target_topic_name, control_topic_name, state_topic_name;
  // Initialize IMU subscriber
  if(n.getParam("foilboat_controller/imu_topic", imu_topic_name))
  {
    imu_sub = n.subscribe(imu_topic_name, 1000, &FoilboatController::onImu, this);
    ROS_INFO("IMU subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/imu_topic");
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
  }

  // Initialize z estimate subscriber
  if(n.getParam("foilboat_controller/z_estimate_topic", z_estimate_topic_name))
  {
    z_estimate_sub = n.subscribe(z_estimate_topic_name, 1000, &FoilboatController::onZEstimate, this);
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/z_estimate_topic");
  }

  if(n.getParam("foilboat_controller/odom_topic", odom_topic_name))
  {
    odom_sub = n.subscribe(odom_topic_name, 1000, &FoilboatController::onOdom, this);
    ROS_INFO("Odom subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/odom_topic");
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

  if(n.getParam("foilboat_controller/state_topic", state_topic_name))
  {
    state_pub = n.advertise<foilboat_controller::FoilboatState>(state_topic_name, 100);
    ROS_INFO("State publisher initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/state_topic");
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
  // Run control loop
  foilboat_controller::FoilboatControl controlOut;

  // Only run control loop if we have valid a target and pose estimate
  if(
    !isnan(last_state.pitch) &&
    !isnan(last_state.roll) &&
    !isnan(last_state.yaw) &&
    !isnan(lastTarget->altitudeTarget) &&
    !isnan(lastTarget->rollTarget)
  )
  {
    foilboat_controller::FoilboatState::ConstPtr current_state_ptr(new foilboat_controller::FoilboatState(last_state));
    
    controlOut = controller_pid.control(lastTarget, current_state_ptr, ros::Time::now().toSec());

    ROS_INFO("Control out: Right foil: %f, Left Foil: %f, Elevator: %f", controlOut.rightFoil, controlOut.leftFoil, controlOut.elevatorFoil);

    state_pub.publish(last_state);
    control_pub.publish(controlOut);
  }
  else
  {
    // ROS_INFO("Either your roll, pitch, yaw, pitchTarget, or rollTarget are NaN");
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f, pitchTarget: %f, rollTarget: %f",
      last_state.roll, last_state.pitch, last_state.yaw, lastTarget->altitudeTarget, lastTarget->rollTarget);
  }
}

// IMU topic subcriber callback
void FoilboatController::onImu(const sensor_msgs::Imu::ConstPtr& imuPtr)
{
  ROS_INFO("onIMU");
  geometry_msgs::Quaternion orientation = imuPtr->orientation;
  lastOrientation = tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

  tf2::Matrix3x3 quaternion_rotation_matrix(lastOrientation);
  double last_roll, last_pitch, last_yaw;
  quaternion_rotation_matrix.getRPY(last_roll, last_pitch, last_yaw);

  last_state.pitch = last_pitch;
  last_state.roll = last_roll;
  last_state.yaw = last_yaw;
}

// Laser sensor topic subscriber callback
void FoilboatController::onLaser(const sensor_msgs::LaserScan::ConstPtr& laserPtr)
{
  if(laserPtr->ranges[0] < 35 && laserPtr->ranges[0] > 0)
  {
    float z_estimate = laserPtr->ranges[0] * cos(last_state.pitch) * cos(last_state.roll);
    ROS_INFO("altitudeRate = (%f - %f) / (%f - %f)",
      z_estimate,
      last_state.altitude,
      ros::Time::now().toSec(),
      last_laser_time.toSec());
    last_state.altitudeRate = (z_estimate - last_state.altitude);// - (ros::Time::now().toSec() - last_laser_time.toSec());
    last_state.altitude = z_estimate;
    last_laser_time = ros::Time::now();
  }
}

void FoilboatController::onZEstimate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr)
{
  double z_estimate = posePtr->pose.pose.position.z;
  if (z_estimate < 35 && z_estimate > 0)
  {
    last_state.altitudeRate = (z_estimate - last_state.altitude);// - (ros::Time::now().toSec() - last_laser_time.toSec());
    last_state.altitude = z_estimate;
    last_laser_time = ros::Time::now();
  }
}

void FoilboatController::onOdom(const nav_msgs::Odometry::ConstPtr& odomPtr)
{
  ROS_INFO("onOdom");
  // Update current orientation estimate
  geometry_msgs::Quaternion orientation = odomPtr->pose.pose.orientation;
  lastOrientation = tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

  // Convert quat to rpy
  tf2::Matrix3x3 quaternion_rotation_matrix(lastOrientation);
  double last_roll, last_pitch, last_yaw;
  quaternion_rotation_matrix.getRPY(last_roll, last_pitch, last_yaw);

  last_state.pitch = last_pitch;
  last_state.roll = last_roll;
  last_state.yaw = last_yaw;

  // Update altitude and altitude rate estimate
  // Commented until I can fix Z position localization
  // last_state.altitude = odomPtr->pose.pose.position.z;
  // last_state.altitudeRate = odomPtr->twist.twist.linear.z;
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

  ROS_INFO("Altitude controller config: P: %f, I: %f, D: %f",
            config.p_altitude,
            config.i_altitude,
            config.d_altitude);
  PIDWrapper::PIDGains altitude_gains(config.p_altitude, config.i_altitude, config.d_altitude);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude, altitude_gains);

  ROS_INFO("Altitude rate controller config: P: %f, I: %f, D: %f",
            config.p_altitude_rate,
            config.i_altitude_rate,
            config.d_altitude_rate);
  PIDWrapper::PIDGains altitude_rate_gains(config.p_altitude_rate, config.i_altitude_rate, config.d_altitude_rate);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude_rate, altitude_rate_gains);

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