#include <cmath> // Needed for isnan() and cos()

#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"

#include <pid_ros_deb/controller/FoilboatController.hpp>

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
  // I don't know if there's a better way to initialize a lot of params than this mess

  map<std::string, float> roll_controller_param_map,
    pitch_controller_param_map,
    altitude_rate_controller_param_map,
    altitude_controller_param_map;

  if(n.getParam("pid_ros_deb/roll_controller/pidff", roll_controller_param_map))
  {
    this->roll_controller_config = convertPIDMapParamToStruct(roll_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::roll, roll_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /roll_controller/pidff");
    return false;
  }

  if(n.getParam("pid_ros_deb/pitch_controller/pidff", pitch_controller_param_map))
  {
    this->pitch_controller_config = convertPIDMapParamToStruct(pitch_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::pitch, pitch_controller_config);
  }

  // Initialize altitude rate controller PID
  if(n.getParam("pid_ros_deb/altitude_rate_controller/pidff", altitude_rate_controller_param_map))
  {
    this->altitude_rate_controller_config = convertPIDMapParamToStruct(altitude_rate_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude_rate, altitude_rate_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /altitude_rate_controller/pidff");
    return false;
  }

  // Initialize altitude controller PID
  if(n.getParam("pid_ros_deb/altitude_controller/pidff", altitude_controller_param_map))
  {
    this->altitude_controller_config = convertPIDMapParamToStruct(altitude_controller_param_map);
    controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude, altitude_controller_config);
  }
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/altitude_controller/pidff");
    return false;
  }

  string odom_topic_name, imu_topic_name, z_estimate_topic_name, laser_topic_name, target_topic_name, control_topic_name, state_topic_name;
  // Initialize IMU subscriber
  if(n.getParam("pid_ros_deb/imu_topic", imu_topic_name))
    imu_sub = n.subscribe(imu_topic_name, 1000, &FoilboatController::onImu, this);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/imu_topic");
  }

  // Initialize laser subscriber
  if(n.getParam("pid_ros_deb/laser_topic", laser_topic_name))
    laser_sub = n.subscribe(laser_topic_name, 1000, &FoilboatController::onLaser, this);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/laser_topic");
  }

  // Initialize z estimate subscriber
  if(n.getParam("pid_ros_deb/z_estimate_topic", z_estimate_topic_name))
    z_estimate_sub = n.subscribe(z_estimate_topic_name, 1000, &FoilboatController::onZEstimate, this);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/z_estimate_topic");
  }

  if(n.getParam("pid_ros_deb/odom_topic", odom_topic_name))
    odom_sub = n.subscribe(odom_topic_name, 1000, &FoilboatController::onOdom, this);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/odom_topic");
  }

  // Initialize target subscriber
  if(n.getParam("pid_ros_deb/target_topic", target_topic_name))
    target_sub = n.subscribe(target_topic_name, 1000, &FoilboatController::onTarget, this);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/target_topic");
    return false;
  }

  if(n.getParam("pid_ros_deb/control_topic", control_topic_name))
    control_pub = n.advertise<pid_ros_deb::FoilboatControl>(control_topic_name, 100);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/control_topic");
    return false;
  }

  if(n.getParam("pid_ros_deb/state_topic", state_topic_name))
    state_pub = n.advertise<pid_ros_deb::FoilboatState>(state_topic_name, 100);
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/state_topic");
    return false;
  }

  int frequency;
  if(n.getParam("pid_ros_deb/controller_frequency", frequency))
  {
    controller_frequency = ros::Rate(frequency);
    ROS_INFO("Controller frequency initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /pid_ros_deb/controller_frequency");
    return false;
  }

  return true;
}

void FoilboatController::start()
{
  // PID dynamic reconfigure server
  dynamic_reconfigure::Server<pid_ros_deb::ControllerConfig> gains_server;
  dynamic_reconfigure::Server<pid_ros_deb::ControllerConfig>::CallbackType onConfig_callback;
  onConfig_callback = boost::bind(&FoilboatController::onConfig, this, _1, _2);
  gains_server.setCallback(onConfig_callback);

  // Overwrite dynamic reconfigure's default PID default values with the values read from rosparam
  controller_pid.updatePID(PIDWrapper::ControllerEnum::roll, this->roll_controller_config);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude, this->altitude_controller_config);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::altitude_rate, this->altitude_rate_controller_config);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::pitch, this->pitch_controller_config);

  // Run control loop on timer
  ros::Timer control_timer = n.createTimer(controller_frequency, &FoilboatController::control, this);
  ros::spin(); // You spin me right round
}

void FoilboatController::control(const ros::TimerEvent &event)
{
  // Run control loop
  pid_ros_deb::FoilboatControl controlOut;

  // Only run control loop if we have valid a target and pose estimate
  if(
    !isnan(last_state.pitch) &&
    !isnan(last_state.roll) &&
    !isnan(last_state.yaw) &&
    !isnan(lastTarget->altitudeTarget) &&
    !isnan(lastTarget->rollTarget)
  )
  {
    pid_ros_deb::FoilboatState::ConstPtr current_state_ptr(new pid_ros_deb::FoilboatState(last_state));
    controlOut = controller_pid.control(lastTarget, current_state_ptr, ros::Time::now().toSec(), this->controller_mode);

    state_pub.publish(last_state);
    control_pub.publish(controlOut);
  }
  else
  {
    // ROS_INFO("Either your roll, pitch, yaw, pitchTarget, or rollTarget are NaN");
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f, pitchTarget: %f, rollTarget: %f",
      last_state.roll, last_state.pitch, last_state.yaw, lastTarget->altitudeTarget, lastTarget->rollTarget);
  }
  ROS_INFO("Controller mode: %d", this->controller_mode);
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

  last_state.pitch = last_pitch - 0.13;
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
  if (z_estimate < 35 && z_estimate > 0.21)
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

  last_state.pose = odomPtr->pose.pose;
  last_state.twist = odomPtr->twist.twist;

  // Update altitude and altitude rate estimate
  if(odomPtr->pose.pose.position.z - last_state.altitude != 0){
    last_state.altitudeRate = (odomPtr->pose.pose.position.z - last_state.altitude);
  }
  last_state.altitude = odomPtr->pose.pose.position.z;
//  last_state.altitudeRate = odomPtr->twist.twist.linear.z; // Maybe one day we will be able to use the actual z rate
}


// Plane pose/altitude target topic subscriber callback
void FoilboatController::onTarget(const pid_ros_deb::FoilboatTarget::ConstPtr& targetPtr)
{
  ROS_INFO("new target trim: %f", targetPtr->flapTrim);
  this->lastTarget = targetPtr;
}

// Dynamic reconfigure callback
void FoilboatController::onConfig(pid_ros_deb::ControllerConfig &config, uint32_t level)
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

  ROS_INFO("Pitch controller config: P: %f, I: %f, D: %f",
            config.p_pitch,
            config.i_pitch,
            config.d_pitch);
  PIDWrapper::PIDGains pitch_gains(config.p_pitch, config.i_pitch, config.d_pitch);
  controller_pid.updatePID(PIDWrapper::ControllerEnum::pitch, pitch_gains);

  this->controller_mode = config.mode;
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