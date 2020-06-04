#include <string>

#include <foilboat_controller/RangeSensorProcessor.hpp>

RangeSensorProcessor::RangeSensorProcessor(ros::NodeHandle nh)
{
  this->n = nh;
}

RangeSensorProcessor::~RangeSensorProcessor()
{
}

bool RangeSensorProcessor::init()
{
  ROS_INFO("[RangeSensorProcessor]: Initializing subscribers");
  std::string laser_topic_name, float_topic_name, state_topic_name, z_estimate_topic_name;
  
  // Initialize range sensor subscriber, either laser or float
  if(n.getParam("range_sensor_wrapper/laser_topic", laser_topic_name))
  {
    laser_sub = n.subscribe(laser_topic_name, 1000, &RangeSensorProcessor::onLaser, this);
    ROS_INFO("Laser subscriber initialized");
  }
  else if (n.getParam("range_sensor_wrapper/float_topic", float_topic_name))
  {
    float_sub = n.subscribe(float_topic_name, 1000, &RangeSensorProcessor::onFloat, this);
    ROS_INFO("Float subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find either param /range_sensor_wrapper/laser_topic or /range_sensor_wrapper/float_topic");
    return false; 
  }

  // Initialize state subscriber
  if(n.getParam("foilboat_controller/state_topic", state_topic_name))
  {
    state_sub = n.subscribe(state_topic_name, 1000, &RangeSensorProcessor::onState, this);
    ROS_INFO("State subscriber initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /foilboat_controller/state_topic");
    return false;
  }

  // Initialize z estimate publisher
  if(n.getParam("range_sensor_wrapper/z_estimate_topic", z_estimate_topic_name))
  {
    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(z_estimate_topic_name, 100);
    ROS_INFO("Z_estimate publisher initialized");
  }
  else
  {
    ROS_ERROR("Failed to find param /range_sensor_wrapper/z_estimate_topic");
    return false;
  }
  return true;
}

void RangeSensorProcessor::run()
{
  ROS_INFO("Spinning node");
  ros::spin();
}

void RangeSensorProcessor::onState(const foilboat_controller::FoilboatState::ConstPtr statePtr)
{
  last_state = statePtr;
}

void RangeSensorProcessor::onLaser(const sensor_msgs::LaserScan::ConstPtr laserPtr)
{
  publishMsg(laserPtr->ranges[0]);
}

void RangeSensorProcessor::onFloat(const std_msgs::Float64::ConstPtr floatPtr)
{
  publishMsg(floatPtr->data);
}

void RangeSensorProcessor::publishMsg(double data)
{
  boost::array<float, 36> covariance;
  geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_stamped_out;

  float z_measured = data * cos(last_state->roll) * cos(last_state->pitch) * 0.0254;
  for(int i = 0; i < 36; i++)
  {
    covariance[i] = 0;
  }
  covariance[14] = 0.04 * cos(last_state->roll) * cos(last_state->pitch);

  pose_with_covariance_stamped_out.header.stamp = ros::Time::now();
  pose_with_covariance_stamped_out.header.frame_id = "toughsonic_link";
  pose_with_covariance_stamped_out.pose.pose.position.z = z_measured;
  pose_with_covariance_stamped_out.pose.covariance = covariance;

  pose_pub.publish(pose_with_covariance_stamped_out);
}