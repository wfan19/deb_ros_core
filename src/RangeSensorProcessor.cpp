#include <foilboat_controller/RangeSensorProcessor.hpp>

RangeSensorProcessor::RangeSensorProcessor(ros::NodeHandle nh)
{
  this->n = nh;

  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/plane_ros/z_estimate", 100);
  
  ROS_INFO("Initializing subscribers");
  state_sub = n.subscribe("/plane_ros/state", 1000, &RangeSensorProcessor::onState, this);
  laser_sub = n.subscribe("/plane_ros/lidar", 1000, &RangeSensorProcessor::onLaser, this);
}

RangeSensorProcessor::~RangeSensorProcessor()
{
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
  ROS_INFO("onLaser");
  boost::array<float, 36> covariance;
  geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_stamped_out;

  float z_measured = laserPtr->ranges[0] * cos(last_state->roll) * cos(last_state->pitch);
  for(int i = 0; i < 36; i++)
  {
    covariance[i] = 0;
  }
  covariance[14] = 0.04 * cos(last_state->roll) * cos(last_state->pitch);

  pose_with_covariance_stamped_out.header.stamp = ros::Time::now();
  pose_with_covariance_stamped_out.pose.pose.position.z = laserPtr->ranges[0] * cos(last_state->roll) * cos(last_state->pitch);
  pose_with_covariance_stamped_out.pose.covariance = covariance;

  ROS_INFO("Publishing pose with z %f", z_measured);

  pose_pub.publish(pose_with_covariance_stamped_out);
}