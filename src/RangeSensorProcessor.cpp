#include <foilboat_controller/RangeSensorProcessor.hpp>

RangeSensorProcessor::RangeSensorProcessor(ros::NodeHandle nh)
{
  this->n = nh;

  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("z_estimate", 100);
  
  ros::Subscriber state_sub = n.subscribe("/plane_ros/state", 100, &RangeSensorProcessor::onState, this);
  ros::Subscriber laser_sub = n.subscribe("/plane_ros/laser", 100, &RangeSensorProcessor::onLaser, this);
}

RangeSensorProcessor::~RangeSensorProcessor()
{
}

void RangeSensorProcessor::run()
{
  ros::spin();
}

void RangeSensorProcessor::onState(const foilboat_controller::FoilboatState::ConstPtr statePtr)
{
  last_state = statePtr;
}

void RangeSensorProcessor::onLaser(const sensor_msgs::LaserScan::ConstPtr laserPtr)
{
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