#include <clearpath_sc_ros/Axis.hpp>


using namespace sFnd;
using namespace std;
Axis::Axis(string axis_name, ros::NodeHandle* driver_nh, INode* node):
  mNode(node)
{
  this->n = ros::NodeHandle(*driver_nh, axis_name);
}

Axis::~Axis()
{
  main_thread.~thread();
  position_thread.~thread();
  mNode->EnableReq(false);
}

int Axis::start()
{
  // Enable node
  mNode->EnableReq(false);
  ros::Duration(0.2).sleep();
  mNode->Status.AlertsClear();
  mNode->Motion.NodeStopClear();
  mNode->EnableReq(true);
  ROS_INFO("Servo enabled");

  // TODO: Add node ATTN mask initialization
  attnReg attn_init_mask;
  attn_init_mask.cpm.MoveDone = 1;
  attn_init_mask.cpm.Disabled = 1;
  attn_init_mask.cpm.NotReady = 1;
  mNode->Adv.Attn.Mask = attn_init_mask;

  // Check homing
  ROS_INFO("Beginning homing");
  if (mNode->Motion.Homing.HomingValid())
  {
    mNode->Motion.Homing.Initiate();
    ros::Time homing_start = ros::Time::now();
    ros::Duration timeout(5);
    while(!mNode->Motion.Homing.WasHomed())
    {
      if(ros::Time::now() > homing_start + timeout)
      {
        ROS_ERROR("Homing timed out!");
        return -1;
      }
    }
    ROS_INFO("Motor homing complete");
  }
  else
  {
    ROS_ERROR("Motor homing invalid");
    return -1;
  }

  // Start ros publishers
  position_state_pub = n.advertise<std_msgs::Float64>("position_state", 10);

  // Start ros subscribers
  position_target_sub = n.subscribe("position_target", 1, &Axis::onPositionTarget, this);

  // Start threads
  main_thread = thread(&Axis::mainLoop, this);
  position_thread = thread(&Axis::positionLoop, this);
  return 0;
}

void Axis::mainLoop()
{
  ros::Rate main_rate(10);
  while(ros::ok())
  {
    double current_position = mNode->Motion.PosnMeasured;
    std_msgs::Float64 position_state_msg;
    position_state_msg.data = current_position;
    position_state_pub.publish(position_state_msg);

    main_rate.sleep();
  }
  ros::shutdown();
}

void Axis::positionLoop()
{
  ros::Rate position_rate(500);
  while(ros::ok())
  {
    try
    {
      attnReg attn_mask, attn_read;
      attn_mask.cpm.MoveDone = 1;
      attn_mask.cpm.Disabled = 1;
      attn_mask.cpm.NotReady = 1;
      mNode->Adv.Attn.ClearAttn(attn_mask);

      int rem_buffer_slots = mNode->Motion.MovePosnStart(position_target, true);
      ROS_INFO("Remaining buffer slots: %d", rem_buffer_slots);
      double move_time = mNode->Motion.MovePosnDurationMsec(position_target, true);

      // Wait for motion to finish or abort
      attn_read = mNode->Adv.Attn.WaitForAttn(attn_mask, move_time + 20);
      if (attn_read.cpm.NotReady || attn_read.cpm.Disabled)
      {
        ROS_ERROR("Servo not ready or disabled!! Shutting down program");
        break;
      }
      else if (!attn_read.cpm.MoveDone)
      {
        ROS_ERROR("Movement wait timed out");
      }
      ROS_INFO("Move done? %s", attn_read.cpm.MoveDone ? "True" : "False");
    }
    catch (mnErr error)
    {
      ROS_ERROR("Error code: %08x", error.ErrorCode);
      ROS_ERROR("Message: %s", error.ErrorMsg);
      ros::Duration(0.5).sleep();
    }
  }
  ros::shutdown();
}

void Axis::onPositionTarget(const std_msgs::Float64::ConstPtr target_msg)
{
  this->position_target = target_msg->data;
}