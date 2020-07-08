#include <clearpath_sc_ros/Axis.hpp>

using namespace sFnd;
using namespace std;
Axis::Axis(string axis_name, ros::NodeHandle* driver_nh, INode* node):
  mNode(node)
{
  this->n = ros::NodeHandle(*driver_nh, axis_name);
  this->axis_name = axis_name;
}

// Shutdown servo upon destroy
Axis::~Axis()
{
  status_thread.~thread();
  position_thread.~thread();
  mNode->EnableReq(false);
}

// Initialization
int Axis::start()
{
  // Enable node
  mNode->EnableReq(false);
  ros::Duration(0.2).sleep();
  mNode->Status.AlertsClear();
  mNode->Motion.NodeStopClear();
  mNode->EnableReq(true);

  ros::Time enable_start = ros::Time::now();
  while(!mNode->Motion.IsReady())
  {
    if(ros::Time::now().toSec() - enable_start.toSec() > 2)
    {
      ROS_ERROR("%s timed out on enabling servo", axis_name.c_str());
      return -1;
    }
  }

  ROS_INFO("Servo enabled");

  // Initialize Attn mask
  // Tells servo which attn msgs to send via SDK
  attnReg attn_init_mask;
  attn_init_mask.cpm.MoveDone = 1;
  attn_init_mask.cpm.Disabled = 1;
  attn_init_mask.cpm.NotReady = 1;
  mNode->Adv.Attn.Mask = attn_init_mask;

  // Start ros publishers
  servo_state_pub = n.advertise<clearpath_sc_ros::ServoState>("state", 10);

  // Start ros subscribers
  position_target_sub = n.subscribe("position_target", 1, &Axis::onPositionTarget, this);

  getConfig_service = n.advertiseService("get_config", &Axis::getConfig, this);
  homeAxis_service = n.advertiseService("home_axis", &Axis::homeAxis, this);

  // Start threads
  status_thread = thread(&Axis::statusLoop, this);
  position_thread = thread(&Axis::positionLoop, this);
  return 0;
}

// Main status publishing thread loop
void Axis::statusLoop()
{
  ros::Rate status_rate(10);
  while(ros::ok())
  {
    mNode->Motion.PosnMeasured.Refresh();
    mNode->Motion.VelMeasured.Refresh();

    clearpath_sc_ros::ServoState servo_state;
    servo_state.position = mNode->Motion.PosnMeasured;
    servo_state.velocity = mNode->Motion.VelMeasured;

    servo_state.error_code = this->last_error_code;

    servo_state_pub.publish(servo_state);

    status_rate.sleep();
  }
  ros::shutdown();
}

// Position control thread loop
void Axis::positionLoop()
{
  while(ros::ok())
  {
    try
    {
      if (
          mNode->Motion.Homing.HomingValid() // Check for valid homing before setting move command
          && !mNode->Motion.Homing.IsHoming() // Make sure we aren't homing right now
      )
      {
        attnReg attn_mask, attn_read;
        attn_mask.cpm.MoveDone = 1;
        attn_mask.cpm.Disabled = 1;
        attn_mask.cpm.NotReady = 1;
        mNode->Adv.Attn.ClearAttn(attn_mask);

        int rem_buffer_slots = mNode->Motion.MovePosnStart(position_target, true);
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
        this->last_error_code = -1;
      }
      else
      {
        ROS_ERROR("Servo homing is invalid or is currently homing");
        ros::Duration(0.2).sleep();
      }
    }
    catch (mnErr error)
    {
      ROS_ERROR("Error code: %08x", error.ErrorCode);
      ROS_ERROR("Message: %s", error.ErrorMsg);
      this->last_error_code = error.ErrorCode;
      ros::Duration(0.5).sleep();
    }
  }
  ros::shutdown();
}

// Position target subscriber
void Axis::onPositionTarget(const std_msgs::Float64::ConstPtr target_msg)
{
  this->position_target = target_msg->data;
}

// ==================
//     Services
// ==================

// Fetch the axis' configuration and return it in the response
bool Axis::getConfig(
    clearpath_sc_ros::GetConfig::Request &req,
    clearpath_sc_ros::GetConfig::Response &res
)
{
  clearpath_sc_ros::ServoConfig current_config;

  mNode->Limits.TrqGlobal.Refresh();
  mNode->Limits.SoftLimit1.Refresh();
  mNode->Limits.SoftLimit2.Refresh();

  mNode->Info.PositioningResolution.Refresh();

  if (mNode->TrqUnit() == mNode->PCT_MAX)
    current_config.torque_unit = current_config.PCT_MAX;
  else if (mNode->TrqUnit() == mNode->AMPS)
    current_config.torque_unit = current_config.AMPS;

  current_config.torque_limit = mNode->Limits.TrqGlobal;
  current_config.soft_limit_1 = mNode->Limits.SoftLimit1;
  current_config.soft_limit_2 = mNode->Limits.SoftLimit2;
  current_config.encoder_cpr = mNode->Info.PositioningResolution;

  res.current_config = current_config;
  return true;
}

// Home the axis based on Clearview's configuration
bool Axis::homeAxis(
    clearpath_sc_ros::HomeAxis::Request &req,
    clearpath_sc_ros::HomeAxis::Response &res
)
{
  ROS_INFO("homeAxis(): Beginning homing");
  try
  {
    if (mNode->Motion.Homing.HomingValid())
    {
      mNode->Motion.Homing.Initiate();
      ros::Time homing_start = ros::Time::now();
      ros::Duration timeout(5);
      while(!mNode->Motion.Homing.WasHomed())
      {
        if(ros::Time::now() > homing_start + timeout)
        {
          ROS_ERROR("homeAxis(): Homing timed out!");
          return false;
        }
      }
      ROS_INFO("homeAxis(): Motor homing complete");
      ros::Duration(0.2).sleep();
    }
    else
    {
      ROS_ERROR("homeAxis(): Motor homing invalid");
      return false;
    }
  }
  catch (mnErr error)
  {
    ROS_ERROR("homeAxis(): Error code: %08x while homing", error.ErrorCode);
    return false;
  }
  return true;
}