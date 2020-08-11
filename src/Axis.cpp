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

  mNode->Limits.SoftLimit1.Refresh();
  mNode->Limits.SoftLimit2.Refresh();
  if (int(mNode->Limits.SoftLimit1) > int(mNode->Limits.SoftLimit2))
  {
    this->max_position = mNode->Limits.SoftLimit1;
    this->min_position = mNode->Limits.SoftLimit2;
  }
  else if (int(mNode->Limits.SoftLimit1) < int(mNode->Limits.SoftLimit2))
  {
    this->max_position = mNode->Limits.SoftLimit2;
    this->min_position = mNode->Limits.SoftLimit1;
  }
  ROS_INFO("Min/Max: [%d, %d]", min_position, max_position);

  // Start ros publishers
  servo_state_pub = n.advertise<clearpath_sc_ros::ServoState>("state", 10);

  // Start ros subscribers
  position_target_sub = n.subscribe("position_target", 1, &Axis::onPositionTarget, this);

  getConfig_service = n.advertiseService("get_config", &Axis::getConfig, this);
  homeAxis_service = n.advertiseService("home_axis", &Axis::homeAxis, this);
  clearAlert_service = n.advertiseService("clear_alert", &Axis::clearAlert, this);

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
}

// Position control thread loop
void Axis::positionLoop()
{
  while(ros::ok())
  {
    try
    {
      if (mNode->Motion.Homing.HomingValid()) // Check for valid homing before setting move command
      {
        if (!mNode->Motion.Homing.IsHoming())
        {
          attnReg attn_mask, attn_read;
          attn_mask.cpm.MoveDone = 1;
          attn_mask.cpm.Disabled = 1;
          attn_mask.cpm.NotReady = 1;
          mNode->Adv.Attn.ClearAttn(attn_mask);

          ROS_INFO("[%s] Max: %d, min: %d, current position target: %d", axis_name.c_str(), this->max_position, this->min_position, position_target);
          if (position_target > max_position)
            position_target = max_position - 1;
          else if (position_target < min_position)
            position_target = min_position + 1;

          int rem_buffer_slots = mNode->Motion.MovePosnStart(position_target, true);
          double move_time = mNode->Motion.MovePosnDurationMsec(position_target, true);

          // Wait for motion to finish or abort
          attn_read = mNode->Adv.Attn.WaitForAttn(attn_mask, move_time + 20);
          if (attn_read.cpm.NotReady || attn_read.cpm.Disabled)
          {
            ROS_ERROR("[%s] Servo not ready or disabled!! Shutting down position thread", axis_name.c_str());
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
          ROS_ERROR("[%s] Currently homing, waiting for homing to complete", axis_name.c_str());
          while(mNode->Motion.Homing.IsHoming())
          {
            ros::Duration(0.5).sleep();
          }
        }
      }
      else
      {
        ROS_ERROR("[%s] Servo homing is invalid", axis_name.c_str());
        return;
      }
    }
    catch (mnErr error)
    {
      ROS_ERROR("[%s] Error code: %08x", axis_name.c_str(), error.ErrorCode);
      ROS_ERROR("[%s] Message: %s", axis_name.c_str(), error.ErrorMsg);
      this->last_error_code = error.ErrorCode;
      ros::Duration(0.5).sleep();
    }
  }
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
  current_config.encoder_cpr = mNode->Info.PositioningResolution;

  if (int(mNode->Limits.SoftLimit1) > int(mNode->Limits.SoftLimit2))
  {
    this->max_position = mNode->Limits.SoftLimit1;
    this->min_position = mNode->Limits.SoftLimit2;
    current_config.max_position = mNode->Limits.SoftLimit1;
    current_config.min_position = mNode->Limits.SoftLimit2;
  }
  else if (int(mNode->Limits.SoftLimit1) < int(mNode->Limits.SoftLimit2))
  {
    this->max_position = mNode->Limits.SoftLimit2;
    this->min_position = mNode->Limits.SoftLimit1;
    current_config.max_position = mNode->Limits.SoftLimit2;
    current_config.min_position = mNode->Limits.SoftLimit1;
  }

  res.current_config = current_config;
  return true;
}

// Home the axis based on Clearview's configuration
bool Axis::homeAxis(
    clearpath_sc_ros::HomeAxis::Request &req,
    clearpath_sc_ros::HomeAxis::Response &res
)
{
  ROS_INFO("[%s] homeAxis(): Beginning homing", axis_name.c_str());
  try
  {
    if (mNode->Motion.Homing.HomingValid())
    {
      mNode->Motion.Homing.Initiate();
      ros::Time homing_start = ros::Time::now();
      ros::Duration timeout(15);
      while(!mNode->Motion.Homing.WasHomed())
      {
        if(ros::Time::now() > homing_start + timeout)
        {
          ROS_ERROR("[%s] homeAxis(): Homing timed out!", axis_name.c_str());
          return false;
        }
      }
      ROS_INFO("[%s] homeAxis(): Motor homing complete", axis_name.c_str());
      ros::Duration(0.2).sleep();
    }
    else
    {
      ROS_ERROR("[%s] homeAxis(): Motor homing invalid", axis_name.c_str());
      return false;
    }
  }
  catch (mnErr error)
  {
    ROS_ERROR("[%s] homeAxis(): Error code: %08x while homing", axis_name.c_str(), error.ErrorCode);
    return false;
  }
  return true;
}

bool Axis::clearAlert(
    clearpath_sc_ros::ClearAlerts::Request &req,
    clearpath_sc_ros::ClearAlerts::Response &res
)
{
  // Enable node
  mNode->EnableReq(false);
  ros::Duration(0.2).sleep();
  mNode->Status.AlertsClear();
  mNode->Motion.NodeStopClear();
  mNode->EnableReq(true);
  
  if(!position_thread.joinable())
  {
    ROS_INFO("Restarting position thread");
    position_thread = thread(&Axis::positionLoop, this);
  }
  return true;
}i tentatively plan on starting a spotify fam with on campus folks if there’s enough interest? so if you can’t get it to work you’re welcome to join if you want
