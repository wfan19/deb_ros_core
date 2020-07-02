#include <clearpath_sc_ros/ClearpathDriver.hpp>

using namespace sFnd;
ClearpathDriver::ClearpathDriver(ros::NodeHandle nh)
{
  this->n = nh;
}

ClearpathDriver::~ClearpathDriver()
{
  ROS_INFO("Closing %d nodes", port_counter);
  for (int port_itr = 0; port_itr < port_counter; port_itr++)
  {
    IPort &current_port = mSysManager.Ports(port_itr);
    for(int node_itr = 0; node_itr < current_port.NodeCount(); node_itr++)
      current_port.Nodes(node_itr).EnableReq(false);
  }
}

int ClearpathDriver::init()
{
  std::vector<std::string> servo_port_names;
  SysManager::FindComHubPorts(servo_port_names);

  for(port_counter = 0; port_counter < servo_port_names.size(); port_counter++)
  {
    mSysManager.ComHubPort(port_counter, servo_port_names[port_counter].c_str());
  }

  if (port_counter <= 0)
  {
    ROS_ERROR("Failed to find any servo ports. Port counter = %d", port_counter);
    return -1;
  }

  mSysManager.PortsOpen(port_counter);
  ROS_INFO("Opened %d ports", port_counter);

  IPort &port = mSysManager.Ports(0);
  INode &node = port.Nodes(0);

  node.EnableReq(false);
  mSysManager.Delay(200);
  node.Status.AlertsClear();
  node.Motion.NodeStopClear();
  node.EnableReq(true);
  ROS_INFO("Servo enabled");

  // Wait for motion to be ready

  // Check homing
  ROS_INFO("Beginning homing");
  if (node.Motion.Homing.HomingValid())
  {
    node.Motion.Homing.Initiate();
    ros::Time homing_start = ros::Time::now();
    ros::Duration timeout(5);
    while(!node.Motion.Homing.WasHomed())
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

  motor_sub = n.subscribe("motor0/position", 100, &ClearpathDriver::onMotorCommand, this);

  return 0;
}

void ClearpathDriver::onMotorCommand(std_msgs::Float64::ConstPtr floatMsg)
{
  try
  {
    mSysManager.Ports(0).Nodes(0).Status.RT.Refresh();
    int buffer_available = mSysManager.Ports(0).Nodes(0).Status.RT.Value().cpm.MoveBufAvail;
    mSysManager.Ports(0).Nodes(0).Status.RT.Refresh();
    ROS_INFO("Buffer available? %s", buffer_available ? "true" : "false");
    if (buffer_available)
    {
      mSysManager.Ports(0).Nodes(0).Motion.MovePosnStart(floatMsg->data, true);

      
    }
    else
      ROS_INFO("Move buffer full, skipping move command");
  }
  catch (mnErr error)
  {
    ROS_ERROR("Error thrown while executing move command");
    ROS_ERROR("Error Address=%d, code=%08x\nmessage=%s", error.TheAddr, error.ErrorCode, error.ErrorMsg);
  }
}