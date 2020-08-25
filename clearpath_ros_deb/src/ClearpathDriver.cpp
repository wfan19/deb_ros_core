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
  try
  {
    std::vector<std::string> servo_port_names;
    SysManager::FindComHubPorts(servo_port_names);

    for(port_counter = 0; port_counter < servo_port_names.size(); port_counter++)
    {
      mSysManager.ComHubPort(port_counter, servo_port_names[port_counter].c_str());
    }
    ROS_INFO("Found %d ports", port_counter);

    if (port_counter <= 0)
    {
      ROS_ERROR("Failed to find any servo ports. Port counter = %d", port_counter);
      return -1;
    }

    mSysManager.PortsOpen(port_counter);
    ROS_INFO("Opened %d ports", port_counter);

    IPort &port = mSysManager.Ports(0);
    port.Adv.Attn.Enable(true);

    for (int i = 0; i < port.NodeCount(); i++) {
      // Fill axes_list with the axes(servos) connected to this port
      axes_list.push_back(new Axis("servo" + to_string(i), &n, &port.Nodes(i)));
      if (axes_list[i]->start() != 0)
      {
        ROS_ERROR("Servo %d failed to start!", i);
        return -1;
      }
    }
  }
  catch (mnErr error)
  {
    ROS_ERROR("mnErr thrown while looking for SC Hubs");
    ROS_ERROR("Error code: %08x", error.ErrorCode);
    ROS_ERROR("Message: %s", error.ErrorMsg);
  }

  return 0;
}