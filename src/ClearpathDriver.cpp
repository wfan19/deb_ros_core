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
  port.Adv.Attn.Enable(true);

  axes_list.push_back(new Axis("servo0", &n, &port.Nodes(0)));
  if (axes_list[0]->start() != 0)
    return -1;

  return 0;
}