#ifndef SRC_CLEARPATHDRIVER_HPP
#define SRC_CLEARPATHDRIVER_HPP

#include <string>
#include <vector>

#include "ros/ros.h"
#include "pubSysCls.h"

using namespace sFnd;
class ClearpathDriver{
public:
  ClearpathDriver();
  ~ClearpathDriver();

  int init();

private:
  SysManager mSysManager;
  int port_counter;
};

#endif //SRC_CLEARPATHDRIVER_HPP
