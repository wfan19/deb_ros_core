#ifndef PIDWRAPPER_HPP
#define PIDWRAPPER_HPP

#include "ros/ros.h"

#include "PIDFF.hpp"
#include <pid_ros_deb/FoilboatTarget.h>
#include <pid_ros_deb/FoilboatControl.h>
#include <pid_ros_deb/FoilboatState.h>

class PIDWrapper
{
public:
  struct PIDGains
  {
    double kp;
    double ki;
    double kd;

    PIDGains()
    {
      kp = 0;
      ki = 0;
      kd = 0;
    }

    PIDGains(double p, double i, double d)
    {
      kp = p;
      ki = i;
      kd = d;
    }
  };
  
  enum ControllerEnum
  {
    altitude_rate,
    altitude,
    pitch,
    roll,
  };

  PIDWrapper();
  ~PIDWrapper();

  void updatePID(ControllerEnum controller, PIDGains gains);
  void updatePID(ControllerEnum controller, PIDFF::PIDConfig config);
  void resetIntegrators();

  pid_ros_deb::FoilboatControl control(
    pid_ros_deb::FoilboatTarget::ConstPtr target,
    pid_ros_deb::FoilboatState::ConstPtr state,
    double time,
    int mode
  );

private:
  ros::NodeHandle n;

  PIDFF altitude_controller;
  PIDFF altitude_rate_controller;
  PIDFF roll_controller;
  PIDFF pitch_controller;
};

#endif