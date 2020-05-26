#ifndef PIDWRAPPER_HPP
#define PIDWRAPPER_HPP

#include "ros/ros.h"

#include <foilboat_controller/PIDFF.hpp>
#include <foilboat_controller/FoilboatTarget.h>
#include <foilboat_controller/FoilboatControl.h>
#include <foilboat_controller/FoilboatState.h>

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
    altitude,
    pitch,
    roll,
  };

  PIDWrapper();
  ~PIDWrapper();

  void updatePID(ControllerEnum controller, PIDGains gains);
  void updatePID(ControllerEnum controller, PIDFF::PIDConfig config);
  void resetIntegrators();

  foilboat_controller::FoilboatControl control(
    foilboat_controller::FoilboatTarget::ConstPtr target,
    foilboat_controller::FoilboatState::ConstPtr state,
    double time
  );

private:
  ros::NodeHandle n;

  PIDFF altitude_controller;
  PIDFF pitch_controller;
  PIDFF roll_controller;
};

#endif