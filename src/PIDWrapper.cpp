#include <foilboat_controller/PIDWrapper.hpp>

PIDWrapper::PIDWrapper()
{
  // this->n = nh;
  altitude_controller = PIDFF();
  altitude_rate_controller = PIDFF();
  roll_controller = PIDFF();
}

PIDWrapper::~PIDWrapper()
{
}

void PIDWrapper::updatePID(ControllerEnum controller, PIDGains gains)
{
  switch(controller)
  {
    case altitude:
      ROS_INFO("Setting altitude PID");
      altitude_controller.setKP(gains.kp);
      altitude_controller.setKI(gains.ki);
      altitude_controller.setKI(gains.ki);
      break;
    case altitude_rate:
      ROS_INFO("Setting altitude rate PID");
      altitude_rate_controller.setKP(gains.kp);
      altitude_rate_controller.setKI(gains.ki);
      altitude_rate_controller.setKI(gains.ki);
      break;
    case roll:
      ROS_INFO("Setting roll PID");
      roll_controller.setKP(gains.kp);
      roll_controller.setKI(gains.ki);
      roll_controller.setKI(gains.ki);
      break;
  }
}

void PIDWrapper::updatePID(ControllerEnum controller, PIDFF::PIDConfig config)
{
  switch(controller)
  {
    case altitude:
      altitude_controller.init(config);
      break;
    case altitude_rate:
      altitude_rate_controller.init(config);
      break;
    case roll:
      roll_controller.init(config);
      break;
  }
}

void PIDWrapper::resetIntegrators()
{
  altitude_controller.resetIntegrator();
  altitude_rate_controller.resetIntegrator();
  roll_controller.resetIntegrator();
}

foilboat_controller::FoilboatControl PIDWrapper::control(
  foilboat_controller::FoilboatTarget::ConstPtr target,
  foilboat_controller::FoilboatState::ConstPtr state,
  double time
)
{
  foilboat_controller::FoilboatControl control_out;
  ROS_INFO("========== Control loop ==========");
  float flap_control = 0;
  if(target->altitudeTarget > 0)
  {
    float altitude_rate_target = altitude_controller.update(target->altitudeTarget, state->altitude, time);
    ROS_INFO("Altitude rate target: %f, Altitude target: %f, Altitude state: %f", altitude_rate_target, target->altitudeTarget, state->altitude);
    control_out.altitudeRateTarget = altitude_rate_target;
    flap_control = -altitude_rate_controller.update(altitude_rate_target, state->altitudeRate, time);
    ROS_INFO("Target flaps: %f, Altitude rate target: %f, Altitude rate state: %f", flap_control, altitude_rate_target, state->altitudeRate);
  }

  // Manual elevator control:
  float elevator_control = target->elevatorTarget;

  // Pitch angle control:
  // float pitch_control = -pitch_controller.update(target_pitch, state->pitch, time) * 30 * 3.14 / 180;
  // ROS_INFO("Pitch control: %f, Pitch target: %f, Pitch state: %f, time: %f", pitch_control, target_pitch, state->pitch, time);

  float roll_control = roll_controller.update(target->rollTarget, state->roll, time) * 30 * 3.14 / 180;
  ROS_INFO("Roll control: %f, Roll target: %f, Roll state: %f, time: %f", roll_control, target->rollTarget, state->roll, time);
  control_out.rightFoil = roll_control + flap_control;
  control_out.leftFoil = -roll_control + flap_control;
  control_out.elevatorFoil = elevator_control;

  return control_out;
}