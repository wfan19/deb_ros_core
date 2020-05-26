#include <foilboat_controller/PIDWrapper.hpp>

PIDWrapper::PIDWrapper()
{
  // this->n = nh;
  altitude_controller = PIDFF();
  pitch_controller = PIDFF();
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
    case pitch:
      ROS_INFO("Setting pitch PID");
      pitch_controller.setKP(gains.kp);
      pitch_controller.setKI(gains.ki);
      pitch_controller.setKI(gains.ki);
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
    case pitch:
      pitch_controller.init(config);
      break;
    case roll:
      roll_controller.init(config);
      break;
  }
}

void PIDWrapper::resetIntegrators()
{
  altitude_controller.resetIntegrator();
  pitch_controller.resetIntegrator();
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
  float pitch_control = -pitch_controller.update(target->pitchTarget, state->pitch, time) * 30 * 3.14 / 180;
  ROS_INFO("Pitch control: %f, Pitch target: %f, Pitch state: %f, time: %f", pitch_control, target->pitchTarget, state->pitch, time);

  float roll_control = roll_controller.update(target->rollTarget, state->roll, time) * 30 * 3.14 / 180;
  ROS_INFO("Roll control: %f, Roll target: %f, Roll state: %f, time: %f", roll_control, target->rollTarget, state->roll, time);

  control_out.rightFoil = roll_control;
  control_out.leftFoil = -roll_control;
  control_out.elevatorFoil = pitch_control;

  return control_out;
}