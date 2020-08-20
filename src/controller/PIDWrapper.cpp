#include <fcs_ros_deb/controller/PIDWrapper.hpp>
#include <fcs_ros_deb/ControllerConfig.h>

PIDWrapper::PIDWrapper()
{
  // this->n = nh;
  altitude_controller = PIDFF();
  altitude_rate_controller = PIDFF();
  roll_controller = PIDFF();
  pitch_controller = PIDFF();
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
      altitude_controller.setKD(gains.kd);
      break;
    case altitude_rate:
      ROS_INFO("Setting altitude rate PID");
      altitude_rate_controller.setKP(gains.kp);
      altitude_rate_controller.setKI(gains.ki);
      altitude_rate_controller.setKD(gains.kd);
      break;
    case roll:
      ROS_INFO("Setting roll PID");
      roll_controller.setKP(gains.kp);
      roll_controller.setKI(gains.ki);
      roll_controller.setKD(gains.kd);
      break;
    case pitch:
      ROS_INFO("Setting pitch PID");
      pitch_controller.setKP(gains.kp);
      pitch_controller.setKI(gains.ki);
      pitch_controller.setKD(gains.kd);
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

fcs_ros_deb::FoilboatControl PIDWrapper::control(
  fcs_ros_deb::FoilboatTarget::ConstPtr target,
  fcs_ros_deb::FoilboatState::ConstPtr state,
  double time,
  int mode
)
{
  fcs_ros_deb::FoilboatControl control_out;
  ROS_INFO("========== Control loop ==========");
  double flap_control = 0;
    if (target->altitudeTarget > 0) {
      double altitude_rate_target = altitude_controller.update(target->altitudeTarget, state->altitude, time);
      ROS_INFO("Altitude rate target: %f, Altitude target: %f, Altitude state: %f", altitude_rate_target,
               target->altitudeTarget, state->altitude);
      control_out.altitudeRateTarget = altitude_rate_target;
      flap_control = -altitude_rate_controller.update(altitude_rate_target, state->altitudeRate, time);
      ROS_INFO("Target flaps: %f, Altitude rate target: %f, Altitude rate state: %f", flap_control,
               altitude_rate_target, state->altitudeRate);
    }

    // Pitch angle control:
    // Tries to keep boat flat
    double target_pitch = target->pitchTarget * 6.28 / 360;
    double pitch_control = -pitch_controller.update(target_pitch, state->pitch, time);
    ROS_INFO("Pitch control: %f, Pitch target: %f, Pitch state: %f, time: %f", pitch_control, target_pitch,
             state->pitch, time);

    double roll_control = roll_controller.update(target->rollTarget * 6.28 / 360, state->roll, time);
    ROS_INFO("Roll control: %f, Roll target: %f, Roll state: %f", roll_control, target->rollTarget, state->roll);

    // Combine PID outputs:

    switch(mode)
    {
      // Positive pitch comp for physical, negative for sim
      case fcs_ros_deb::Controller_flaps_roll:
        control_out.rightFlap = roll_control + target->flapTrim - state->pitch;
        control_out.leftFlap = -roll_control + target->flapTrim - state->pitch;
        control_out.onlyElevator = 0;
        control_out.leftElevator = 0;
        control_out.rightElevator = 0;
        break;

      case fcs_ros_deb::Controller_flaps_both:
        control_out.rightFlap = roll_control + flap_control - state->pitch;
        control_out.leftFlap = -roll_control + flap_control - state->pitch;
        control_out.onlyElevator = 0;
        control_out.leftElevator = 0;
        control_out.rightElevator = 0;
        break;

      case fcs_ros_deb::Controller_both_roll:
        control_out.rightFlap = roll_control + target->flapTrim - state->pitch;
        control_out.leftFlap = -roll_control + target->flapTrim - state->pitch;
        control_out.onlyElevator = pitch_control + 0.2 * (target->flapTrim - state->pitch);
        control_out.leftElevator = pitch_control + 0.2 * (target->flapTrim + roll_control + state->pitch);
        control_out.rightElevator = pitch_control + 0.2 * (target->flapTrim - roll_control + state->pitch);
        break;

      case fcs_ros_deb::Controller_both_both:
        control_out.rightFlap = roll_control + flap_control - state->pitch;
        control_out.leftFlap = -roll_control + flap_control - state->pitch;
        control_out.onlyElevator = pitch_control + 0.2 * (flap_control - state->pitch);
        control_out.leftElevator = pitch_control + 0.2 * (flap_control + roll_control + state->pitch);
        control_out.rightElevator = pitch_control + 0.2 * (flap_control - roll_control + state->pitch);
        break;

      default:
        control_out.rightFlap = 0;
        control_out.leftFlap = 0;
        control_out.onlyElevator = 0;
        control_out.leftElevator = 0;
        control_out.rightElevator = 0;
        break;
    }

  return control_out;
}