#include <algorithm>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 18.0;
const double RegulatedMotor::kI = 0.0;
const double RegulatedMotor::kD = 0.0;
const double RegulatedMotor::kFF = 0.4;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;
const double RegulatedMotor::VEL_SMOOTHING = 0.6;

RegulatedMotor::RegulatedMotor(unsigned long period_ms) : abstract_force(0), integral(0), last_angle_rad(0),
                                                          last_error(0), setpoint_rps(0), smoothed_velocity_rps(0),
                                                          period_ms(period_ms) {}

double RegulatedMotor::run_pid(unsigned long time_ms, double angle_rad) {
  static unsigned long last_run_time_ms;
  static bool initialized = false;

  if (!initialized) {
    initialized = true;
    last_run_time_ms = time_ms;
    last_angle_rad = angle_rad;
    return 0;
  }

  auto dt = time_ms - last_run_time_ms;
  if (dt >= period_ms) {

    velocity_rps = (angle_rad - last_angle_rad) / (dt / 1000.0);
    smoothed_velocity_rps = (VEL_SMOOTHING * smoothed_velocity_rps) + ((1 - VEL_SMOOTHING) * velocity_rps);
    error = setpoint_rps - velocity_rps;
    derivative = (error - last_error) / dt;
    integral += error * dt * kI;
    integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

    abstract_force = setpoint_rps * kFF + error * kP + integral * kI + derivative * kD;

    abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

    last_run_time_ms = time_ms;
    last_angle_rad = angle_rad;
  }

  return 15;
//  return abstract_force;
}

void RegulatedMotor::set_setpoint(double setpoint_rps) {
  this->setpoint_rps = setpoint_rps;
}
