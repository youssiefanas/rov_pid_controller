#include "rov_pid_controller/pid.hpp"

#include <algorithm>
#include <cmath>

namespace rov_pid_controller {

Pid::Pid(const Config & cfg) : cfg_(cfg) {}

void Pid::set_config(const Config & cfg) { cfg_ = cfg; }

void Pid::reset() {
  i_term_ = 0.0;
  prev_error_ = 0.0;
  prev_measurement_ = 0.0;
  prev_output_ = 0.0;
  first_ = true;
}

double Pid::update(double setpoint, double measurement, double dt) {
  if (dt <= 0.0) return prev_output_;

  double error = setpoint - measurement;
  if (cfg_.angular) {
    // std::remainder gives a symmetric [-180, 180] result regardless of sign,
    // unlike std::fmod which keeps the dividend's sign and breaks for error < -180.
    error = std::remainder(error, 360.0);
  }
  const double p = cfg_.kp * error;

  // Angular axes have no velocity feedback available (mimosa odom), so the
  // derivative term is skipped there. For linear axes, prefer
  // derivative-on-measurement to avoid the "derivative kick" on setpoint steps.
  double d = 0.0;
  if (!cfg_.angular && !first_) {
    d = cfg_.derivative_on_measurement
            ? -cfg_.kd * (measurement - prev_measurement_) / dt
            :  cfg_.kd * (error - prev_error_) / dt;
  }

  // Tentative output using current (not-yet-updated) integrator state.
  double u_unsat = p + i_term_ + d;
  double u = std::clamp(u_unsat, cfg_.out_min, cfg_.out_max);

  // Conditional integration: skip integrating if saturated in the direction the
  // error would push further. Prevents wind-up without back-calculation gains.
  const bool push_high = (u_unsat > cfg_.out_max) && (error > 0.0);
  const bool push_low  = (u_unsat < cfg_.out_min) && (error < 0.0);
  if (!push_high && !push_low) {
    i_term_ += cfg_.ki * error * dt;
    if (cfg_.i_max > 0.0) {
      i_term_ = std::clamp(i_term_, -cfg_.i_max, cfg_.i_max);
    }
    u_unsat = p + i_term_ + d;
    u = std::clamp(u_unsat, cfg_.out_min, cfg_.out_max);
  }

  // Optional output rate limit.
  // could be replaced with set points
  if (cfg_.rate_limit > 0.0 && !first_) {
    const double max_delta = cfg_.rate_limit * dt;
    u = std::clamp(u, prev_output_ - max_delta, prev_output_ + max_delta);
  }

  prev_error_ = error;
  prev_measurement_ = measurement;
  prev_output_ = u;
  first_ = false;
  return u;
}

}  // namespace rov_pid_controller
