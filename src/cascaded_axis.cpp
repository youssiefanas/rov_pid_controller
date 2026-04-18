#include "rov_pid_controller/cascaded_axis.hpp"

#include <cmath>

namespace rov_pid_controller {

namespace {
double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

CascadedAxis::CascadedAxis(const CascadedAxisConfig & cfg) { set_config(cfg); }

void CascadedAxis::set_config(const CascadedAxisConfig & cfg) {
  cfg_ = cfg;

  Pid::Config outer_cfg = cfg.outer;
  if (cfg.max_velocity > 0.0) {
    outer_cfg.out_min = -cfg.max_velocity;
    outer_cfg.out_max =  cfg.max_velocity;
  }
  outer_.set_config(outer_cfg);

  Pid::Config inner_cfg = cfg.inner;
  if (cfg.max_effort > 0.0) {
    inner_cfg.out_min = -cfg.max_effort;
    inner_cfg.out_max =  cfg.max_effort;
  }
  inner_.set_config(inner_cfg);
}

void CascadedAxis::set_mode(AxisMode mode) {
  if (mode == mode_) return;
  // Reset integrators to avoid a bump when transitioning between modes.
  outer_.reset();
  inner_.reset();
  mode_ = mode;
}

void CascadedAxis::reset() {
  outer_.reset();
  inner_.reset();
}

double CascadedAxis::update(double measured_pose, double measured_vel,
                            double velocity_sp, double dt) {
  if (mode_ == AxisMode::OFF) return 0.0;

  double vel_sp = velocity_sp;
  if (mode_ == AxisMode::FULL) {
    // For angular axes, feed the wrapped error directly so the PID sees a
    // continuous signal around ±π.
    double sp_input = pose_sp_;
    double meas_input = measured_pose;
    if (cfg_.angular) {
      sp_input = 0.0;
      meas_input = -wrap_pi(pose_sp_ - measured_pose);
    }
    vel_sp = outer_.update(sp_input, meas_input, dt);
  }

  return inner_.update(vel_sp, measured_vel, dt);
}

}  // namespace rov_pid_controller
