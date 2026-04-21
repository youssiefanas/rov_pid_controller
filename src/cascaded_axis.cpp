#include "rov_pid_controller/cascaded_axis.hpp"

#include <cmath>

namespace rov_pid_controller {

CascadedAxis::CascadedAxis(const CascadedAxisConfig & cfg) { set_config(cfg); }

void CascadedAxis::set_config(const CascadedAxisConfig & cfg) {
  cfg_ = cfg;

  Pid::Config outer_cfg = cfg.outer;
  outer_cfg.angular = cfg.angular;
  if (cfg.angular) {
    // Angular axes run single-loop: outer PID produces torque directly (mimosa
    // odometry has no angular velocity, so a cascaded inner velocity loop
    // would have no feedback). Clamp the outer output to max_effort.
    if (cfg.max_effort > 0.0) {
      outer_cfg.out_min = -cfg.max_effort;
      outer_cfg.out_max =  cfg.max_effort;
    }
  } else if (cfg.max_velocity > 0.0) {
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
 
  ReferenceModel::Config ref_cfg = cfg.ref;
  ref_cfg.angular = cfg.angular;
  ref_model_.set_config(ref_cfg);
}

void CascadedAxis::set_mode(AxisMode mode) {
  if (mode == mode_) return;
  // Reset integrators to avoid a bump when transitioning between modes.
  outer_.reset();
  inner_.reset();
  mode_ = mode;
}

void CascadedAxis::set_pose_setpoint(double sp) {
  pose_sp_ = sp;
  ref_model_.set_setpoint(sp);
}
 
void CascadedAxis::arm(double measured_pose) {
  // Snap the reference model onto the current pose so the trajectory starts
  // where the vehicle actually is. Preserve pose_sp_ so a subsequent
  // set_pose_setpoint still resolves to the operator's intended target.
  ref_model_.snap_to(measured_pose);
  ref_model_.set_setpoint(pose_sp_);
}
 
double CascadedAxis::desired_pose() const {
  return cfg_.use_reference_model ? ref_model_.x_d() : pose_sp_;
}

void CascadedAxis::reset() {
  outer_.reset();
  inner_.reset();
  ref_model_.reset();
  ref_model_.set_setpoint(pose_sp_);
}

double CascadedAxis::update(double measured_pose, double measured_vel,
                            double velocity_sp, double dt) {
  if (mode_ == AxisMode::OFF) return 0.0;

  double effective_sp = pose_sp_;
  if (cfg_.use_reference_model) {
    effective_sp = ref_model_.update(dt);
  }

  if (cfg_.angular) {
    // Single-loop: outer PID maps pose error directly to torque.
    // The PID class handles angular error wrapping and D-term wrapping internally,
    // so we can safely pass raw [-180, 180] degree measurements here.
    // INNER_ONLY has no inner loop for angular axes and falls through to a no-op.
    if (mode_ == AxisMode::FULL) {
      return outer_.update(effective_sp, measured_pose, dt);
    }
    return 0.0;
  }

  double vel_sp = velocity_sp;
  if (mode_ == AxisMode::FULL) {
    vel_sp = outer_.update(effective_sp, measured_pose, dt);
  }
  return inner_.update(vel_sp, measured_vel, dt);
}

double CascadedAxis::update_with_pose_error(double pose_error,
                                            double measured_vel,
                                            double velocity_sp, double dt) {
  if (mode_ == AxisMode::OFF || cfg_.angular) return 0.0;

  // Advance the reference model so x_d progresses in lockstep with update()
  // even when the caller provides a pre-computed error. The value itself is
  // unused — the caller already baked it into pose_error.
  if (cfg_.use_reference_model) {
    ref_model_.update(dt);
  }

  double vel_sp = velocity_sp;
  if (mode_ == AxisMode::FULL) {
    // Pass setpoint=0, measurement=-pose_error so the PID computes
    // error = 0 - (-pose_error) = pose_error. Derivative-on-measurement then
    // yields -kd * d(-pose_error)/dt = kd * d(pose_error)/dt, matching the
    // sign of d/dt of the body-frame error.
    vel_sp = outer_.update(0.0, -pose_error, dt);
  }
  return inner_.update(vel_sp, measured_vel, dt);
}

}  // namespace rov_pid_controller
