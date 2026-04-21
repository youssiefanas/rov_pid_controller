#pragma once

#include "rov_pid_controller/pid.hpp"
#include "rov_pid_controller/reference_model.hpp"

namespace rov_pid_controller {

enum class AxisMode {
  OFF = 0,         // PID disabled; node handles passthrough externally
  INNER_ONLY = 1,  // inner velocity loop tracks an external velocity setpoint
  FULL = 2,        // outer pose loop feeds velocity setpoint to inner loop
};

struct CascadedAxisConfig {
  Pid::Config outer;        // pose  -> velocity setpoint
  Pid::Config inner;        // vel   -> force / torque
  double max_velocity = 0.0;  // clamp on outer loop output; <=0 disables
  double max_effort   = 0.0;  // clamp on inner loop output; <=0 disables
  bool angular = false;       // if true, pose error wraps to [-pi, pi]

  // Optional Fossen (7.87–7.91) reference model that smooths the pose
  // setpoint before it reaches the outer PID. When use_reference_model is
  // false, the raw pose_sp is fed straight through (legacy behavior).
  ReferenceModel::Config ref{};
  bool use_reference_model = false;
};

class CascadedAxis {
public:
  CascadedAxis() = default;
  explicit CascadedAxis(const CascadedAxisConfig & cfg);

  void set_config(const CascadedAxisConfig & cfg);
  const CascadedAxisConfig & config() const { return cfg_; }

  void set_mode(AxisMode mode);
  AxisMode mode() const { return mode_; }

  void set_pose_setpoint(double sp);
  double pose_setpoint() const { return pose_sp_; }

    // Snap the reference model onto a measured pose (zero velocity/accel) so
  // enabling FULL mode doesn't schedule a slew from stale state. Call this
  // before transitioning OFF -> FULL when the reference model is active.
  void arm(double measured_pose);
 
  // Desired smoothed pose from the reference model; equals pose_sp_ when the
  // reference model is disabled.
  double desired_pose() const;
 
  void reset();

  // One PID step. Caller must ensure mode != OFF.
  //   measured_pose, measured_vel : current state for this axis
  //   velocity_sp                 : external velocity command (used in INNER_ONLY)
  //   dt                          : time step in seconds
  double update(double measured_pose, double measured_vel,
                double velocity_sp, double dt);

  // Variant that takes a pre-computed pose error (e.g. the linear position
  // error already expressed in the body frame by the caller). Only valid for
  // linear axes — the outer PID on angular axes wraps the error itself, so
  // those must still use update(). Returns 0 for OFF; otherwise behaves like
  // update() except the outer loop consumes `pose_error` directly.
  double update_with_pose_error(double pose_error, double measured_vel,
                                double velocity_sp, double dt);

private:
  CascadedAxisConfig cfg_{};
  Pid outer_{};
  Pid inner_{};
  ReferenceModel ref_model_{};
  AxisMode mode_ = AxisMode::OFF;
  double pose_sp_ = 0.0;
};

}  // namespace rov_pid_controller
