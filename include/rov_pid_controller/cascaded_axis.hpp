#pragma once

#include "rov_pid_controller/pid.hpp"

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
};

class CascadedAxis {
public:
  CascadedAxis() = default;
  explicit CascadedAxis(const CascadedAxisConfig & cfg);

  void set_config(const CascadedAxisConfig & cfg);
  const CascadedAxisConfig & config() const { return cfg_; }

  void set_mode(AxisMode mode);
  AxisMode mode() const { return mode_; }

  void set_pose_setpoint(double sp) { pose_sp_ = sp; }
  double pose_setpoint() const { return pose_sp_; }

  void reset();

  // One PID step. Caller must ensure mode != OFF.
  //   measured_pose, measured_vel : current state for this axis
  //   velocity_sp                 : external velocity command (used in INNER_ONLY)
  //   dt                          : time step in seconds
  double update(double measured_pose, double measured_vel,
                double velocity_sp, double dt);

private:
  CascadedAxisConfig cfg_{};
  Pid outer_{};
  Pid inner_{};
  AxisMode mode_ = AxisMode::OFF;
  double pose_sp_ = 0.0;
};

}  // namespace rov_pid_controller
