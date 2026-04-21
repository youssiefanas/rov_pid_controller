#pragma once
 
namespace rov_pid_controller {
 
// Scalar reference model per Fossen (7.87–7.91): a first-order setpoint
// filter feeding a second-order mass-spring-damper, producing a smooth
// desired trajectory (x_d, v_d, a_d) from a raw setpoint that may change
// discontinuously. One instance per DOF — because Ω, Γ, and A_f are
// diagonal, the multi-DOF system decouples into independent scalar filters.
class ReferenceModel {
public:
  struct Config {
    double omega = 1.0;      // natural frequency ω (rad/s); sets bandwidth
    double zeta  = 1.0;      // damping ζ; 1.0 = critical (no overshoot)
    double tau   = 0.5;      // first-order setpoint filter time constant (s)
    bool   angular = false;  // wrap raw-setpoint jumps to the short path (±180°)
  };
 
  ReferenceModel() = default;
  explicit ReferenceModel(const Config & cfg);
 
  void set_config(const Config & cfg);
  const Config & config() const { return cfg_; }
 
  // Update the raw setpoint. For angular axes, the internal filtered
  // reference jumps along the shortest path so x_ref stays within 180° of
  // the new target.
  void set_setpoint(double raw_setpoint);
  double setpoint() const { return raw_sp_; }
 
  // Snap the model to a measured pose with zero velocity/acceleration. Use
  // this when arming a control mode so the model doesn't schedule a slew
  // from whatever state it happened to hold.
  void snap_to(double measured_pose);
 
  // Clear all state (equivalent to snap_to(0) with raw_sp=0).
  void reset();
 
  // Advance one time step. Returns x_d (the smoothed setpoint to feed into
  // the outer PID).
  double update(double dt);
 
  double x_d() const { return x_d_; }
  double v_d() const { return v_d_; }
  double a_d() const { return a_d_; }
 
private:
  Config cfg_{};
  double raw_sp_ = 0.0;  // most recent external setpoint
  double x_ref_  = 0.0;  // filtered setpoint (eq 7.88)
  double x_d_    = 0.0;  // desired pose
  double v_d_    = 0.0;  // desired velocity
  double a_d_    = 0.0;  // desired acceleration
};
 
}  // namespace rov_pid_controller