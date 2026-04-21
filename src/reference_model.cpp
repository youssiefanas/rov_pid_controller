#include "rov_pid_controller/reference_model.hpp"
 
#include <cmath>
 
namespace rov_pid_controller {
 
ReferenceModel::ReferenceModel(const Config & cfg) { set_config(cfg); }
 
void ReferenceModel::set_config(const Config & cfg) { cfg_ = cfg; }
 
void ReferenceModel::set_setpoint(double raw_setpoint) {
  if (cfg_.angular) {
    // Rebase the filtered reference onto the short path toward the new
    // target: x_ref' = x_ref + wrap(new - x_ref). Without this, a yaw step
    // from +170° to −170° would drive the internal state 340° the long way.
    x_ref_ += std::remainder(raw_setpoint - x_ref_, 360.0);
    raw_sp_ = x_ref_;
  } else {
    raw_sp_ = raw_setpoint;
  }
}
 
void ReferenceModel::snap_to(double measured_pose) {
  raw_sp_ = measured_pose;
  x_ref_  = measured_pose;
  x_d_    = measured_pose;
  v_d_    = 0.0;
  a_d_    = 0.0;
}
 
void ReferenceModel::reset() { snap_to(0.0); }
 
double ReferenceModel::update(double dt) {
  if (dt <= 0.0) return x_d_;
 
  // First-order setpoint filter (Fossen 7.88):  ẋ_ref = (raw_sp - x_ref)/τ.
  // Use the exact discrete solution x_ref ← x_ref + (1 - e^(-dt/τ))·(raw_sp - x_ref).
  // This is unconditionally stable regardless of dt/τ — explicit Euler would
  // overshoot badly when τ is small relative to the control period. τ <= 0
  // disables the filter (x_ref tracks raw_sp instantly).
  if (cfg_.tau > 0.0) {
    const double alpha = 1.0 - std::exp(-dt / cfg_.tau);
    x_ref_ += alpha * (raw_sp_ - x_ref_);
  } else {
    x_ref_ = raw_sp_;
  }
 
  // Second-order desired trajectory (Fossen 7.87), rearranged as
  //   a_d = ω² (x_ref - x_d) - 2ζω v_d
  // and integrated with semi-implicit Euler — v_d updates first, then x_d
  // uses the new v_d. This is stable for reasonable ω·dt (< 2).
  a_d_ = cfg_.omega * cfg_.omega * (x_ref_ - x_d_)
       - 2.0 * cfg_.zeta * cfg_.omega * v_d_;
  v_d_ += dt * a_d_;
  x_d_ += dt * v_d_;
 
  return x_d_;
}
 
}  // namespace rov_pid_controller