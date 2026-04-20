#pragma once

#include <limits>

namespace rov_pid_controller {

class Pid {
public:
  struct Config {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double i_max = 0.0;   // symmetric integrator clamp; <=0 disables
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max =  std::numeric_limits<double>::infinity();
    double rate_limit = 0.0;  // max |du/dt|; <=0 disables
    bool derivative_on_measurement = true;
    bool angular = false;        // if true, wrap error to [-180, 180] (degrees)
  };

  Pid() = default;
  explicit Pid(const Config & cfg);

  void set_config(const Config & cfg);
  const Config & config() const { return cfg_; }
  void reset();

  // Compute one control step. Returns saturated output.
  double update(double setpoint, double measurement, double dt);

private:
  Config cfg_{};
  double i_term_ = 0.0;
  double prev_error_ = 0.0;
  double prev_measurement_ = 0.0;
  double prev_output_ = 0.0;
  bool first_ = true;
};

}  // namespace rov_pid_controller
