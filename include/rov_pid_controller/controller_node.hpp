#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rov_pid_controller/cascaded_axis.hpp"
#include "rov_pid_controller/msg/controller_status.hpp"
#include "rov_pid_controller/srv/capture_setpoint.hpp"
#include "rov_pid_controller/srv/set_all_modes.hpp"
#include "rov_pid_controller/srv/set_axis_mode.hpp"

namespace rov_pid_controller {

class ControllerNode : public rclcpp::Node {
public:
  static constexpr size_t N_AXES = 6;
  enum Axis { SURGE = 0, SWAY, HEAVE, ROLL, PITCH, YAW };

  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // -- setup --
  void declare_parameters();
  void reload_config();
  CascadedAxisConfig build_axis_config(size_t i) const;

  // -- callbacks --
  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void ff_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void control_tick();

  rcl_interfaces::msg::SetParametersResult
    on_parameters_set(const std::vector<rclcpp::Parameter> & params);

  void handle_set_axis_mode(
    const std::shared_ptr<srv::SetAxisMode::Request> request,
    std::shared_ptr<srv::SetAxisMode::Response> response);

  void handle_capture_setpoint(
    const std::shared_ptr<srv::CaptureSetpoint::Request> request,
    std::shared_ptr<srv::CaptureSetpoint::Response> response);

  void handle_set_all_modes(
    const std::shared_ptr<srv::SetAllModes::Request> request,
    std::shared_ptr<srv::SetAllModes::Response> response);



  void handle_clear_safety(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publish_status();

  // Force all axes to OFF, reset integrators, and block output until a new
  // cmd_vel_in message arrives (so stale PID effort can't leak through).
  void trip_safety(const std::string & reason);

  // -- control state --
  std::array<CascadedAxis, N_AXES> axes_{};
  std::array<double, N_AXES> max_velocity_{};        // for joystick scale in OFF mode
  std::array<double, N_AXES> max_effort_norm_{};     // for wrench->Twist normalization

  std::array<double, N_AXES> pose_{};                // x, y, z (m); roll, pitch, yaw (degrees [-180,180])
  std::array<double, N_AXES> vel_{};                 // body-frame linear + angular
  std::array<double, N_AXES> ff_vel_{};              // cmd_vel_in (physical units)
  std::array<double, N_AXES> last_effort_{};         // last-published force/torque

  bool have_odom_ = false;
  bool params_dirty_ = false;
  rclcpp::Time last_tick_;

  bool publish_wrench_ = true;   // whether to publish wrench messages for sim; if false, only publish cmd_vel
  bool publish_cmd_vel_ = false;

  // -- safety watchdog --
  double odom_timeout_ = 0.5;    // seconds; 0 or negative disables the watchdog
  rclcpp::Time last_odom_time_;  // updated in odom_callback
  rclcpp::Time last_ff_time_;    // updated in ff_callback
  rclcpp::Time safety_trip_time_;
  bool safety_tripped_ = false;

  // -- ROS I/O --
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ff_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<msg::ControllerStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<srv::SetAxisMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<srv::SetAllModes>::SharedPtr set_all_modes_srv_;
  rclcpp::Service<srv::CaptureSetpoint>::SharedPtr capture_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_safety_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace rov_pid_controller
