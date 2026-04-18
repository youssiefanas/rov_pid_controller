#include "rov_pid_controller/controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace {
rcl_interfaces::msg::ParameterDescriptor
make_float_slider(double lo, double hi, double step, const std::string & desc) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = desc;
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = step;
  d.floating_point_range.push_back(r);
  return d;
}
}  // namespace

namespace rov_pid_controller {

namespace {
const char * AXIS_NAMES[6] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
constexpr bool AXIS_ANGULAR[6] = {false, false, false, true, true, true};

double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
    : Node("rov_pid_controller", options) {
  declare_parameters();
  reload_config();

  const double rate = this->get_parameter("control_rate").as_double();
  publish_wrench_   = this->get_parameter("publish_wrench").as_bool();
  publish_cmd_vel_  = this->get_parameter("publish_cmd_vel").as_bool();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odometry_topic").as_string(), qos,
      std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1));

  ff_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", qos,
      std::bind(&ControllerNode::ff_callback, this, std::placeholders::_1));

  if (publish_wrench_) {
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("wrench", 10);
  }
  if (publish_cmd_vel_) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  status_pub_ = this->create_publisher<msg::ControllerStatus>("status", 10);

  set_mode_srv_ = this->create_service<srv::SetAxisMode>(
      "set_axis_mode",
      std::bind(&ControllerNode::handle_set_axis_mode, this,
                std::placeholders::_1, std::placeholders::_2));

  set_all_modes_srv_ = this->create_service<srv::SetAllModes>(
      "set_all_modes",
      std::bind(&ControllerNode::handle_set_all_modes, this,
                std::placeholders::_1, std::placeholders::_2));

  capture_srv_ = this->create_service<srv::CaptureSetpoint>(
      "capture_setpoint",
      std::bind(&ControllerNode::handle_capture_setpoint, this,
                std::placeholders::_1, std::placeholders::_2));

  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ControllerNode::on_parameters_set, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / rate);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControllerNode::control_tick, this));

  const double status_rate = 10.0;
  status_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / status_rate)),
      std::bind(&ControllerNode::publish_status, this));

  last_tick_ = this->now();

  RCLCPP_INFO(this->get_logger(),
              "rov_pid_controller ready @ %.1f Hz  (wrench=%d twist=%d)",
              rate, publish_wrench_, publish_cmd_vel_);
}

// ---------------------------------------------------------------------------
// Parameter machinery
// ---------------------------------------------------------------------------
void ControllerNode::declare_parameters() {
  this->declare_parameter("control_rate", 50.0);
  this->declare_parameter("odometry_topic", "imu/manager/odometry");
  this->declare_parameter("publish_wrench", true);
  this->declare_parameter("publish_cmd_vel", false);

  // Slider ranges — chosen to cover typical BlueROV tuning regimes.
  // Outer loop produces a velocity setpoint; inner loop produces force/torque,
  // so inner gains are naturally an order of magnitude larger.
  const auto outer_kp = make_float_slider(0.0, 20.0, 0.05, "outer (pose) Kp");
  const auto outer_ki = make_float_slider(0.0, 10.0, 0.01, "outer (pose) Ki");
  const auto outer_kd = make_float_slider(0.0, 10.0, 0.01, "outer (pose) Kd");
  const auto outer_im = make_float_slider(0.0, 50.0, 0.1,  "outer integrator clamp (<=0 disables)");
  const auto inner_kp = make_float_slider(0.0, 200.0, 0.5, "inner (velocity) Kp");
  const auto inner_ki = make_float_slider(0.0, 100.0, 0.1, "inner (velocity) Ki");
  const auto inner_kd = make_float_slider(0.0, 100.0, 0.1, "inner (velocity) Kd");
  const auto inner_im = make_float_slider(0.0, 200.0, 0.5, "inner integrator clamp (<=0 disables)");
  const auto max_vel  = make_float_slider(0.0, 5.0,  0.05, "outer loop velocity clamp");
  const auto max_eff  = make_float_slider(0.0, 400.0, 1.0, "inner loop effort clamp (N or Nm)");
  const auto max_effn = make_float_slider(0.0, 400.0, 1.0, "effort normalization for Twist output");

  for (size_t i = 0; i < N_AXES; ++i) {
    const std::string a = AXIS_NAMES[i];
    this->declare_parameter("gains." + a + ".outer.kp",    1.0,  outer_kp);
    this->declare_parameter("gains." + a + ".outer.ki",    0.0,  outer_ki);
    this->declare_parameter("gains." + a + ".outer.kd",    0.0,  outer_kd);
    this->declare_parameter("gains." + a + ".outer.i_max", 0.0,  outer_im);
    this->declare_parameter("gains." + a + ".inner.kp",    10.0, inner_kp);
    this->declare_parameter("gains." + a + ".inner.ki",    0.0,  inner_ki);
    this->declare_parameter("gains." + a + ".inner.kd",    0.0,  inner_kd);
    this->declare_parameter("gains." + a + ".inner.i_max", 0.0,  inner_im);
    this->declare_parameter("limits." + a + ".max_velocity",    0.5,  max_vel);
    this->declare_parameter("limits." + a + ".max_effort",      40.0, max_eff);
    this->declare_parameter("limits." + a + ".max_effort_norm", 40.0, max_effn);
    this->declare_parameter("mode." + a + ".default", 0);
  }
}

CascadedAxisConfig ControllerNode::build_axis_config(size_t i) const {
  const std::string a = AXIS_NAMES[i];
  CascadedAxisConfig c;
  c.outer.kp    = this->get_parameter("gains." + a + ".outer.kp").as_double();
  c.outer.ki    = this->get_parameter("gains." + a + ".outer.ki").as_double();
  c.outer.kd    = this->get_parameter("gains." + a + ".outer.kd").as_double();
  c.outer.i_max = this->get_parameter("gains." + a + ".outer.i_max").as_double();
  c.inner.kp    = this->get_parameter("gains." + a + ".inner.kp").as_double();
  c.inner.ki    = this->get_parameter("gains." + a + ".inner.ki").as_double();
  c.inner.kd    = this->get_parameter("gains." + a + ".inner.kd").as_double();
  c.inner.i_max = this->get_parameter("gains." + a + ".inner.i_max").as_double();
  c.max_velocity = this->get_parameter("limits." + a + ".max_velocity").as_double();
  c.max_effort   = this->get_parameter("limits." + a + ".max_effort").as_double();
  c.angular = AXIS_ANGULAR[i];
  return c;
}

void ControllerNode::reload_config() {
  for (size_t i = 0; i < N_AXES; ++i) {
    const std::string a = AXIS_NAMES[i];
    const auto cfg = build_axis_config(i);

    // Preserve current mode+setpoint across gain updates. Only adopt the
    // declared default when the axis is still at its initial state.
    AxisMode current = axes_[i].mode();
    double sp = axes_[i].pose_setpoint();

    axes_[i].set_config(cfg);

    if (current == AxisMode::OFF && sp == 0.0) {
      int m = this->get_parameter("mode." + a + ".default").as_int();
      axes_[i].set_mode(static_cast<AxisMode>(std::clamp(m, 0, 2)));
    } else {
      axes_[i].set_mode(current);
      axes_[i].set_pose_setpoint(sp);
    }

    max_velocity_[i]    = cfg.max_velocity;
    max_effort_norm_[i] = this->get_parameter("limits." + a + ".max_effort_norm").as_double();
  }
}

rcl_interfaces::msg::SetParametersResult
ControllerNode::on_parameters_set(const std::vector<rclcpp::Parameter> & /*params*/) {
  // rclcpp commits the new values after this callback returns success, so we
  // defer the rebuild to the next control tick when get_parameter() is current.
  params_dirty_ = true;
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// ---------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------
void ControllerNode::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  pose_[SURGE] = msg->pose.pose.position.x;
  pose_[SWAY]  = msg->pose.pose.position.y;
  pose_[HEAVE] = msg->pose.pose.position.z;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  pose_[ROLL]  = r;
  pose_[PITCH] = p;
  pose_[YAW]   = y;

  // nav_msgs/Odometry twist is specified in child_frame_id (body).
  vel_[SURGE] = msg->twist.twist.linear.x;
  vel_[SWAY]  = msg->twist.twist.linear.y;
  vel_[HEAVE] = msg->twist.twist.linear.z;
  vel_[ROLL]  = msg->twist.twist.angular.x;
  vel_[PITCH] = msg->twist.twist.angular.y;
  vel_[YAW]   = msg->twist.twist.angular.z;

  have_odom_ = true;
}

void ControllerNode::ff_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  ff_vel_[SURGE] = msg->linear.x;
  ff_vel_[SWAY]  = msg->linear.y;
  ff_vel_[HEAVE] = msg->linear.z;
  ff_vel_[ROLL]  = msg->angular.x;
  ff_vel_[PITCH] = msg->angular.y;
  ff_vel_[YAW]   = msg->angular.z;
}

// ---------------------------------------------------------------------------
// Control tick
// ---------------------------------------------------------------------------
void ControllerNode::control_tick() {
  if (params_dirty_) {
    reload_config();
    params_dirty_ = false;
  }

  const auto now = this->now();
  const double dt = (now - last_tick_).seconds();
  last_tick_ = now;

  if (!have_odom_ || dt <= 0.0) return;

  std::array<double, N_AXES> effort{};
  std::array<double, N_AXES> twist_out{};

  for (size_t i = 0; i < N_AXES; ++i) {
    if (axes_[i].mode() == AxisMode::OFF) {
      // Passthrough: cmd_vel_in is in physical units (m/s, rad/s). Normalize
      // by max_velocity to get a Twist the downstream mixer expects, and scale
      // to N/Nm for the wrench publisher so sim sees something equivalent.
      const double maxv = std::max(1e-9, max_velocity_[i]);
      const double norm = std::clamp(ff_vel_[i] / maxv, -1.0, 1.0);
      twist_out[i] = norm;
      effort[i]    = norm * max_effort_norm_[i];
    } else {
      effort[i] = axes_[i].update(pose_[i], vel_[i], ff_vel_[i], dt);
      twist_out[i] = (max_effort_norm_[i] > 0.0)
                         ? std::clamp(effort[i] / max_effort_norm_[i], -1.0, 1.0)
                         : 0.0;
    }
  }

  if (publish_wrench_ && wrench_pub_) {
    geometry_msgs::msg::Wrench w;
    w.force.x  = effort[SURGE];
    w.force.y  = effort[SWAY];
    w.force.z  = effort[HEAVE];
    w.torque.x = effort[ROLL];
    w.torque.y = effort[PITCH];
    w.torque.z = effort[YAW];
    wrench_pub_->publish(w);
  }

  if (publish_cmd_vel_ && cmd_vel_pub_) {
    geometry_msgs::msg::Twist t;
    t.linear.x  = twist_out[SURGE];
    t.linear.y  = twist_out[SWAY];
    t.linear.z  = twist_out[HEAVE];
    t.angular.x = twist_out[ROLL];
    t.angular.y = twist_out[PITCH];
    t.angular.z = twist_out[YAW];
    cmd_vel_pub_->publish(t);
  }

  last_effort_ = effort;
}

// ---------------------------------------------------------------------------
// Status publisher
// ---------------------------------------------------------------------------
void ControllerNode::publish_status() {
  if (!status_pub_) return;

  msg::ControllerStatus s;
  s.header.stamp = this->now();
  s.header.frame_id = "base_link";

  // Display-only conversion: angular axes (roll/pitch/yaw) are reported in
  // degrees so the status / Foxglove plots are easier to read. The control
  // loop internally still uses radians.
  constexpr double RAD2DEG = 180.0 / M_PI;

  for (size_t i = 0; i < N_AXES; ++i) {
    const auto mode = axes_[i].mode();
    const double scale = AXIS_ANGULAR[i] ? RAD2DEG : 1.0;

    s.modes[i]     = static_cast<uint8_t>(mode);
    s.setpoints[i] = axes_[i].pose_setpoint() * scale;
    s.pose[i]      = pose_[i] * scale;
    s.velocity[i]  = vel_[i] * scale;
    s.effort[i]    = last_effort_[i];  // force/torque — unit unchanged

    // Pose error is only meaningful in FULL mode — wrap angular axes to (-pi, pi]
    // (in radians) before converting to degrees for display.
    if (mode == AxisMode::FULL) {
      double e = axes_[i].pose_setpoint() - pose_[i];
      if (AXIS_ANGULAR[i]) e = wrap_pi(e);
      s.pose_error[i] = e * scale;
    } else {
      s.pose_error[i] = 0.0;
    }

    // Velocity error tracks the active velocity command: ff_vel drives INNER_ONLY
    // directly; in FULL it's the outer loop's output, which we don't surface —
    // fall back to ff_vel for a useful "commanded minus measured" readout.
    s.velocity_error[i] = (mode == AxisMode::OFF)
                              ? 0.0
                              : (ff_vel_[i] - vel_[i]) * scale;
  }
  status_pub_->publish(s);
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------
void ControllerNode::handle_set_axis_mode(
    const std::shared_ptr<srv::SetAxisMode::Request> request,
    std::shared_ptr<srv::SetAxisMode::Response> response) {
  if (request->axis >= N_AXES) {
    response->success = false;
    response->message = "axis index out of range (0..5)";
    return;
  }
  if (request->mode > 2) {
    response->success = false;
    response->message = "mode must be 0=OFF, 1=INNER_ONLY, 2=FULL";
    return;
  }

  const auto new_mode = static_cast<AxisMode>(request->mode);
  axes_[request->axis].set_mode(new_mode);

  if (new_mode == AxisMode::FULL) {
    if (request->capture_current) {
      if (!have_odom_) {
        response->success = false;
        response->message = "no odometry received yet; cannot capture setpoint";
        return;
      }
      axes_[request->axis].set_pose_setpoint(pose_[request->axis]);
    } else {
      axes_[request->axis].set_pose_setpoint(request->setpoint);
    }
  }

  response->success = true;
  response->message = "ok";
  RCLCPP_INFO(this->get_logger(),
              "axis %s -> mode %u, setpoint %.4f",
              AXIS_NAMES[request->axis], request->mode,
              axes_[request->axis].pose_setpoint());
}

void ControllerNode::handle_set_all_modes(
    const std::shared_ptr<srv::SetAllModes::Request> request,
    std::shared_ptr<srv::SetAllModes::Response> response) {
  // Validate
  for (size_t i = 0; i < N_AXES; ++i) {
    if (request->modes[i] > 2) {
      response->success = false;
      response->message = "mode out of range at axis " + std::to_string(i);
      return;
    }
  }

  // If any axis wants capture_current, require odometry.
  bool needs_capture = false;
  for (size_t i = 0; i < N_AXES; ++i) {
    if (request->capture_current[i] && request->modes[i] == 2) {
      needs_capture = true;
      break;
    }
  }
  if (needs_capture && !have_odom_) {
    response->success = false;
    response->message = "no odometry yet; cannot capture current pose";
    return;
  }

  for (size_t i = 0; i < N_AXES; ++i) {
    const auto mode = static_cast<AxisMode>(request->modes[i]);
    axes_[i].set_mode(mode);
    if (mode == AxisMode::FULL) {
      axes_[i].set_pose_setpoint(
          request->capture_current[i] ? pose_[i] : request->setpoints[i]);
    }
  }

  response->success = true;
  response->message = "ok";
  RCLCPP_INFO(this->get_logger(), "set_all_modes applied");
}

void ControllerNode::handle_capture_setpoint(
    const std::shared_ptr<srv::CaptureSetpoint::Request> request,
    std::shared_ptr<srv::CaptureSetpoint::Response> response) {
  if (!have_odom_) {
    response->success = false;
    response->message = "no odometry received yet";
    return;
  }

  std::vector<uint8_t> targets = request->axes.empty()
      ? std::vector<uint8_t>{0, 1, 2, 3, 4, 5}
      : std::vector<uint8_t>(request->axes.begin(), request->axes.end());

  for (uint8_t i : targets) {
    if (i < N_AXES) {
      axes_[i].set_pose_setpoint(pose_[i]);
    }
  }
  for (size_t i = 0; i < N_AXES; ++i) {
    response->captured_setpoints[i] = pose_[i];
  }

  response->success = true;
  response->message = "captured";
}

}  // namespace rov_pid_controller
