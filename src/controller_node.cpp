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
// step=0 means "continuous" — no modulo check, which would reject YAML-loaded
// values like 0.2 or 0.3 that aren't exact multiples of the slider step in
// IEEE 754. rqt/Foxglove still render a slider from the range alone.
rcl_interfaces::msg::ParameterDescriptor
make_float_slider(double lo, double hi, const std::string & desc) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = desc;
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 0.0;
  d.floating_point_range.push_back(r);
  return d;
}
}  // namespace

namespace rov_pid_controller {

namespace {
const char * AXIS_NAMES[6] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
constexpr bool AXIS_ANGULAR[6] = {false, false, false, true, true, true};

}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
    : Node("rov_pid_controller", options) {
  declare_parameters();
  reload_config();

  const double rate = this->get_parameter("control_rate").as_double();
  publish_wrench_   = this->get_parameter("publish_wrench").as_bool();
  publish_cmd_vel_  = this->get_parameter("publish_cmd_vel").as_bool();
  odom_timeout_     = this->get_parameter("odom_timeout").as_double();

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

  clear_safety_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_safety",
      std::bind(&ControllerNode::handle_clear_safety, this,
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
  last_odom_time_ = last_tick_;
  last_ff_time_ = last_tick_;
  safety_trip_time_ = last_tick_;

  RCLCPP_INFO(this->get_logger(),
              "rov_pid_controller ready @ %.1f Hz  (wrench=%d twist=%d odom_timeout=%.2fs)",
              rate, publish_wrench_, publish_cmd_vel_, odom_timeout_);
}

// ---------------------------------------------------------------------------
// Parameter machinery
// ---------------------------------------------------------------------------
void ControllerNode::declare_parameters() {
  this->declare_parameter("control_rate", 50.0);
  this->declare_parameter("odometry_topic", "imu/manager/odometry");
  this->declare_parameter("publish_wrench", true);
  this->declare_parameter("publish_cmd_vel", false);
  this->declare_parameter(
      "odom_timeout", 0.5,
      make_float_slider(0.0, 5.0,
          "seconds without odom before the safety watchdog trips; "
          "<=0 disables"));

  // Fossen (7.87–7.91) reference model — global enable, then per-axis shape.
  // Disabled by default so existing tuning reproduces bit-for-bit until
  // enabled explicitly.
  this->declare_parameter("reference_model.enabled", false);
  const auto ref_omega = make_float_slider(0.1, 5.0,  "reference-model natural frequency (rad/s)");
  const auto ref_zeta  = make_float_slider(0.5, 2.0,  "reference-model damping (1.0 = critical)");
  const auto ref_tau   = make_float_slider(0.01, 5.0, "first-order setpoint-filter time constant (s)");

  // Slider ranges — chosen to cover typical BlueROV tuning regimes. Linear axes
  // use a cascade (pose → velocity → force); angular axes run single-loop
  // (pose → torque) because mimosa's odometry has no angular velocity. So
  // outer gains differ in units between axis groups and get wider ranges for
  // angular so they can directly emit torque.
  const auto outer_kp_lin = make_float_slider(-20.0, 20.0,  "outer (pose→vel) Kp");
  const auto outer_ki_lin = make_float_slider(-10.0, 10.0,  "outer (pose→vel) Ki");
  const auto outer_kd_lin = make_float_slider(-10.0, 10.0,  "outer (pose→vel) Kd");
  const auto outer_im_lin = make_float_slider(0.0, 50.0,  "outer integrator clamp (<=0 disables)");
  const auto outer_kp_ang = make_float_slider(-5.0, 5.0, "pose→torque Kp");
  const auto outer_ki_ang = make_float_slider(-5.0, 5.0, "pose→torque Ki");
  const auto outer_kd_ang = make_float_slider(-5.0, 5.0, "pose→torque Kd");
  const auto outer_im_ang = make_float_slider(0.0, 200.0, "integrator clamp (<=0 disables)");
  const auto inner_kp = make_float_slider(-100.0,   100.0, "inner (velocity) Kp");
  const auto inner_ki = make_float_slider(-50.0, 50.0, "inner (velocity) Ki");
  const auto inner_kd = make_float_slider(-50.0, 50.0, "inner (velocity) Kd");
  const auto inner_im = make_float_slider(0.0, 200.0, "inner integrator clamp (<=0 disables)");
  const auto max_vel  = make_float_slider(0.0, 5.0,   "velocity clamp / joystick full-scale");
  const auto max_eff  = make_float_slider(0.0, 100.0, "effort clamp (N or Nm)");
  const auto max_effn = make_float_slider(0.0, 100.0, "effort normalization for Twist output");

  for (size_t i = 0; i < N_AXES; ++i) {
    const std::string a = AXIS_NAMES[i];
    const bool ang = AXIS_ANGULAR[i];
    this->declare_parameter("gains." + a + ".outer.kp",    ang ? 10.0 : 1.0, ang ? outer_kp_ang : outer_kp_lin);
    this->declare_parameter("gains." + a + ".outer.ki",    0.0,              ang ? outer_ki_ang : outer_ki_lin);
    this->declare_parameter("gains." + a + ".outer.kd",    0.0,              ang ? outer_kd_ang : outer_kd_lin);
    this->declare_parameter("gains." + a + ".outer.i_max", 0.0,              ang ? outer_im_ang : outer_im_lin);
    if (!ang) {
      // Inner velocity loop is linear-only; angular axes produce torque from
      // the outer pose PID directly.
      this->declare_parameter("gains." + a + ".inner.kp",    10.0, inner_kp);
      this->declare_parameter("gains." + a + ".inner.ki",    0.0,  inner_ki);
      this->declare_parameter("gains." + a + ".inner.kd",    0.0,  inner_kd);
      this->declare_parameter("gains." + a + ".inner.i_max", 0.0,  inner_im);
    }
    this->declare_parameter("limits." + a + ".max_velocity",    0.5,  max_vel);
    this->declare_parameter("limits." + a + ".max_effort",      40.0, max_eff);
    this->declare_parameter("limits." + a + ".max_effort_norm", 40.0, max_effn);
    this->declare_parameter("mode." + a + ".default", 0);

    this->declare_parameter("reference_model." + a + ".omega", 1.0, ref_omega);
    this->declare_parameter("reference_model." + a + ".zeta",  1.0, ref_zeta);
    this->declare_parameter("reference_model." + a + ".tau",   0.5, ref_tau);
  }
}

CascadedAxisConfig ControllerNode::build_axis_config(size_t i) const {
  const std::string a = AXIS_NAMES[i];
  CascadedAxisConfig c;
  c.angular     = AXIS_ANGULAR[i];
  c.outer.kp    = this->get_parameter("gains." + a + ".outer.kp").as_double();
  c.outer.ki    = this->get_parameter("gains." + a + ".outer.ki").as_double();
  c.outer.kd    = this->get_parameter("gains." + a + ".outer.kd").as_double();
  c.outer.i_max = this->get_parameter("gains." + a + ".outer.i_max").as_double();
  if (!c.angular) {
    c.inner.kp    = this->get_parameter("gains." + a + ".inner.kp").as_double();
    c.inner.ki    = this->get_parameter("gains." + a + ".inner.ki").as_double();
    c.inner.kd    = this->get_parameter("gains." + a + ".inner.kd").as_double();
    c.inner.i_max = this->get_parameter("gains." + a + ".inner.i_max").as_double();
  }
  c.max_velocity = this->get_parameter("limits." + a + ".max_velocity").as_double();
  c.max_effort   = this->get_parameter("limits." + a + ".max_effort").as_double();


  c.use_reference_model = this->get_parameter("reference_model.enabled").as_bool();
  c.ref.omega   = this->get_parameter("reference_model." + a + ".omega").as_double();
  c.ref.zeta    = this->get_parameter("reference_model." + a + ".zeta").as_double();
  c.ref.tau     = this->get_parameter("reference_model." + a + ".tau").as_double();
  c.ref.angular = c.angular;
  return c;
}

void ControllerNode::reload_config() {
  odom_timeout_ = this->get_parameter("odom_timeout").as_double();
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
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  R_W_B_.setRotation(q);
  double rpy[3];
  // getRPY returns roll/yaw in [-pi, pi] and pitch in [-pi/2, pi/2], so no
  // extra wrapping is needed before converting to degrees.
  R_W_B_.getRPY(rpy[0], rpy[1], rpy[2]);

  constexpr double RAD2DEG = 180.0 / M_PI;
  pose_[SURGE] = msg->pose.pose.position.x;
  pose_[SWAY]  = msg->pose.pose.position.y;
  pose_[HEAVE] = msg->pose.pose.position.z;
  pose_[ROLL]  = rpy[0] * RAD2DEG;
  pose_[PITCH] = rpy[1] * RAD2DEG;
  pose_[YAW]   = rpy[2] * RAD2DEG;

  // nav_msgs/Odometry twist is specified in child_frame_id (body).
  vel_[SURGE] = msg->twist.twist.linear.x;
  vel_[SWAY]  = msg->twist.twist.linear.y;
  vel_[HEAVE] = msg->twist.twist.linear.z;

  last_odom_time_ = this->now();
  have_odom_ = true;
}

void ControllerNode::ff_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  ff_vel_[SURGE] = msg->linear.x;
  ff_vel_[SWAY]  = msg->linear.y;
  ff_vel_[HEAVE] = msg->linear.z;
  ff_vel_[ROLL]  = msg->angular.x;
  ff_vel_[PITCH] = msg->angular.y;
  ff_vel_[YAW]   = msg->angular.z;
  last_ff_time_ = this->now();
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

  // Safety watchdog: if the odometry stream has gone stale, force all axes to
  // OFF, reset integrators, and hold output at zero until a fresh cmd_vel_in
  // arrives — then OFF passthrough takes over. odom_timeout <= 0 disables.
  if (odom_timeout_ > 0.0 && !safety_tripped_) {
    const double odom_age = (now - last_odom_time_).seconds();
    if (odom_age > odom_timeout_) {
      trip_safety("odometry stale: " + std::to_string(odom_age) + "s > " +
                  std::to_string(odom_timeout_) + "s timeout");
    }
  }

  std::array<double, N_AXES> effort{};
  std::array<double, N_AXES> twist_out{};

  // While safety is tripped and no fresh joystick command has come in since
  // the trip, publish zero. After the pilot touches the stick (or an external
  // node publishes cmd_vel_in), fall through to normal OFF passthrough.
  const bool hold_zero =
      safety_tripped_ && (last_ff_time_ <= safety_trip_time_);

  if (!hold_zero) {
    // Linear pose error expressed in the body frame. Only axes in FULL
    // contribute to the world-frame error vector — OFF/INNER_ONLY axes have a
    // stale pose_sp_ that would otherwise leak a bogus "desired = 0" component
    // through the rotation and corrupt the body-frame error on the FULL axes.
    tf2::Vector3 e_W(0.0, 0.0, 0.0);
    if (axes_[SURGE].mode() == AxisMode::FULL) {
      e_W.setX(axes_[SURGE].desired_pose() - pose_[SURGE]);
    }
    if (axes_[SWAY].mode() == AxisMode::FULL) {
      e_W.setY(axes_[SWAY].desired_pose() - pose_[SWAY]);
    }
    if (axes_[HEAVE].mode() == AxisMode::FULL) {
      e_W.setZ(axes_[HEAVE].desired_pose() - pose_[HEAVE]);
    }
    const tf2::Vector3 e_B = R_W_B_.transpose() * e_W;
    last_pose_error_B_ = {e_B.x(), e_B.y(), e_B.z()};
    const std::array<double, 3> pose_err_B = last_pose_error_B_;

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
        if (AXIS_ANGULAR[i]) {
          effort[i] = axes_[i].update(pose_[i], vel_[i], ff_vel_[i], dt);
        } else {
          effort[i] = axes_[i].update_with_pose_error(pose_err_B[i], vel_[i],
                                                     ff_vel_[i], dt);
        }
        twist_out[i] = (max_effort_norm_[i] > 0.0)
                           ? std::clamp(effort[i] / max_effort_norm_[i], -1.0, 1.0)
                           : 0.0;
      }
    }
  } else {
    last_pose_error_B_.fill(0.0);
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
  s.safety_tripped = safety_tripped_;

  // Angular axes are already stored in degrees; linear axes in SI units.
  for (size_t i = 0; i < N_AXES; ++i) {
    const auto mode = axes_[i].mode();

    s.modes[i]        = static_cast<uint8_t>(mode);
    s.setpoints[i]    = axes_[i].pose_setpoint();
    s.desired_pose[i] = axes_[i].desired_pose();
    s.pose[i]         = pose_[i];
    s.velocity[i]     = vel_[i];
    s.effort[i]       = last_effort_[i];

    // Pose error — wrap angular axes to [-180, 180] degrees so this matches
    // exactly what the PID sees in Pid::update (std::remainder). Linear axes
    // report the body-frame error the outer PID actually consumes (see
    // control_tick); angular axes stay in world-frame Euler.
    if (mode == AxisMode::FULL) {
      double e = AXIS_ANGULAR[i]
                     ? std::remainder(axes_[i].desired_pose() - pose_[i], 360.0)
                     : last_pose_error_B_[i];
      s.pose_error[i] = e;
    } else {
      s.pose_error[i] = 0.0;
    }

    if (mode == AxisMode::OFF || AXIS_ANGULAR[i]) {
      s.velocity_error[i] = 0.0;
    } else {
      s.velocity_error[i] = ff_vel_[i] - vel_[i];
    }
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
  if (safety_tripped_ && request->mode != 0) {
    response->success = false;
    response->message = "safety is tripped; call clear_safety first";
    return;
  }

  const auto new_mode = static_cast<AxisMode>(request->mode);
  axes_[request->axis].set_mode(new_mode);

  if (new_mode == AxisMode::FULL) {
    // Arm the reference model from the current pose so enabling FULL never
    // schedules a slew from stale x_d. Requires fresh odometry.
    if (have_odom_) {
      axes_[request->axis].arm(pose_[request->axis]);
    }
    if (request->capture_current) {
      if (!have_odom_) {
        response->success = false;
        response->message = "no odometry received yet; cannot capture setpoint";
        return;
      }
      axes_[request->axis].set_pose_setpoint(pose_[request->axis]);
    } else if (!std::isnan(request->setpoint)) {
      // NaN means "keep the stored setpoint". Any real number overwrites it.
      // Angular setpoints are in degrees; the PID's error wrapping handles
      // the [-180, 180] discontinuity.
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
  if (safety_tripped_) {
    for (size_t i = 0; i < N_AXES; ++i) {
      if (request->modes[i] != 0) {
        response->success = false;
        response->message = "safety is tripped; call clear_safety first";
        return;
      }
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
      if (have_odom_) {
        axes_[i].arm(pose_[i]);
      }
      if (request->capture_current[i]) {
        axes_[i].set_pose_setpoint(pose_[i]);
      } else if (!std::isnan(request->setpoints[i])) {
        // NaN per-axis means "keep the stored setpoint".
        axes_[i].set_pose_setpoint(request->setpoints[i]);
      }
    }
  }

  response->success = true;
  response->message = "ok";
  RCLCPP_INFO(this->get_logger(), "set_all_modes applied");
}



void ControllerNode::trip_safety(const std::string & reason) {
  safety_tripped_ = true;
  safety_trip_time_ = this->now();
  for (size_t i = 0; i < N_AXES; ++i) {
    axes_[i].set_mode(AxisMode::OFF);
    axes_[i].reset();
  }
  last_effort_.fill(0.0);
  RCLCPP_ERROR(this->get_logger(),
               "SAFETY: %s — all axes forced OFF, output held at zero until "
               "fresh cmd_vel_in. Call clear_safety to re-arm.",
               reason.c_str());
}

void ControllerNode::handle_clear_safety(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!safety_tripped_) {
    response->success = true;
    response->message = "safety not tripped; nothing to clear";
    return;
  }
  const double odom_age = (this->now() - last_odom_time_).seconds();
  if (odom_timeout_ > 0.0 && odom_age > odom_timeout_) {
    response->success = false;
    response->message = "odometry still stale (" + std::to_string(odom_age) +
                        "s); refusing to clear";
    RCLCPP_WARN(this->get_logger(), "clear_safety refused: %s",
                response->message.c_str());
    return;
  }
  safety_tripped_ = false;
  for (size_t i = 0; i < N_AXES; ++i) {
    axes_[i].reset();  // drop any integrator that's stuck from before the trip
  }
  response->success = true;
  response->message = "safety cleared; axes remain OFF — re-engage modes manually";
  RCLCPP_INFO(this->get_logger(), "safety cleared");
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
      // Snap the reference model to the fresh pose first so capturing a new
      // setpoint never introduces a transient trajectory.
      axes_[i].arm(pose_[i]);
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
