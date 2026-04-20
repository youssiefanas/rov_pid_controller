#include <gtest/gtest.h>

#include <cmath>

#include "rov_pid_controller/cascaded_axis.hpp"

using rov_pid_controller::AxisMode;
using rov_pid_controller::CascadedAxis;
using rov_pid_controller::CascadedAxisConfig;

namespace {

CascadedAxisConfig linear_cfg() {
  CascadedAxisConfig c;
  c.angular = false;
  c.outer.kp = 1.0;
  c.inner.kp = 10.0;
  c.max_velocity = 0.5;   // clamp outer output
  c.max_effort   = 40.0;  // clamp inner output
  return c;
}

CascadedAxisConfig angular_cfg() {
  CascadedAxisConfig c;
  c.angular = true;
  c.outer.kp = 2.0;   // produces torque directly from pose error (deg)
  c.max_effort = 30.0;
  return c;
}

constexpr double DT = 0.02;

}  // namespace

// ---- OFF -------------------------------------------------------------------

TEST(CascadedAxisTest, OffModeProducesZeroEffort) {
  CascadedAxis ax(linear_cfg());
  ax.set_mode(AxisMode::OFF);
  EXPECT_DOUBLE_EQ(ax.update(10.0, 0.0, 1.0, DT), 0.0);
}

TEST(CascadedAxisTest, AngularOffModeProducesZeroEffort) {
  CascadedAxis ax(angular_cfg());
  ax.set_mode(AxisMode::OFF);
  EXPECT_DOUBLE_EQ(ax.update(10.0, 0.0, 1.0, DT), 0.0);
}

// ---- Linear INNER_ONLY -----------------------------------------------------

TEST(CascadedAxisLinearTest, InnerOnlyUsesExternalVelocitySetpoint) {
  CascadedAxis ax(linear_cfg());
  ax.set_mode(AxisMode::INNER_ONLY);
  // Inner kp=10, vel error = 1.0 - 0.0 = 1.0 => effort = 10.0.
  EXPECT_DOUBLE_EQ(ax.update(0.0, 0.0, 1.0, DT), 10.0);
}

// ---- Linear FULL cascade ---------------------------------------------------

TEST(CascadedAxisLinearTest, FullCascadeDrivesEffortTowardsSetpoint) {
  auto cfg = linear_cfg();
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(5.0);

  // Positive pose error should yield positive effort on the first tick.
  const double u = ax.update(0.0, 0.0, 0.0, DT);
  EXPECT_GT(u, 0.0);
  EXPECT_LE(std::abs(u), cfg.max_effort);
}

TEST(CascadedAxisLinearTest, EffortClampedToMaxEffort) {
  auto cfg = linear_cfg();
  cfg.outer.kp = 100.0;
  cfg.inner.kp = 1000.0;
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(50.0);

  const double u = ax.update(0.0, 0.0, 0.0, DT);
  EXPECT_LE(std::abs(u), cfg.max_effort + 1e-9);
}

TEST(CascadedAxisLinearTest, OuterVelocityCommandIsClamped) {
  // With outer clamp at 0.5 m/s, large pose errors must not push the inner
  // loop past that velocity setpoint. We detect this by comparing the effort
  // we get against the effort we'd get if we fed the clamped velocity directly
  // in INNER_ONLY mode.
  auto cfg = linear_cfg();
  cfg.outer.kp = 1000.0;  // outer will definitely saturate
  CascadedAxis ax_full(cfg);
  ax_full.set_mode(AxisMode::FULL);
  ax_full.set_pose_setpoint(10.0);
  const double u_full = ax_full.update(0.0, 0.0, 0.0, DT);

  CascadedAxis ax_inner(cfg);
  ax_inner.set_mode(AxisMode::INNER_ONLY);
  const double u_inner = ax_inner.update(0.0, 0.0, cfg.max_velocity, DT);

  EXPECT_NEAR(u_full, u_inner, 1e-9);
}

TEST(CascadedAxisLinearTest, CascadeConvergesAtSetpoint) {
  // At the setpoint with zero velocity, effort should be exactly zero on the
  // first tick (no I wind-up yet, no D kick on first tick).
  CascadedAxis ax(linear_cfg());
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(3.0);
  EXPECT_DOUBLE_EQ(ax.update(3.0, 0.0, 0.0, DT), 0.0);
}

// ---- Angular single-loop ---------------------------------------------------

TEST(CascadedAxisAngularTest, FullModeOutputsTorqueProportionalToError) {
  auto cfg = angular_cfg();
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(5.0);
  // error = 5 - 0 = 5 deg, kp = 2 => torque = 10.
  EXPECT_NEAR(ax.update(0.0, 0.0, 0.0, DT), 10.0, 1e-9);
}

TEST(CascadedAxisAngularTest, TorqueClampedToMaxEffort) {
  auto cfg = angular_cfg();
  cfg.outer.kp = 100.0;
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(90.0);
  EXPECT_NEAR(ax.update(0.0, 0.0, 0.0, DT), cfg.max_effort, 1e-9);
}

TEST(CascadedAxisAngularTest, ShortWayAcrossSeamFromNegativeToPositive) {
  // setpoint=+170, measurement=-170. The short path is -20 deg (rotate
  // backwards through the ±180 seam), NOT +340.
  auto cfg = angular_cfg();
  cfg.outer.kp = 1.0;  // keep output well within max_effort for exact checks
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(170.0);
  const double u = ax.update(-170.0, 0.0, 0.0, DT);
  EXPECT_LT(u, 0.0) << "must take the -20 short path, not +340";
  EXPECT_NEAR(u, -20.0, 1e-9);
}

TEST(CascadedAxisAngularTest, ShortWayAcrossSeamFromPositiveToNegative) {
  // setpoint=-170, measurement=+170. Short path is +20 (the previous
  // std::fmod-based wrap returned -200 here, which is the bug we fixed).
  auto cfg = angular_cfg();
  cfg.outer.kp = 1.0;
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(-170.0);
  const double u = ax.update(170.0, 0.0, 0.0, DT);
  EXPECT_GT(u, 0.0) << "must take the +20 short path (regression guard)";
  EXPECT_NEAR(u, 20.0, 1e-9);
}

TEST(CascadedAxisAngularTest, InnerOnlyIsNoopForAngular) {
  // Angular axes have no inner velocity loop configured; INNER_ONLY falls
  // through to zero per the CascadedAxis implementation.
  CascadedAxis ax(angular_cfg());
  ax.set_mode(AxisMode::INNER_ONLY);
  EXPECT_DOUBLE_EQ(ax.update(10.0, 0.0, 1.0, DT), 0.0);
}

// ---- Mode / reset transitions ---------------------------------------------

TEST(CascadedAxisTest, ModeTransitionResetsIntegrators) {
  auto cfg = linear_cfg();
  cfg.outer.ki = 5.0;
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(10.0);
  // Wind up the outer integrator.
  for (int i = 0; i < 20; ++i) ax.update(0.0, 0.0, 0.0, DT);

  ax.set_mode(AxisMode::OFF);
  ax.set_mode(AxisMode::FULL);

  // After a round-trip through OFF, the integrator must be cleared so a zero
  // error produces zero effort.
  EXPECT_DOUBLE_EQ(ax.update(10.0, 0.0, 0.0, DT), 0.0);
}

TEST(CascadedAxisTest, ExplicitResetClearsState) {
  auto cfg = angular_cfg();
  cfg.outer.ki = 5.0;
  CascadedAxis ax(cfg);
  ax.set_mode(AxisMode::FULL);
  ax.set_pose_setpoint(30.0);
  for (int i = 0; i < 10; ++i) ax.update(0.0, 0.0, 0.0, DT);
  ax.reset();
  // At setpoint after reset: no I, no D -> zero.
  EXPECT_DOUBLE_EQ(ax.update(30.0, 0.0, 0.0, DT), 0.0);
}
