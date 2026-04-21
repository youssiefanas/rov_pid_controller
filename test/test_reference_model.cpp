#include <gtest/gtest.h>
 
#include <cmath>
 
#include "rov_pid_controller/reference_model.hpp"
 
using rov_pid_controller::ReferenceModel;
 
namespace {
 
ReferenceModel::Config linear_cfg() {
  ReferenceModel::Config c;
  c.omega = 2.0;
  c.zeta  = 1.0;
  c.tau   = 0.2;
  c.angular = false;
  return c;
}
 
ReferenceModel::Config angular_cfg() {
  ReferenceModel::Config c;
  c.omega = 2.0;
  c.zeta  = 1.0;
  c.tau   = 0.2;
  c.angular = true;
  return c;
}
 
constexpr double DT = 0.02;
 
void run_for(ReferenceModel & m, double duration) {
  const int steps = static_cast<int>(std::round(duration / DT));
  for (int i = 0; i < steps; ++i) m.update(DT);
}
 
}  // namespace
 
TEST(ReferenceModelTest, DefaultStateIsZero) {
  ReferenceModel m(linear_cfg());
  EXPECT_DOUBLE_EQ(m.x_d(), 0.0);
  EXPECT_DOUBLE_EQ(m.v_d(), 0.0);
  EXPECT_DOUBLE_EQ(m.a_d(), 0.0);
}
 
TEST(ReferenceModelTest, SteadyStateConvergesToSetpoint) {
  ReferenceModel m(linear_cfg());
  m.snap_to(0.0);
  m.set_setpoint(5.0);
  // ~10 seconds >> settling time for ω=2, ζ=1 with τ=0.2.
  run_for(m, 10.0);
  EXPECT_NEAR(m.x_d(), 5.0, 1e-3);
  EXPECT_NEAR(m.v_d(), 0.0, 1e-3);
  EXPECT_NEAR(m.a_d(), 0.0, 1e-3);
}
 
TEST(ReferenceModelTest, CriticalDampingHasNoOvershoot) {
  auto cfg = linear_cfg();
  cfg.zeta = 1.0;
  cfg.tau  = 1e-6;  // effectively bypass the first-order filter
  ReferenceModel m(cfg);
  m.snap_to(0.0);
  m.set_setpoint(1.0);
 
  double peak = 0.0;
  for (int i = 0; i < 1000; ++i) {
    m.update(DT);
    peak = std::max(peak, m.x_d());
  }
  // At critical damping, the second-order response approaches the setpoint
  // monotonically. Allow a hair of numerical slack.
  EXPECT_LE(peak, 1.0 + 1e-6);
}
 
TEST(ReferenceModelTest, SettlingTimeMatchesBandwidth) {
  auto cfg = linear_cfg();
  cfg.omega = 2.0;
  cfg.zeta  = 1.0;
  cfg.tau   = 0.25;
  ReferenceModel m(cfg);
  m.snap_to(0.0);
  m.set_setpoint(1.0);
  // Rough 5% settling bound: a few time constants of the slowest mode.
  const double budget = 3.0 * (1.0 / cfg.omega + cfg.tau);
  run_for(m, budget * 2.5);  // plenty of margin
  EXPECT_LT(std::abs(1.0 - m.x_d()), 0.05);
}
 
TEST(ReferenceModelTest, SnapToZeroesVelocityAndAccel) {
  ReferenceModel m(linear_cfg());
  m.set_setpoint(10.0);
  run_for(m, 0.5);  // build up some v_d / a_d
  EXPECT_GT(std::abs(m.v_d()), 0.0);
 
  m.snap_to(3.0);
  EXPECT_DOUBLE_EQ(m.x_d(), 3.0);
  EXPECT_DOUBLE_EQ(m.v_d(), 0.0);
  EXPECT_DOUBLE_EQ(m.a_d(), 0.0);
 
  // With the raw setpoint now also snapped to 3.0, one more tick must not
  // generate motion.
  m.update(DT);
  EXPECT_NEAR(m.x_d(), 3.0, 1e-12);
  EXPECT_NEAR(m.v_d(), 0.0, 1e-12);
}
 
TEST(ReferenceModelTest, AngularShortPathFromPositiveToNegative) {
  ReferenceModel m(angular_cfg());
  m.snap_to(170.0);
  m.set_setpoint(-170.0);  // short path is +20°, not -340°
 
  // First tick — v_d must be positive (heading toward +180° → -180° seam).
  m.update(DT);
  EXPECT_GT(m.v_d(), 0.0) << "reference model took the long way around";
 
  // Converges (allowing x_d to pass through ±180°; the PID wraps the error).
  run_for(m, 10.0);
  // After convergence, x_d should equal the shortest-path target: 170 + 20 = 190.
  EXPECT_NEAR(m.x_d(), 190.0, 1e-3);
}
 
TEST(ReferenceModelTest, AngularShortPathFromNegativeToPositive) {
  ReferenceModel m(angular_cfg());
  m.snap_to(-170.0);
  m.set_setpoint(170.0);  // short path is -20°
 
  m.update(DT);
  EXPECT_LT(m.v_d(), 0.0);
 
  run_for(m, 10.0);
  EXPECT_NEAR(m.x_d(), -190.0, 1e-3);
}
 
TEST(ReferenceModelTest, ZeroDtReturnsCurrentXd) {
  ReferenceModel m(linear_cfg());
  m.snap_to(4.0);
  m.set_setpoint(9.0);
  EXPECT_DOUBLE_EQ(m.update(0.0), 4.0);
  EXPECT_DOUBLE_EQ(m.update(-1.0), 4.0);
}
 
TEST(ReferenceModelTest, NoFilterWhenTauIsZero) {
  auto cfg = linear_cfg();
  cfg.tau = 0.0;  // disables the first-order filter
  ReferenceModel m(cfg);
  m.snap_to(0.0);
  m.set_setpoint(1.0);
  // x_ref jumps instantly to raw_sp, so after many steps x_d reaches 1.
  run_for(m, 10.0);
  EXPECT_NEAR(m.x_d(), 1.0, 1e-3);
}
 
TEST(ReferenceModelTest, ReconfigurePreservesState) {
  ReferenceModel m(linear_cfg());
  m.snap_to(2.0);
  m.set_setpoint(2.0);
 
  auto cfg = linear_cfg();
  cfg.omega = 0.5;
  m.set_config(cfg);
  // State survives; at the setpoint, x_d must stay put.
  m.update(DT);
  EXPECT_NEAR(m.x_d(), 2.0, 1e-9);
}