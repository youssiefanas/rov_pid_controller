#include <gtest/gtest.h>

#include <cmath>

#include "rov_pid_controller/pid.hpp"

using rov_pid_controller::Pid;

namespace {

Pid::Config basic_cfg(double kp = 1.0, double ki = 0.0, double kd = 0.0) {
  Pid::Config c;
  c.kp = kp;
  c.ki = ki;
  c.kd = kd;
  c.out_min = -1e9;
  c.out_max =  1e9;
  c.derivative_on_measurement = true;
  c.angular = false;
  return c;
}

constexpr double DT = 0.02;  // 50 Hz

}  // namespace

// ---- Proportional ----------------------------------------------------------

TEST(PidTest, ProportionalOnlyProducesKpTimesError) {
  Pid pid(basic_cfg(2.0));
  EXPECT_DOUBLE_EQ(pid.update(10.0, 4.0, DT), 12.0);  // 2.0 * (10 - 4)
}

TEST(PidTest, ZeroErrorZeroOutput) {
  Pid pid(basic_cfg(5.0));
  EXPECT_DOUBLE_EQ(pid.update(3.0, 3.0, DT), 0.0);
}

TEST(PidTest, NonPositiveDtReturnsPreviousOutput) {
  Pid pid(basic_cfg(1.0));
  const double first = pid.update(5.0, 0.0, DT);
  EXPECT_DOUBLE_EQ(pid.update(99.0, -99.0, 0.0),  first);
  EXPECT_DOUBLE_EQ(pid.update(99.0, -99.0, -0.1), first);
}

// ---- Integral --------------------------------------------------------------

TEST(PidTest, IntegralAccumulatesOverSteps) {
  auto cfg = basic_cfg(0.0, 1.0, 0.0);  // pure I
  Pid pid(cfg);
  const double err = 2.0;
  // After N steps, i_term ≈ ki * err * N * dt.
  double last = 0.0;
  for (int i = 0; i < 10; ++i) last = pid.update(err, 0.0, DT);
  EXPECT_NEAR(last, 1.0 * err * 10 * DT, 1e-12);
}

TEST(PidTest, IntegralClampRespected) {
  auto cfg = basic_cfg(0.0, 10.0, 0.0);
  cfg.i_max = 0.5;
  Pid pid(cfg);
  for (int i = 0; i < 100; ++i) pid.update(1.0, 0.0, DT);
  EXPECT_LE(pid.update(1.0, 0.0, DT), 0.5 + 1e-12);
}

// ---- Conditional anti-windup ----------------------------------------------

TEST(PidTest, IntegratorDoesNotGrowWhenSaturatedSameDirection) {
  auto cfg = basic_cfg(10.0, 5.0, 0.0);
  cfg.out_min = -1.0;
  cfg.out_max =  1.0;
  Pid pid(cfg);

  // Error always positive, output will saturate at +1. Drive for a long time.
  for (int i = 0; i < 500; ++i) pid.update(1.0, 0.0, DT);

  // Now reverse the error: if integrator had wound up, output would be stuck
  // high until it bled off. A well-behaved anti-windup should let it respond
  // quickly.
  const double u = pid.update(-1.0, 0.0, DT);
  EXPECT_LT(u, 0.0) << "integrator did not release after saturation";
}

// ---- Derivative ------------------------------------------------------------

TEST(PidTest, DerivativeOnMeasurementDampensVelocity) {
  auto cfg = basic_cfg(0.0, 0.0, 1.0);
  cfg.derivative_on_measurement = true;
  Pid pid(cfg);

  // First call seeds state, no D kick expected.
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.0, DT), 0.0);

  // Measurement rising at 1.0/s: d = -kd * (dx/dt) = -1.0.
  EXPECT_NEAR(pid.update(0.0, DT * 1.0, DT), -1.0, 1e-12);
}

TEST(PidTest, DerivativeOnErrorSignedCorrectly) {
  auto cfg = basic_cfg(0.0, 0.0, 1.0);
  cfg.derivative_on_measurement = false;
  Pid pid(cfg);

  pid.update(0.0, 0.0, DT);              // seed
  // Setpoint jumps up by 1 => error jumps up by 1 => +d/dt => positive output.
  EXPECT_NEAR(pid.update(1.0, 0.0, DT), 1.0 / DT, 1e-9);
}

TEST(PidTest, FirstTickHasNoDerivativeKick) {
  auto cfg = basic_cfg(0.0, 0.0, 100.0);
  Pid pid(cfg);
  // Even with a huge error the first call must not emit derivative output.
  EXPECT_DOUBLE_EQ(pid.update(50.0, 0.0, DT), 0.0);
}

TEST(PidTest, AngularAxisHasNoDerivativeRegardlessOfKd) {
  auto cfg = basic_cfg(0.0, 0.0, 10.0);
  cfg.angular = true;
  Pid pid(cfg);
  pid.update(0.0, 0.0, DT);
  // Measurement rapidly changing — for a linear axis this would produce a big
  // D term; for angular it must stay zero.
  EXPECT_DOUBLE_EQ(pid.update(0.0, 45.0, DT), 0.0);
}

// ---- Output saturation -----------------------------------------------------

TEST(PidTest, OutputClampedToOutMax) {
  auto cfg = basic_cfg(100.0);
  cfg.out_min = -2.0;
  cfg.out_max =  2.0;
  Pid pid(cfg);
  EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, DT),  2.0);
  EXPECT_DOUBLE_EQ(pid.update(0.0, 10.0, DT), -2.0);
}

TEST(PidTest, RateLimitRestrictsStepDelta) {
  auto cfg = basic_cfg(1000.0);
  cfg.rate_limit = 10.0;  // units/sec
  Pid pid(cfg);
  // First tick seeds, no rate limit applied.
  (void)pid.update(1.0, 0.0, DT);
  const double u = pid.update(1.0, 0.0, DT);
  // Delta bounded by rate_limit * dt = 0.2.
  EXPECT_LE(std::abs(u - 1000.0), 1000.0);  // sanity
  EXPECT_LE(u - pid.config().rate_limit * DT - 1e-9, 1000.0);
}

// ---- Angular wrap (the bug that was fixed) --------------------------------

TEST(PidTest, AngularWrapPositiveOverflow) {
  auto cfg = basic_cfg(1.0);
  cfg.angular = true;
  Pid pid(cfg);
  // setpoint=170, measurement=-30 -> raw error=200, short way=-160.
  EXPECT_NEAR(pid.update(170.0, -30.0, DT), -160.0, 1e-9);
}

TEST(PidTest, AngularWrapNegativeOverflow) {
  auto cfg = basic_cfg(1.0);
  cfg.angular = true;
  Pid pid(cfg);
  // setpoint=-170, measurement=30 -> raw error=-200, short way=+160.
  // This is the case std::fmod used to return -200 for.
  EXPECT_NEAR(pid.update(-170.0, 30.0, DT), 160.0, 1e-9);
}

TEST(PidTest, AngularWrapExactlyOnSeam) {
  auto cfg = basic_cfg(1.0);
  cfg.angular = true;
  Pid pid(cfg);
  // Exactly ±180 — either sign is acceptable but magnitude must be 180.
  EXPECT_NEAR(std::abs(pid.update(180.0, 0.0, DT)), 180.0, 1e-9);
}

TEST(PidTest, AngularWrapSmallErrorUnchanged) {
  auto cfg = basic_cfg(2.0);
  cfg.angular = true;
  Pid pid(cfg);
  EXPECT_NEAR(pid.update(30.0, 10.0, DT), 40.0, 1e-9);  // 2.0 * 20
}

// ---- Reset -----------------------------------------------------------------

TEST(PidTest, ResetClearsIntegrator) {
  auto cfg = basic_cfg(0.0, 1.0, 0.0);
  Pid pid(cfg);
  for (int i = 0; i < 20; ++i) pid.update(1.0, 0.0, DT);
  pid.reset();
  // After reset, integrator should be empty again.
  EXPECT_NEAR(pid.update(1.0, 0.0, DT), 1.0 * 1.0 * DT, 1e-12);
}

TEST(PidTest, ResetClearsDerivativeState) {
  auto cfg = basic_cfg(0.0, 0.0, 100.0);
  Pid pid(cfg);
  pid.update(0.0, 0.0, DT);
  pid.update(0.0, 1.0, DT);
  pid.reset();
  // After reset the next call is "first" again -> no D kick.
  EXPECT_DOUBLE_EQ(pid.update(0.0, 50.0, DT), 0.0);
}
