// Header-only ControllerMetrics.h contract (CLAUDE.md Phase 3.8 Stage 3.1): no rclcpp/
// hardware_interface dependency, so the efficiency/chattering/safety telemetry math is
// verifiable in isolation, same style as test_loop_profiler.cpp/test_dynamics_and_pid.cpp's
// OutputSafetyGate suite.
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "adhere_controller/common/ControllerMetrics.h"
#include "adhere_controller/dynamics/ModelParams.h"

namespace
{

using adhere_controller::ControllerMetricsAccumulator;
using adhere_controller::ModelParams;

const ModelParams kModelParams;
constexpr double kDt = 0.004;

}  // namespace

TEST(ControllerMetrics, FirstSampleHasZeroSlewRate)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 8> u = {10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 4> contact_forces = {0.0, 0.0, 0.0, 0.0};

  const auto sample = acc.update(u, contact_forces, kModelParams, kDt);
  EXPECT_DOUBLE_EQ(sample.thrust_slew_rate_l2, 0.0);
}

TEST(ControllerMetrics, SlewRateMatchesHandComputedDeltaUOverDt)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 4> contact_forces = {0.0, 0.0, 0.0, 0.0};

  acc.update({10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, contact_forces, kModelParams, kDt);
  const auto sample =
    acc.update({20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, contact_forces, kModelParams, kDt);

  // ||delta_u||_2 / dt = |20-10| / 0.004 = 2500.
  EXPECT_NEAR(sample.thrust_slew_rate_l2, 2500.0, 1e-6);
}

TEST(ControllerMetrics, CumulativeEnergyAndImpulseGrowMonotonicallyAndMatchIntegral)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 8> u = {10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 4> contact_forces = {0.0, 0.0, 0.0, 0.0};

  // ||u||_2^2 = 400, ||u||_1 = 40 at every tick (constant input).
  double prev_energy = -1.0;
  double prev_impulse = -1.0;
  adhere_controller::ControllerMetricsSample sample;
  for (int i = 0; i < 10; ++i) {
    sample = acc.update(u, contact_forces, kModelParams, kDt);
    EXPECT_GT(sample.cumulative_energy_l2, prev_energy);
    EXPECT_GT(sample.cumulative_impulse_l1, prev_impulse);
    prev_energy = sample.cumulative_energy_l2;
    prev_impulse = sample.cumulative_impulse_l1;
  }
  EXPECT_NEAR(sample.cumulative_energy_l2, 400.0 * kDt * 10, 1e-9);
  EXPECT_NEAR(sample.cumulative_impulse_l1, 40.0 * kDt * 10, 1e-9);
}

TEST(ControllerMetrics, ResetClearsCumulativesAndPreviousSampleSlew)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 8> u = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 4> contact_forces = {0.0, 0.0, 0.0, 0.0};

  acc.update(u, contact_forces, kModelParams, kDt);
  acc.update(u, contact_forces, kModelParams, kDt);
  acc.reset();

  // A large jump right after reset must NOT be reported as slew (no spurious cross-state-
  // transition artifact), and cumulative integrators restart from zero.
  const auto sample =
    acc.update({90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, contact_forces, kModelParams, kDt);
  EXPECT_DOUBLE_EQ(sample.thrust_slew_rate_l2, 0.0);
  EXPECT_NEAR(sample.cumulative_energy_l2, 90.0 * 90.0 * kDt, 1e-9);
  EXPECT_NEAR(sample.cumulative_impulse_l1, 90.0 * kDt, 1e-9);
}

TEST(ControllerMetrics, SaturatedThrusterCountTriggersAtNinetyEightPercentBoundary)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 4> contact_forces = {0.0, 0.0, 0.0, 0.0};
  const double max = kModelParams.thruster_max[0];  // 40.0N default

  // Just below the 98% boundary: not saturated.
  std::array<double, 8> u_below = {0.98 * max - 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto sample = acc.update(u_below, contact_forces, kModelParams, kDt);
  EXPECT_EQ(sample.saturated_thruster_count, 0);

  // Exactly at the 98% boundary: saturated (>= comparison).
  std::array<double, 8> u_at = {0.98 * max, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sample = acc.update(u_at, contact_forces, kModelParams, kDt);
  EXPECT_EQ(sample.saturated_thruster_count, 1);

  // Negative rail (thruster_min) also counts.
  const double min = kModelParams.thruster_min[0];  // -40.0N default
  std::array<double, 8> u_neg = {0.98 * min, 0.98 * min, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sample = acc.update(u_neg, contact_forces, kModelParams, kDt);
  EXPECT_EQ(sample.saturated_thruster_count, 2);
}

TEST(ControllerMetrics, ActiveContactPadCountUsesContactForceThreshold)
{
  ControllerMetricsAccumulator acc;
  const std::array<double, 8> u = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const double thresh = kModelParams.contact_force_thresh;  // 2.0N default

  const std::array<double, 4> contact_forces = {thresh + 0.1, thresh - 0.1, 0.0, thresh + 5.0};
  const auto sample = acc.update(u, contact_forces, kModelParams, kDt);
  EXPECT_EQ(sample.active_contact_pad_count, 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
