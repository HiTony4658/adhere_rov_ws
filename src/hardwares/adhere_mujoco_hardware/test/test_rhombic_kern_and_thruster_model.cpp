#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <utility>

#include "adhere_mujoco_hardware/rhombic_kern.hpp"

using adhere_mujoco_hardware::applyThrusterDeadzone;
using adhere_mujoco_hardware::applyThrusterLagStep;
using adhere_mujoco_hardware::classifyRhombicKern;
using adhere_mujoco_hardware::RhombicKernResult;

namespace
{
// Mirrors rov_wall.xml's symmetric corner-pad layout (a = 0.15 m, b = 0.25 m) used throughout
// this package's documentation/tests, but this file deliberately does not depend on MuJoCo or
// any MJCF -- classifyRhombicKern() is pure geometry/arithmetic.
const std::array<std::pair<double, double>, 4> kPadYz = {
  std::make_pair(0.15, 0.25), std::make_pair(-0.15, 0.25), std::make_pair(0.15, -0.25),
  std::make_pair(-0.15, -0.25)};
}  // namespace

// Equal load on all 4 pads: CoP at the geometric center, well inside the kern, no liftoff.
TEST(RhombicKern, EqualLoadIsWellInsideKern)
{
  const std::array<double, 4> forces = {25.0, 25.0, 25.0, 25.0};
  const RhombicKernResult r = classifyRhombicKern(forces, kPadYz);
  EXPECT_NEAR(r.cop_y, 0.0, 1e-12);
  EXPECT_NEAR(r.cop_z, 0.0, 1e-12);
  EXPECT_NEAR(r.kern_val, 0.0, 1e-12);
  EXPECT_NEAR(r.m_kern, 1.0, 1e-12);
  EXPECT_FALSE(r.is_liftoff);
  EXPECT_FALSE(r.is_corner_trap);
  EXPECT_EQ(r.liftoff_pad_count, 0);
}

// One pad at (near-)zero force: must be flagged as liftoff, and must drop out of active_pads
// accounting (active_pads = 4 - liftoff_pad_count in the caller).
TEST(RhombicKern, SinglePadLiftoffIsDetected)
{
  const std::array<double, 4> forces = {25.0, 25.0, 25.0, 0.0};
  const RhombicKernResult r = classifyRhombicKern(forces, kPadYz);
  EXPECT_TRUE(r.is_liftoff);
  EXPECT_EQ(r.liftoff_pad_count, 1);
}

// CoP pushed to a corner (TL dominant) so it lands inside the support rectangle but outside the
// inscribed diamond: K_val > 1 while every pad is still bearing some load (no liftoff) -- the
// "Corner-Triangle Trap" CLAUDE.md Sec.4.4 describes.
TEST(RhombicKern, CornerTrapWithAllPadsLoaded)
{
  const std::array<double, 4> forces = {40.0, 5.0, 5.0, 2.0};
  const RhombicKernResult r = classifyRhombicKern(forces, kPadYz);
  EXPECT_GT(r.kern_val, 1.0);
  EXPECT_LT(r.m_kern, 0.0);
  EXPECT_FALSE(r.is_liftoff);
  EXPECT_TRUE(r.is_corner_trap);
}

// All 4 pads fully detached: CoP is physically undefined. Must fall back to the documented
// defaults (cop=0, kern_val=2.0, m_kern=-1.0) rather than 0/0 -> NaN.
TEST(RhombicKern, FullyDetachedCopIsUndefinedDefault)
{
  const std::array<double, 4> forces = {0.0, 0.0, 0.0, 0.0};
  const RhombicKernResult r = classifyRhombicKern(forces, kPadYz);
  EXPECT_TRUE(r.is_liftoff);
  EXPECT_EQ(r.liftoff_pad_count, 4);
  EXPECT_NEAR(r.cop_y, 0.0, 1e-12);
  EXPECT_NEAR(r.cop_z, 0.0, 1e-12);
  EXPECT_NEAR(r.kern_val, 2.0, 1e-12);
  EXPECT_NEAR(r.m_kern, -1.0, 1e-12);
  ASSERT_TRUE(std::isfinite(r.kern_val));
}

// Hydrodynamic-accuracy audit finding 5.3: a degenerate (collinear) pad layout with L_y == 0
// must not produce NaN/Inf -- previously `kern_val = |p_y|/L_y + ...` divided by exactly zero.
TEST(RhombicKern, DegenerateCollinearPadsDoNotProduceNan)
{
  const std::array<std::pair<double, double>, 4> degenerate_pad_yz = {
    std::make_pair(0.0, 0.25), std::make_pair(0.0, 0.25), std::make_pair(0.0, -0.25),
    std::make_pair(0.0, -0.25)};
  const std::array<double, 4> forces = {10.0, 30.0, 10.0, 10.0};
  const RhombicKernResult r = classifyRhombicKern(forces, degenerate_pad_yz);
  ASSERT_TRUE(std::isfinite(r.kern_val));
  ASSERT_TRUE(std::isfinite(r.m_kern));
  ASSERT_TRUE(std::isfinite(r.cop_y));
}

// First-order ESC lag filter: a single step must move strictly partway from previous toward
// target (never overshoot, never jump all the way), and repeated application must converge.
TEST(ThrusterModel, LagStepConvergesMonotonically)
{
  constexpr double kTauThruster = 0.015;
  constexpr double kDt = 0.002;
  const double alpha = 1.0 - std::exp(-kDt / kTauThruster);

  double applied = 0.0;
  const double target = 40.0;
  double previous = applied;
  for (int i = 0; i < 2000; ++i) {
    const double next = applyThrusterLagStep(applied, target, alpha);
    EXPECT_GE(next, previous - 1e-9);  // monotonically approaching target from below
    EXPECT_LE(next, target + 1e-9);    // never overshoots
    previous = applied;
    applied = next;
  }
  EXPECT_NEAR(applied, target, 1e-3);
}

TEST(ThrusterModel, LagStepHoldsAtEquilibrium)
{
  const double alpha = 0.1;
  EXPECT_NEAR(applyThrusterLagStep(15.0, 15.0, alpha), 15.0, 1e-12);
}

TEST(ThrusterModel, DeadzoneZeroesSmallMagnitudeBothSigns)
{
  EXPECT_NEAR(applyThrusterDeadzone(1.5, 2.0), 0.0, 1e-12);
  EXPECT_NEAR(applyThrusterDeadzone(-1.5, 2.0), 0.0, 1e-12);
}

TEST(ThrusterModel, DeadzonePassesThroughAboveThreshold)
{
  EXPECT_NEAR(applyThrusterDeadzone(5.0, 2.0), 5.0, 1e-12);
  EXPECT_NEAR(applyThrusterDeadzone(-5.0, 2.0), -5.0, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
