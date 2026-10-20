// Pure-CUDA Baseline MPPI verification contract (CLAUDE.md Phase 4.1 Stage 3): exercises the
// real GPU kernels on this workstation's Quadro RTX5000 (no CPU mock) -- determinism/latency/
// thruster-bound safety (Test 1), directional cost-gradient sanity (Test 2), and closed-loop
// position tracking through StateBaselineMppi's 250Hz ZOH driven by direct solve() calls at the
// 25Hz outer-loop rate against a fake point-mass plant (Test 3), same idiom as
// test_fsm_states.cpp's FakePlant (no real MuJoCo process).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiConfig.h"
#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiController.h"
#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/OutputSafety.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"
#include "adhere_controller/FSM/StateBaselineMppi.h"

namespace
{

using adhere_controller::CtrlInterfaces;
using adhere_controller::Mat68;
using adhere_controller::Mat86;
using adhere_controller::ModelParams;
using adhere_controller::Quat;
using adhere_controller::sanitizeAndWriteThrusterCommands;
using adhere_controller::Vec3;
using adhere_controller::Vec6;
using adhere_controller::Vec8;
using adhere_controller::baseline_mppi::BaselineMppiConfig;
using adhere_controller::baseline_mppi::BaselineMppiController;

const ModelParams kModelParams;

constexpr double kDt = 0.004;       // 250Hz inner loop
constexpr double kOuterDt = 0.04;   // 25Hz outer loop (matches BaselineMppiConfig::dt default)
constexpr int kInnerPerOuter = 10;  // outer_decimation_=20 at 500Hz -> 10 inner ticks per outer

constexpr double kMass = 26.0;
constexpr double kNetBuoyancyZ = 1.962;  // N, rho*g*V - m_dry*g (matches test_fsm_states.cpp)
const Vec3 kDLin(30.0, 20.0, 25.0);
const Vec3 kDQuad(140.0, 50.0, 70.0);

// Phase 4.2: one-sided wall-contact spring (same idiom as test_adhere_wall_states.cpp's
// FakeWallPlant) -- needed now that Test 3 below drives the target all the way onto the wall
// plane (WALL_ADHERE), not just an arbitrary free-swimming setpoint; without it there is nothing
// to stop the point-mass coasting straight through x=0.
constexpr double kWallStiffness = 4000.0;
constexpr double kContactStartX = 0.05;

Mat68 buildB()
{
  return adhere_controller::buildThrusterAllocationMatrix(
    kModelParams.thruster_pos, kModelParams.thruster_dir);
}

Mat86 buildBPinv(const Mat68 & b)
{
  return adhere_controller::buildThrusterAllocationPseudoInverse(b);
}

// ------------------------------------------------------------------------------------------
// Test 1: kernel determinism/latency/thruster-bound safety (CLAUDE.md Phase 4.1 Stage 3.1).
// ------------------------------------------------------------------------------------------
TEST(BaselineMppi, SolveIsDeterministicFastAndBounded)
{
  const Mat68 b = buildB();
  const Mat86 b_pinv = buildBPinv(b);
  BaselineMppiConfig cfg;  // defaults: K=1024, H=30

  BaselineMppiController a, c;
  a.init(kModelParams, b, b_pinv, cfg);
  c.init(kModelParams, b, b_pinv, cfg);

  const Vec3 pos(0.0, 0.0, -2.0);
  const Quat quat(1.0, 0.0, 0.0, 0.0);
  const Vec3 vel = Vec3::Zero();
  const Vec3 omega = Vec3::Zero();
  const Vec3 target_pos(0.2, -0.1, -2.3);
  const Quat target_quat(1.0, 0.0, 0.0, 0.0);

  a.warmStart(pos, quat);
  c.warmStart(pos, quat);

  const auto ua = a.solve(pos, quat, vel, omega, target_pos, target_quat);
  EXPECT_LT(a.lastSolveMs(), 5.0)
    << "K=1024,H=30 solve must stay well under the 40ms outer-loop budget";

  const auto uc = c.solve(pos, quat, vel, omega, target_pos, target_quat);

  for (int i = 0; i < 8; ++i) {
    EXPECT_TRUE(std::isfinite(ua[i])) << "thruster " << i;
    EXPECT_LE(std::abs(ua[i]), kModelParams.thruster_max[i] + 1e-6) << "thruster " << i;
    // Two independently-initialized controllers with the SAME fixed seed and the SAME input
    // snapshot must produce bit-for-bit-reproducible (up to float round-trip) output -- this is
    // what "deterministic" means for an MPPI engine whose internal RNG stream otherwise evolves
    // call-to-call within a single instance.
    EXPECT_NEAR(ua[i], uc[i], 1e-4)
      << "thruster " << i << " not reproducible across identical seeds";
  }
}

// ------------------------------------------------------------------------------------------
// Test 2: directional cost-gradient sanity (CLAUDE.md Phase 4.1 Stage 3.1). Each sub-case
// re-solves several times in a row (receding-horizon iteration, warm-starting from the previous
// call's nominal sequence) rather than trusting a single MPPI update step -- the real outer-loop
// worker always runs this way, and a single update from a flat warmStart() can be dominated by
// sampling noise rather than the target-dependent cost gradient this test wants to isolate.
TEST(BaselineMppi, ProducesRestoringWrenchTowardTarget)
{
  const Mat68 b = buildB();
  const Mat86 b_pinv = buildBPinv(b);
  BaselineMppiConfig cfg;
  constexpr int kIterations = 15;

  // x=0.5m, safely clear of the x_wall=0.0 (+/-0.10m preload-activation band) so this test
  // isolates the position/attitude tracking cost from the wall-pressing preload bias.
  const Vec3 pos(0.5, 0.0, -2.0);
  const Quat quat(1.0, 0.0, 0.0, 0.0);
  const Vec3 vel = Vec3::Zero();
  const Vec3 omega = Vec3::Zero();

  // (a) Target directly above (less negative Z, i.e. shallower) should command a net +Fz
  // thruster allocation larger than tracking the current depth exactly.
  {
    BaselineMppiController at_target, above_target;
    at_target.init(kModelParams, b, b_pinv, cfg);
    above_target.init(kModelParams, b, b_pinv, cfg);

    at_target.warmStart(pos, quat);
    above_target.warmStart(pos, quat);

    std::array<double, 8> u_at{}, u_above{};
    for (int it = 0; it < kIterations; ++it) {
      u_at = at_target.solve(pos, quat, vel, omega, pos, quat);
      u_above = above_target.solve(pos, quat, vel, omega, pos + Vec3(0, 0, 1.0), quat);
    }

    Vec8 ua, ub;
    for (int i = 0; i < 8; ++i) {
      ua(i) = u_at[i];
      ub(i) = u_above[i];
    }
    const double fz_at = (b * ua)(2);
    const double fz_above = (b * ub)(2);
    EXPECT_GT(fz_above, fz_at)
      << "a target 1m shallower should command strictly more net +Z thrust than holding depth";
  }

  // (b) Target yaw ahead (+90deg) vs. behind (-90deg) of the current heading should produce
  // opposite-signed net yaw torque.
  {
    BaselineMppiController yaw_ahead, yaw_behind;
    yaw_ahead.init(kModelParams, b, b_pinv, cfg);
    yaw_behind.init(kModelParams, b, b_pinv, cfg);

    yaw_ahead.warmStart(pos, quat);
    yaw_behind.warmStart(pos, quat);

    const Quat q_plus90 = adhere_controller::quatFromRPY(Vec3(0.0, 0.0, 90.0 * M_PI / 180.0));
    const Quat q_minus90 = adhere_controller::quatFromRPY(Vec3(0.0, 0.0, -90.0 * M_PI / 180.0));

    std::array<double, 8> u_plus{}, u_minus{};
    for (int it = 0; it < kIterations; ++it) {
      u_plus = yaw_ahead.solve(pos, quat, vel, omega, pos, q_plus90);
      u_minus = yaw_behind.solve(pos, quat, vel, omega, pos, q_minus90);
    }

    Vec8 up, um;
    for (int i = 0; i < 8; ++i) {
      up(i) = u_plus[i];
      um(i) = u_minus[i];
    }
    const double tz_plus = (b * up)(5);
    const double tz_minus = (b * um)(5);
    EXPECT_GT(tz_plus, tz_minus)
      << "a +90deg-ahead yaw target must command more net +Tz torque than a -90deg-behind one";
  }
}

// ------------------------------------------------------------------------------------------
// Test 3: closed-loop position tracking through StateBaselineMppi's 250Hz ZOH, driven by direct
// solve() calls at the 25Hz outer-loop rate against a fake translational point-mass plant (same
// idiom as test_fsm_states.cpp's FakePlant -- never a real MuJoCo process).
// ------------------------------------------------------------------------------------------
class FakePlant
{
public:
  FakePlant()
  {
    for (int i = 0; i < 8; ++i) {
      cmd_handles_.emplace_back(kThrusterNames_[i], "effort", &thr_cmd_[i]);
      loaned_cmd_.emplace_back(cmd_handles_.back());
      ci.thruster_command_interface_.emplace_back(loaned_cmd_.back());
    }
  }

  void setPosition(double x, double y, double z)
  {
    pos_ = Vec3(x, y, z);
    ci.active_state.pos_w[0] = x;
    ci.active_state.pos_w[1] = y;
    ci.active_state.pos_w[2] = z;
    ci.active_state.quat_w[0] = 1.0;
  }

  void step(double dt)
  {
    Vec8 u;
    for (int i = 0; i < 8; ++i) {
      u(i) = thr_cmd_[i];
    }
    static const Mat68 b = buildB();
    const Vec3 f = (b * u).head<3>();
    const Vec3 buoy(0.0, 0.0, kNetBuoyancyZ);
    const Vec3 drag =
      kDLin.cwiseProduct(vel_) + kDQuad.cwiseProduct(vel_.cwiseProduct(vel_.cwiseAbs()));
    double f_contact_x = 0.0;
    if (pos_.x() < kContactStartX) {
      f_contact_x = kWallStiffness * (kContactStartX - pos_.x());
    }
    const Vec3 contact(f_contact_x, 0.0, 0.0);
    const Vec3 accel = (f + buoy - drag + contact) / kMass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    ci.active_state.pos_w[0] = pos_.x();
    ci.active_state.pos_w[1] = pos_.y();
    ci.active_state.pos_w[2] = pos_.z();
    ci.active_state.vel_body[0] = vel_.x();
    ci.active_state.vel_body[1] = vel_.y();
    ci.active_state.vel_body[2] = vel_.z();
  }

  const Vec3 & position() const { return pos_; }

  void writeCommands() { sanitizeAndWriteThrusterCommands(ci, kModelParams, violations_); }

  CtrlInterfaces ci;

private:
  static constexpr std::array<const char *, 8> kThrusterNames_ = {
    "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};

  std::deque<hardware_interface::CommandInterface> cmd_handles_;
  std::deque<hardware_interface::LoanedCommandInterface> loaned_cmd_;
  std::array<double, 8> thr_cmd_ = {0, 0, 0, 0, 0, 0, 0, 0};
  Vec3 pos_ = Vec3::Zero();
  Vec3 vel_ = Vec3::Zero();
  uint64_t violations_ = 0;
};

TEST(BaselineMppi, ClosedLoopPositionTrackingWithin15cm)
{
  const Mat68 b = buildB();
  const Mat86 b_pinv = buildBPinv(b);
  BaselineMppiConfig cfg;

  BaselineMppiController planner;
  planner.init(kModelParams, b, b_pinv, cfg);

  // Phase 4.2 two-stage adhere sequence: BASELINE_MPPI is now also a wall-adhesion FSM state
  // (same as ADHERE_PID/ADHERE_SMC) -- StateBaselineMppi::run() ignores/overrides any
  // setpoint-driven target until DEPTH_ALIGN completes, then re-targets onto the wall plane for
  // WALL_ADHERE (CLAUDE.md Phase 4.2 Sec.3). This test therefore exercises the full two-stage
  // sequence (not an arbitrary free-swimming setpoint, which Test 2 above already covers via
  // direct solve() calls) and checks convergence to the shared adhere_stage target, same idiom
  // as test_adhere_wall_states.cpp's StateAdherePid/StateAdhereSmc tests.
  FakePlant plant;
  plant.setPosition(1.0, 0.1, -2.1);

  adhere_controller::StateBaselineMppi state(plant.ci, kModelParams, planner);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

  const int kOuterSteps = static_cast<int>(20.0 / kOuterDt);  // 20.0s simulated
  for (int outer = 0; outer < kOuterSteps; ++outer) {
    const auto target = state.targetPose();
    const Vec3 pos(
      plant.ci.active_state.pos_w[0], plant.ci.active_state.pos_w[1],
      plant.ci.active_state.pos_w[2]);
    const Quat quat(
      plant.ci.active_state.quat_w[0], plant.ci.active_state.quat_w[1],
      plant.ci.active_state.quat_w[2], plant.ci.active_state.quat_w[3]);
    const Vec3 vel(
      plant.ci.active_state.vel_body[0], plant.ci.active_state.vel_body[1],
      plant.ci.active_state.vel_body[2]);
    const auto u_star = planner.solve(pos, quat, vel, Vec3::Zero(), target.pos, target.quat);
    for (int i = 0; i < 8; ++i) {
      plant.ci.outer_loop_cmd_.u_star[i].store(u_star[i]);
    }
    plant.ci.outer_loop_cmd_.valid.store(true);

    for (int inner = 0; inner < kInnerPerOuter; ++inner) {
      state.run(t0, period);
      plant.writeCommands();
      plant.step(kDt);
    }
  }

  EXPECT_TRUE(state.isDepthAligned());
  EXPECT_NEAR(plant.position().x(), planner.xWall(), 0.15);
  EXPECT_NEAR(plant.position().y(), kModelParams.adhere_stage.target_y, 0.15);
  EXPECT_NEAR(plant.position().z(), -kModelParams.adhere_stage.target_depth_m, 0.15);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
