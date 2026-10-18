// Closed-loop wall-adhesion FSM state contract (CLAUDE.md Phase 3.8 Stage 3.1). Extends the
// test_fsm_states.cpp FakePlant pattern with a one-sided wall-contact spring model (CLAUDE.md
// Sec.4.2's unilateral Kelvin-Voigt model, simplified to a pure spring -- no damping term needed
// for a converge-and-hold gtest): once pos.x() <= x_wall, a +X reaction force pushes back and
// the same penetration synthesizes 4 equal pad normal forces (plant orientation held at
// identity throughout, so the load is always perfectly symmetric across the 4 pads).
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/OutputSafety.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"
#include "adhere_controller/FSM/StateAdherePid.h"
#include "adhere_controller/FSM/StateAdhereSmc.h"

namespace
{

using adhere_controller::CtrlInterfaces;
using adhere_controller::Mat48;
using adhere_controller::Mat68;
using adhere_controller::ModelParams;
using adhere_controller::sanitizeAndWriteThrusterCommands;
using adhere_controller::Vec3;
using adhere_controller::Vec4;
using adhere_controller::Vec8;

const ModelParams kModelParams;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;

// This test's simplified point-mass + one-sided spring contact model has a much shallower
// passive equilibrium penetration than the real MuJoCo contact geometry (no pad lever arm, no
// multi-pad asymmetry), and the production dock_force_thresh/x_dock_eps are tuned against the
// real plant's integral-windup timescale (verified separately via the Phase 3.8 manual MuJoCo
// closed-loop smoke test, not here). Loosen just the two dock-gate thresholds for this
// synthetic plant so the test exercises the APPROACH->ADHERED transition logic itself on a
// tractable timescale, without needing to wait for slow integral windup against an unrealistic
// spring. Position/attitude/force-loop gains are left at production defaults.
ModelParams makeTestModelParams()
{
  ModelParams p = kModelParams;
  p.adhere_pid.dock_force_thresh = 5.0;
  p.adhere_pid.x_dock_eps = 0.06;
  p.adhere_smc.dock_force_thresh = 5.0;
  p.adhere_smc.x_dock_eps = 0.06;
  return p;
}
const ModelParams kTestModelParams = makeTestModelParams();

constexpr std::array<const char *, 8> kThrusterNames = {
  "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};

constexpr double kMass = 26.0;
constexpr double kNetBuoyancyZ = 1.962;  // N, rho*g*V - m_dry*g
const Vec3 kDLin(30.0, 20.0, 25.0);
const Vec3 kDQuad(140.0, 50.0, 70.0);

// Wall contact spring stiffness (N/m) and contact-start offset (m): a one-sided penetration
// spring that engages once pos.x() drops below kContactStartX (not exactly x_wall=0.0 -- the
// real hull's contact pads sit forward of the point-mass's body origin by CLAUDE.md Sec.4.2's
// -0.10m lever arm plus the 0.035m pad radius, so physical contact begins before the idealized
// body-origin wall distance reaches zero; this test's point-mass simplification models that same
// "contact starts before x_wall" effect with a single scalar offset instead of the full pad
// geometry). Tuned stiff enough to hold position within the test's tolerance without a damping
// term.
constexpr double kWallStiffness = 4000.0;
constexpr double kContactStartX = 0.05;

class FakeWallPlant
{
public:
  FakeWallPlant()
  {
    for (int i = 0; i < 8; ++i) {
      cmd_handles_.emplace_back(kThrusterNames[i], "effort", &thr_cmd_[i]);
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
  }

  // Constant disturbance wrench in world frame (N), applied every step() call until reset to
  // zero -- used by the comparative PID-vs-SMC disturbance-rejection test.
  void setDisturbance(const Vec3 & disturbance_force) { disturbance_force_ = disturbance_force; }

  void step(double dt)
  {
    Vec8 u;
    for (int i = 0; i < 8; ++i) {
      u(i) = thr_cmd_[i];
    }
    static const Mat68 b =
      adhere_controller::buildThrusterAllocationMatrix(
      kModelParams.thruster_pos, kModelParams.thruster_dir);
    const Vec3 f = (b * u).head<3>();
    const Vec3 buoy(0.0, 0.0, kNetBuoyancyZ);
    const Vec3 drag =
      kDLin.cwiseProduct(vel_) + kDQuad.cwiseProduct(vel_.cwiseProduct(vel_.cwiseAbs()));

    // One-sided wall-contact spring: only pushes back (+X) once within the contact-start offset.
    double f_contact_x = 0.0;
    if (pos_.x() < kContactStartX) {
      f_contact_x = kWallStiffness * (kContactStartX - pos_.x());
    }
    const Vec3 contact(f_contact_x, 0.0, 0.0);

    const Vec3 accel = (f + buoy - drag + contact + disturbance_force_) / kMass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    ci.active_state.pos_w[0] = pos_.x();
    ci.active_state.pos_w[1] = pos_.y();
    ci.active_state.pos_w[2] = pos_.z();
    ci.active_state.vel_body[0] = vel_.x();
    ci.active_state.vel_body[1] = vel_.y();
    ci.active_state.vel_body[2] = vel_.z();

    // Synthesize 4 equal pad normal forces from the same penetration reaction (plant orientation
    // held at identity, so the load is always perfectly symmetric).
    const double per_pad = f_contact_x / 4.0;
    for (int i = 0; i < 4; ++i) {
      ci.active_state.pad_force[i] = per_pad;
    }
  }

  const Vec3 & position() const { return pos_; }

  void writeCommands() { sanitizeAndWriteThrusterCommands(ci, kModelParams, violations_); }

  CtrlInterfaces ci;

private:
  uint64_t violations_ = 0;
  std::array<double, 8> thr_cmd_{};

  std::deque<CommandInterface> cmd_handles_;
  std::deque<LoanedCommandInterface> loaned_cmd_;

  Vec3 pos_ = Vec3::Zero();
  Vec3 vel_ = Vec3::Zero();
  Vec3 disturbance_force_ = Vec3::Zero();
};

constexpr double kDt = 0.004;  // 250Hz inner loop

}  // namespace

TEST(AdhereWallStates, StateAdherePidDocksAndHoldsAllFourPadsWithPositiveKernMargin)
{
  // Start 1.0m from the wall, free-swimming.
  FakeWallPlant plant;
  plant.setPosition(1.0, 0.1, -2.1);

  adhere_controller::StateAdherePid state(plant.ci, kTestModelParams);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

  // 20s settling budget: approach + dock + force-hold convergence.
  for (int i = 0; i < 5000; ++i) {
    state.run(t0, period);
    plant.writeCommands();
    plant.step(kDt);
  }

  EXPECT_TRUE(state.isDocked());
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(plant.ci.active_state.pad_force[i], 0.0);
  }

  // Rhombic Kern margin check via the real C_kern matrix, using the actually-commanded u.
  const auto b = adhere_controller::buildThrusterAllocationMatrix(
    kModelParams.thruster_pos, kModelParams.thruster_dir);
  const auto spans = adhere_controller::padHalfSpans(kModelParams.pad_pos);
  const Mat48 c_kern = adhere_controller::buildKernConstraintMatrix(
    b, spans.first, spans.second, kModelParams.kern_eps_margin);
  Vec8 u;
  for (int i = 0; i < 8; ++i) {
    u(i) = plant.ci.thruster_command_interface_[i].get().get_value();
  }
  const Vec4 kern_result = c_kern * u;
  for (int i = 0; i < 4; ++i) {
    EXPECT_LT(kern_result(i), 0.0);  // C_kern*u <= 0 admissible region
  }

  // Y/Z setpoint held (latched at enter() to the starting 0.1/-2.1).
  EXPECT_NEAR(plant.position().y(), 0.1, 0.02);
  EXPECT_NEAR(plant.position().z(), -2.1, 0.02);
}

TEST(AdhereWallStates, StateAdhereSmcDocksAndHoldsAllFourPadsWithPositiveKernMargin)
{
  FakeWallPlant plant;
  plant.setPosition(1.0, 0.1, -2.1);

  adhere_controller::StateAdhereSmc state(plant.ci, kTestModelParams);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

  for (int i = 0; i < 5000; ++i) {
    state.run(t0, period);
    plant.writeCommands();
    plant.step(kDt);
  }

  EXPECT_TRUE(state.isDocked());
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(plant.ci.active_state.pad_force[i], 0.0);
  }

  const auto b = adhere_controller::buildThrusterAllocationMatrix(
    kModelParams.thruster_pos, kModelParams.thruster_dir);
  const auto spans = adhere_controller::padHalfSpans(kModelParams.pad_pos);
  const Mat48 c_kern = adhere_controller::buildKernConstraintMatrix(
    b, spans.first, spans.second, kModelParams.kern_eps_margin);
  Vec8 u;
  for (int i = 0; i < 8; ++i) {
    u(i) = plant.ci.thruster_command_interface_[i].get().get_value();
  }
  const Vec4 kern_result = c_kern * u;
  for (int i = 0; i < 4; ++i) {
    EXPECT_LT(kern_result(i), 0.0);
  }

  EXPECT_NEAR(plant.position().y(), 0.1, 0.02);
  EXPECT_NEAR(plant.position().z(), -2.1, 0.02);
}

// Comparative trade-off check (CLAUDE.md Phase 3.8 Stage 3.1): under an identical sustained +Y
// disturbance after both controllers have docked, StateAdhereSmc should recover to a tighter Y
// position bound than StateAdherePid, while exhibiting a measurably higher observed thrust slew
// rate (chattering indicator) -- the "SMC: better disturbance rejection, worse chattering"
// narrative, verified via real closed-loop dynamics rather than asserted as given.
namespace
{

struct DisturbanceTrial
{
  double max_abs_y_error = 0.0;
  double slew_rate_sum_sq = 0.0;
  int slew_samples = 0;
};

template<typename StateT>
DisturbanceTrial runDockThenDisturb(const ModelParams & params)
{
  FakeWallPlant plant;
  plant.setPosition(1.0, 0.0, -2.1);

  StateT state(plant.ci, params);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

  // Dock first (60s budget -- the force-regulation loop's small ki_force gain converges slowly).
  for (int i = 0; i < 15000; ++i) {
    state.run(t0, period);
    plant.writeCommands();
    plant.step(kDt);
  }
  EXPECT_TRUE(state.isDocked());

  DisturbanceTrial trial;
  Vec8 prev_u = Vec8::Zero();
  for (int i = 0; i < 8; ++i) {
    prev_u(i) = plant.ci.thruster_command_interface_[i].get().get_value();
  }

  // Sustained +Y disturbance for 2s, tracking max |y error| throughout (including the initial
  // excursion -- that IS the disturbance-rejection capability being measured) and thrust slew
  // rate only from kOnsetSkipTicks onward (excludes the single-tick onset transient every
  // controller incurs the instant a step disturbance appears, which is not "chattering" and
  // would otherwise swamp the RMS with one large, legitimate correction).
  constexpr int kOnsetSkipTicks = 50;  // 0.2s
  plant.setDisturbance(Vec3(0.0, 15.0, 0.0));
  for (int i = 0; i < 500; ++i) {
    state.run(t0, period);
    plant.writeCommands();
    plant.step(kDt);

    trial.max_abs_y_error = std::max(trial.max_abs_y_error, std::abs(plant.position().y()));

    Vec8 u;
    for (int j = 0; j < 8; ++j) {
      u(j) = plant.ci.thruster_command_interface_[j].get().get_value();
    }
    if (i >= kOnsetSkipTicks) {
      const double slew = (u - prev_u).norm() / kDt;
      trial.slew_rate_sum_sq += slew * slew;
      trial.slew_samples += 1;
    }
    prev_u = u;
  }

  return trial;
}

// SMC with a deliberately TIGHT boundary layer and high switching gain (CLAUDE.md Phase 3.8
// Stage 3.1's "ADHERE_SMC with tight boundary layer" phrasing) -- the production default
// adhere_smc.phi=0.05 is wide enough that this discrete-time (4ms), noise-free fake plant
// settles smoothly without crossing the switching surface every tick; tightening phi (and
// raising k_switch so the bang-bang term still dominates once saturated) reproduces the
// textbook discrete-time SMC chattering artifact: the controller overshoots the (now much
// narrower) switching band every control cycle, causing the commanded sign to flip tick to
// tick.
ModelParams makeTightBoundaryLayerSmcParams()
{
  ModelParams p = kTestModelParams;
  p.adhere_smc.phi = (adhere_controller::Vec6() << 0.002, 0.002, 0.002, 0.002, 0.002, 0.002)
    .finished();
  p.adhere_smc.k_switch =
    (adhere_controller::Vec6() << 120.0, 120.0, 120.0, 30.0, 30.0, 30.0).finished();
  return p;
}
const ModelParams kTightBoundaryLayerSmcParams = makeTightBoundaryLayerSmcParams();

}  // namespace

TEST(AdhereWallStates, StateAdhereSmcRejectsDisturbanceBetterButChattersMoreThanAdherePid)
{
  const DisturbanceTrial pid_trial =
    runDockThenDisturb<adhere_controller::StateAdherePid>(kTestModelParams);
  const DisturbanceTrial smc_trial =
    runDockThenDisturb<adhere_controller::StateAdhereSmc>(kTightBoundaryLayerSmcParams);

  const double pid_slew_rms = std::sqrt(pid_trial.slew_rate_sum_sq / pid_trial.slew_samples);
  const double smc_slew_rms = std::sqrt(smc_trial.slew_rate_sum_sq / smc_trial.slew_samples);

  EXPECT_LT(smc_trial.max_abs_y_error, pid_trial.max_abs_y_error);
  EXPECT_GT(smc_slew_rms, pid_slew_rms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
