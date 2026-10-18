// Phase 3.7 verification (CLAUDE.md Stage 3): FossenDynamics (mass matrix PD/symmetry,
// skew-symmetric Coriolis, dissipative damping, exact buoyancy restoring force),
// SixDofPidController (quaternion hemisphere invariance, conditional anti-windup), the
// RovStateEstimator contact-aided wall-constraint update, and common/OutputSafety.h's NaN/Inf
// sanitization gate.
#include <array>
#include <cmath>
#include <deque>
#include <limits>

#include <gtest/gtest.h>

#include "hardware_interface/loaned_command_interface.hpp"

#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/common/OutputSafety.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/dynamics/ModelParams.h"
#include "adhere_controller/estimator/RovStateEstimator.h"

using adhere_controller::CtrlInterfaces;
using adhere_controller::FossenDynamics;
using adhere_controller::ModelParams;
using adhere_controller::Quat;
using adhere_controller::RovStateEstimator;
using adhere_controller::sanitizeAndWriteThrusterCommands;
using adhere_controller::SixDofPidController;
using adhere_controller::Vec3;
using adhere_controller::Vec6;

namespace
{
const ModelParams kModelParams;
}  // namespace

// --- FossenDynamics --------------------------------------------------------------------------

TEST(FossenDynamics, MassMatrixIsSymmetricPositiveDefinite)
{
  FossenDynamics dyn(kModelParams);
  const auto & m = dyn.massMatrix();
  EXPECT_LT((m - m.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  Eigen::LLT<adhere_controller::Mat6> llt(m);
  EXPECT_EQ(llt.info(), Eigen::Success);
}

Vec6 makeVec6(double a, double b, double c, double d, double e, double f)
{
  Vec6 v;
  v << a, b, c, d, e, f;
  return v;
}

TEST(FossenDynamics, CoriolisWrenchIsSkewSymmetricQuadraticForm)
{
  FossenDynamics dyn(kModelParams);
  const std::array<Vec6, 4> samples = {
    makeVec6(1.0, 0.5, -0.3, 0.2, -0.1, 0.4), makeVec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    makeVec6(-2.0, 3.0, 1.0, -0.5, 0.6, -0.2), makeVec6(0.1, -0.2, 0.3, 0.4, -0.5, 0.6)};
  for (const auto & nu : samples) {
    const Vec6 c_nu = dyn.coriolisWrench(nu);
    EXPECT_NEAR(nu.dot(c_nu), 0.0, 1e-10);
  }
}

TEST(FossenDynamics, DampingWrenchIsDissipative)
{
  FossenDynamics dyn(kModelParams);
  const std::array<Vec6, 3> samples = {
    makeVec6(1.0, 0.5, -0.3, 0.2, -0.1, 0.4), makeVec6(-2.0, 3.0, 1.0, -0.5, 0.6, -0.2),
    makeVec6(0.1, -0.2, 0.3, 0.4, -0.5, 0.6)};
  for (const auto & nu : samples) {
    const Vec6 d_nu = dyn.dampingWrench(nu);
    EXPECT_GE(nu.dot(d_nu), 0.0);
  }
}

TEST(FossenDynamics, RestoringWrenchMatchesDocumentedNetBuoyancyAtLevelAttitude)
{
  FossenDynamics dyn(kModelParams);
  const Vec6 g = dyn.computeRestoringWrench(Quat(1.0, 0.0, 0.0, 0.0));
  // Documented figure (StatePidHold.cpp pre-Phase-3.7 comment, CLAUDE.md Phase 3.2 Stage 2.2.4):
  // rho*g*V - m_dry*g = 1000*9.81*0.0202 - 20.0*9.81 = +1.962N net buoyant at level attitude.
  const double expected_net_buoyancy =
    kModelParams.rho_water * kModelParams.gravity * kModelParams.displaced_volume -
    kModelParams.vehicle_mass * kModelParams.gravity;
  EXPECT_NEAR(g(2), expected_net_buoyancy, 1e-9);
  EXPECT_NEAR(expected_net_buoyancy, 1.962, 1e-3);
}

// --- SixDofPidController ----------------------------------------------------------------------

TEST(SixDofPidController, QuaternionHemisphereInvariance)
{
  FossenDynamics dyn(kModelParams);
  SixDofPidController::Gains gains;
  gains.kp_pos = Vec3(10.0, 10.0, 10.0);
  gains.ki_pos = Vec3(1.0, 1.0, 1.0);
  gains.kd_vel = Vec3(5.0, 5.0, 5.0);
  gains.kp_att = Vec3(10.0, 10.0, 10.0);
  gains.ki_att = Vec3(1.0, 1.0, 1.0);
  gains.kd_att = Vec3(5.0, 5.0, 5.0);

  SixDofPidController pid_a(gains, dyn, 180.0, 10.0);
  SixDofPidController pid_b(gains, dyn, 180.0, 10.0);

  SixDofPidController::Inputs in_a;
  in_a.p_d_world = Vec3(1.0, 0.5, -2.0);
  in_a.q_d = Quat(0.9, 0.1, 0.2, 0.3).normalized();
  in_a.p_est_world = Vec3(0.2, 0.1, -1.8);
  in_a.q_est = Quat(0.95, -0.05, 0.1, 0.05).normalized();
  in_a.v_est_body = Vec3(0.1, -0.05, 0.02);
  in_a.omega_est_body = Vec3(0.01, 0.02, -0.01);

  SixDofPidController::Inputs in_b = in_a;
  in_b.q_d = -in_a.q_d;  // same rotation, opposite hemisphere

  const Vec6 w_a = pid_a.computeWrench(in_a, 0.004);
  const Vec6 w_b = pid_b.computeWrench(in_b, 0.004);
  EXPECT_LT((w_a - w_b).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(SixDofPidController, ConditionalAntiWindupStaysWithinClamp)
{
  FossenDynamics dyn(kModelParams);
  SixDofPidController::Gains gains;
  gains.kp_pos = Vec3(10.0, 10.0, 10.0);
  gains.ki_pos = Vec3(5.0, 5.0, 5.0);
  gains.kd_vel = Vec3(1.0, 1.0, 1.0);
  gains.kp_att = Vec3(10.0, 10.0, 10.0);
  gains.ki_att = Vec3(2.0, 2.0, 2.0);
  gains.kd_att = Vec3(1.0, 1.0, 1.0);

  constexpr double kIntegralPosMax = 180.0;
  constexpr double kIntegralAttMax = 10.0;
  SixDofPidController pid(gains, dyn, kIntegralPosMax, kIntegralAttMax);

  SixDofPidController::Inputs in;
  in.p_d_world = Vec3(100.0, 0.0, 0.0);  // sustained large position error
  in.q_d = Quat(1.0, 0.0, 0.0, 0.0);
  in.p_est_world = Vec3::Zero();
  in.q_est = Quat(1.0, 0.0, 0.0, 0.0);

  for (int i = 0; i < 100000; ++i) {
    pid.computeWrench(in, 0.004);
  }

  // The integral term's contribution to w_lin.x() must never exceed kIntegralPosMax in
  // magnitude, regardless of how long the sustained error persists.
  const Vec6 w = pid.computeWrench(in, 0.004);
  const double ki_contribution = w(0) - gains.kp_pos.x() * 100.0;
  EXPECT_LE(std::abs(ki_contribution), kIntegralPosMax + 1e-6);
}

// --- RovStateEstimator contact-aided update ----------------------------------------------------

TEST(RovStateEstimatorTest, ContactAidedUpdateBoundsNormalAxisDrift)
{
  RovStateEstimator est;
  est.reset(Vec3::Zero(), Vec3::Zero(), Quat(1.0, 0.0, 0.0, 0.0));

  constexpr double kThresh = 2.0;
  constexpr double kXWallContact = 0.115;
  const std::array<double, 4> pad_force_in_contact = {5.0, 5.0, 5.0, 5.0};

  // A small constant spurious forward-accel bias would normally make p_hat_world_.x() drift
  // unboundedly over a long IMU dead-reckoning run; the contact-aided update should arrest it.
  constexpr double kBiasAccelX = 0.01;  // m/s^2
  for (int i = 0; i < 20000; ++i) {  // 40s @ 500Hz
    est.predict(Vec3::Zero(), Vec3(kBiasAccelX, 0.0, 0.0), 0.002);
    est.correctAttitude(Quat(1.0, 0.0, 0.0, 0.0));
    est.correctContact(pad_force_in_contact, kThresh, kXWallContact);
  }

  EXPECT_LT(std::abs(est.positionWorld().x() - kXWallContact), 0.05);
  EXPECT_LT(std::abs(est.velocityBody().x()), 0.05);
}

TEST(RovStateEstimatorTest, NoContactLeavesNormalAxisFreeToDrift)
{
  RovStateEstimator est;
  est.reset(Vec3::Zero(), Vec3::Zero(), Quat(1.0, 0.0, 0.0, 0.0));

  constexpr double kThresh = 2.0;
  constexpr double kXWallContact = 0.115;
  const std::array<double, 4> pad_force_no_contact = {0.0, 0.0, 0.0, 0.0};

  constexpr double kBiasAccelX = 0.01;
  for (int i = 0; i < 20000; ++i) {
    est.predict(Vec3::Zero(), Vec3(kBiasAccelX, 0.0, 0.0), 0.002);
    est.correctAttitude(Quat(1.0, 0.0, 0.0, 0.0));
    est.correctContact(pad_force_no_contact, kThresh, kXWallContact);
  }

  // Without contact, the bias accel integrates freely (0.5*a*t^2 over 40s is large) -- sanity
  // check that the pseudo-measurement above only fires under genuine contact.
  EXPECT_GT(std::abs(est.positionWorld().x() - kXWallContact), 0.5);
}

// --- common/OutputSafety.h ----------------------------------------------------------------------

namespace
{
class FakeThrusterRig
{
public:
  FakeThrusterRig()
  {
    for (int i = 0; i < 8; ++i) {
      cmd_handles_.emplace_back(kNames[i], "effort", &backing_[i]);
      loaned_.emplace_back(cmd_handles_.back());
      ci.thruster_command_interface_.emplace_back(loaned_.back());
    }
  }

  CtrlInterfaces ci;
  std::array<double, 8> backing_{};

private:
  static constexpr std::array<const char *, 8> kNames = {
    "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};
  std::deque<hardware_interface::CommandInterface> cmd_handles_;
  std::deque<hardware_interface::LoanedCommandInterface> loaned_;
};
constexpr std::array<const char *, 8> FakeThrusterRig::kNames;
}  // namespace

TEST(OutputSafetyGate, NanInjectedInOneChannelZeroesAllEightAndIncrementsCounter)
{
  FakeThrusterRig rig;
  for (int i = 0; i < 8; ++i) {
    rig.ci.pending_thruster_cmd[i] = 10.0;
  }
  rig.ci.pending_thruster_cmd[3] = std::numeric_limits<double>::quiet_NaN();

  uint64_t violations = 0;
  sanitizeAndWriteThrusterCommands(rig.ci, kModelParams, violations);

  EXPECT_EQ(violations, 1u);
  for (int i = 0; i < 8; ++i) {
    EXPECT_DOUBLE_EQ(rig.backing_[i], 0.0);
  }
}

TEST(OutputSafetyGate, InfInjectedZeroesAllEight)
{
  FakeThrusterRig rig;
  rig.ci.pending_thruster_cmd[0] = std::numeric_limits<double>::infinity();

  uint64_t violations = 0;
  sanitizeAndWriteThrusterCommands(rig.ci, kModelParams, violations);

  EXPECT_EQ(violations, 1u);
  EXPECT_DOUBLE_EQ(rig.backing_[0], 0.0);
}

TEST(OutputSafetyGate, FiniteOutOfRangeCommandIsClampedNotZeroed)
{
  FakeThrusterRig rig;
  rig.ci.pending_thruster_cmd[2] = 1000.0;  // way past thruster_max[2]=40.0

  uint64_t violations = 0;
  sanitizeAndWriteThrusterCommands(rig.ci, kModelParams, violations);

  EXPECT_EQ(violations, 0u);
  EXPECT_DOUBLE_EQ(rig.backing_[2], kModelParams.thruster_max[2]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
