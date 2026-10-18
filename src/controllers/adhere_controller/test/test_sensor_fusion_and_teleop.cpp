// Phase 3.5 verification (CLAUDE.md Stage 3): RovStateEstimator's DVL lever-arm compensation,
// closed-loop StateDepthHold/StatePidHold tracking driven entirely through sensor_fusion (never
// ground truth), and the Xbox LT/RT -> pitch/roll teleop mapping used by StateOpenLoop/
// StatePidHold. Reuses the same "fake CtrlInterfaces with LoanedStateInterface/
// LoanedCommandInterface backed by local doubles" pattern as test_fsm_states.cpp -- no
// ResourceManager/MuJoCo needed.
#include <array>
#include <cmath>
#include <deque>
#include <random>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/OutputSafety.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"
#include "adhere_controller/estimator/RovStateEstimator.h"
#include "adhere_controller/FSM/StateDepthHold.h"
#include "adhere_controller/FSM/StateOpenLoop.h"
#include "adhere_controller/FSM/StatePidHold.h"

namespace
{

using adhere_controller::ActiveState;
using adhere_controller::CtrlInterfaces;
using adhere_controller::Mat68;
using adhere_controller::ModelParams;
using adhere_controller::RovStateEstimator;
using adhere_controller::sanitizeAndWriteThrusterCommands;
using adhere_controller::Vec3;
using adhere_controller::Vec8;

const ModelParams kModelParams;

constexpr double kMass = 26.0;
constexpr double kNetBuoyancyZ = 1.962;
const Vec3 kDLin(30.0, 20.0, 25.0);
const Vec3 kDQuad(140.0, 50.0, 70.0);

// Noise std values mirror CLAUDE.md Phase 3.5 Stage 2.1 / adhere_mujoco_hardware.cpp's
// kGyroNoiseStd etc. defaults exactly, so this test exercises RovStateEstimator against the
// same noise magnitudes it is tuned for.
constexpr double kGyroNoiseStd = 0.005;
constexpr double kAccelNoiseStd = 0.02;
constexpr double kAttNoiseStd = 0.003;
constexpr double kDepthNoiseStd = 0.005;
constexpr double kDvlVelNoiseStd = 0.008;
constexpr double kDvlRangeNoiseStd = 0.003;
constexpr long long kDepthDecimation = 10;  // 500/10 = 50Hz
constexpr long long kDvlDecimation = 25;    // 500/25 = 20Hz
constexpr double kDtPhys = 0.002;           // 500Hz physics tick

double rmse(const std::vector<double> & errors)
{
  double sum_sq = 0.0;
  for (double e : errors) {
    sum_sq += e * e;
  }
  return std::sqrt(sum_sq / static_cast<double>(errors.size()));
}

}  // namespace

// --- Test 1: DVL lever-arm compensation (CLAUDE.md Stage 3.2) -------------------------------
// A body spinning at a constant, known angular velocity with ZERO translational velocity: the
// raw DVL reading at r_dvl=(0,0,-0.20) is dominated by the lever-arm term omega x r_dvl (here
// magnitude |omega|*0.20), which RovStateEstimator must subtract out so its CoM velocity
// estimate converges to the true zero velocity, not the raw sensor reading.
TEST(RovStateEstimatorTest, DvlLeverArmCompensationConvergesWithin15mmPerSec)
{
  const Vec3 r_dvl(0.0, 0.0, -0.20);
  const Vec3 omega_true(0.0, 0.3, 0.0);  // rad/s, pure pitch-rate spin
  const Vec3 v_com_true = Vec3::Zero();

  // Sanity check: the raw (noise-free) lever-arm offset is NOT negligible relative to the
  // <0.015 m/s convergence target, so a filter that failed to compensate for it would fail
  // this test by an obvious margin (|omega x r_dvl| = 0.3*0.20 = 0.06 m/s).
  const Vec3 raw_lever_offset = omega_true.cross(r_dvl);
  ASSERT_GT(raw_lever_offset.norm(), 0.03);

  std::mt19937 rng(42);
  std::normal_distribution<double> nd(0.0, 1.0);

  RovStateEstimator est;
  est.reset(Vec3::Zero(), v_com_true, adhere_controller::Quat(1.0, 0.0, 0.0, 0.0));

  std::vector<double> vel_errors;
  for (long long tick = 0; tick < 20000; ++tick) {  // 40s @ 500Hz
    const Vec3 gyro_meas = omega_true + Vec3(nd(rng), nd(rng), nd(rng)) * kGyroNoiseStd;
    const Vec3 accel_meas = Vec3(nd(rng), nd(rng), nd(rng)) * kAccelNoiseStd;  // zero true accel
    est.predict(gyro_meas, accel_meas, kDtPhys);
    est.correctAttitude(adhere_controller::Quat(1.0, 0.0, 0.0, 0.0));

    if (tick % kDvlDecimation == 0) {
      const Vec3 v_dvl_meas = v_com_true + omega_true.cross(r_dvl) +
        Vec3(nd(rng), nd(rng), nd(rng)) * kDvlVelNoiseStd;
      est.correctDvl(v_dvl_meas, 0.0 + nd(rng) * kDvlRangeNoiseStd, r_dvl);
    }

    if (tick >= 15000) {  // last 10s: steady-state RMSE window
      vel_errors.push_back((est.velocityBody() - v_com_true).norm());
    }
  }

  EXPECT_LT(rmse(vel_errors), 0.015);
}

namespace
{

// Translation-only "noisy plant" (orientation fixed at identity, omega=0 -- rotation/lever-arm
// correctness is already covered by the estimator-only test above). Generates the exact same
// 500Hz-noisy-IMU / 50Hz-sample-and-hold-depth / 20Hz-sample-and-hold-DVL pipeline
// AdhereMujocoHardware::read() + AdhereController::update() implement, and drives
// ctrl_interfaces_.active_state purely from RovStateEstimator -- ground truth is NEVER written
// into active_state in this fixture, matching the "sensor_fusion" isolation contract.
class NoisySensorFusionPlant
{
public:
  NoisySensorFusionPlant()
  : rng_(7)
  {
    for (int i = 0; i < 8; ++i) {
      cmd_handles_.emplace_back(kThrusterNames[i], "effort", &thr_cmd_[i]);
      loaned_cmd_.emplace_back(cmd_handles_.back());
      ci.thruster_command_interface_.emplace_back(loaned_cmd_.back());
    }
    estimator_.reset(Vec3::Zero(), Vec3::Zero(), adhere_controller::Quat(1.0, 0.0, 0.0, 0.0));
  }

  void setPosition(double x, double y, double z)
  {
    true_pos_ = Vec3(x, y, z);
    estimator_.reset(true_pos_, true_vel_, adhere_controller::Quat(1.0, 0.0, 0.0, 0.0));
    syncActiveState();
  }

  // Advances the true (ground-truth, private to this fixture) plant by one 250Hz inner period,
  // substepped as 2 x 500Hz physics ticks -- exactly AdhereController's default
  // inner_decimation_=2 -- generating fresh noisy sensor samples and running the estimator on
  // EVERY substep, using the thruster command most recently written by the FSMState under test.
  void stepInnerPeriod()
  {
    for (int sub = 0; sub < 2; ++sub) {
      Vec8 u;
      for (int i = 0; i < 8; ++i) {
        u(i) = thr_cmd_[i];
      }
      static const Mat68 b =
        adhere_controller::buildThrusterAllocationMatrix(kModelParams.thruster_pos, kModelParams.thruster_dir);
      const Vec3 f = (b * u).head<3>();
      const Vec3 buoy(0.0, 0.0, kNetBuoyancyZ);
      const Vec3 drag =
        kDLin.cwiseProduct(true_vel_) + kDQuad.cwiseProduct(true_vel_.cwiseProduct(true_vel_.cwiseAbs()));
      const Vec3 accel_true = (f + buoy - drag) / kMass;

      // --- Generate noisy sensors from the CURRENT true state (pre-integration), exactly
      // like AdhereMujocoHardware::read() reports state before mj_step advances it. ---
      const auto noise = [this](double std_dev) { return nd_(rng_) * std_dev; };
      const Vec3 gyro_meas(noise(kGyroNoiseStd), noise(kGyroNoiseStd), noise(kGyroNoiseStd));
      const Vec3 accel_meas = accel_true + Vec3(noise(kAccelNoiseStd), noise(kAccelNoiseStd), noise(kAccelNoiseStd));
      estimator_.predict(gyro_meas, accel_meas, kDtPhys);
      estimator_.correctAttitude(adhere_controller::Quat(1.0, 0.0, 0.0, 0.0));

      if (step_count_ % kDepthDecimation == 0) {
        depth_hold_ = true_pos_.z() + noise(kDepthNoiseStd);
      }
      estimator_.correctDepth(depth_hold_, kModelParams.sensor_mount.r_depth);

      if (step_count_ % kDvlDecimation == 0) {
        dvl_vel_hold_ = true_vel_ + Vec3(noise(kDvlVelNoiseStd), noise(kDvlVelNoiseStd), noise(kDvlVelNoiseStd));
      }
      estimator_.correctDvl(dvl_vel_hold_, true_pos_.x() + noise(kDvlRangeNoiseStd), kModelParams.sensor_mount.r_dvl);

      // --- Integrate the TRUE (ground-truth, private) state forward using the same force. ---
      true_vel_ += accel_true * kDtPhys;
      true_pos_ += true_vel_ * kDtPhys;
      ++step_count_;
    }

    syncActiveState();
  }

  const Vec3 & truePosition() const { return true_pos_; }

  // Phase 3.7: FSMState::run() now writes ci.pending_thruster_cmd instead of calling
  // set_value() directly -- mirrors AdhereController::update()'s centralized output safety
  // gate call site, so this fixture exercises the exact same production code path.
  void writeCommands() { sanitizeAndWriteThrusterCommands(ci, kModelParams, violations_); }

  CtrlInterfaces ci;

private:
  uint64_t violations_ = 0;
  static constexpr std::array<const char *, 8> kThrusterNames = {
    "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};

  // Populates ci.active_state EXCLUSIVELY from the estimator -- this is the only place in the
  // fixture that touches active_state, and it never reads true_pos_/true_vel_ directly.
  void syncActiveState()
  {
    const Vec3 est_pos = estimator_.positionWorld();
    const Vec3 est_vel = estimator_.velocityBody();
    const Vec3 est_rpy = estimator_.rpy();
    const Vec3 est_omega = estimator_.angularVelocityBody();
    const auto est_quat = estimator_.orientation();
    for (int i = 0; i < 3; ++i) {
      ci.active_state.pos_w[i] = est_pos(i);
      ci.active_state.rpy[i] = est_rpy(i);
      ci.active_state.vel_body[i] = est_vel(i);
      ci.active_state.omega_body[i] = est_omega(i);
    }
    for (int i = 0; i < 4; ++i) {
      ci.active_state.quat_w[i] = est_quat(i);
    }
  }

  std::array<double, 8> thr_cmd_{};
  std::deque<hardware_interface::CommandInterface> cmd_handles_;
  std::deque<hardware_interface::LoanedCommandInterface> loaned_cmd_;

  Vec3 true_pos_ = Vec3::Zero();
  Vec3 true_vel_ = Vec3::Zero();
  RovStateEstimator estimator_;

  std::mt19937 rng_;
  std::normal_distribution<double> nd_{0.0, 1.0};
  long long step_count_ = 0;
  double depth_hold_ = 0.0;
  Vec3 dvl_vel_hold_ = Vec3::Zero();
};

constexpr std::array<const char *, 8> NoisySensorFusionPlant::kThrusterNames;

constexpr double kInnerDt = 0.004;  // 250Hz inner loop

}  // namespace

// --- Test 2: closed-loop StateDepthHold on sensor_fusion only (CLAUDE.md Stage 3.2) ---------
TEST(SensorFusionClosedLoop, StateDepthHoldTracksWithin15mmUsingOnlyNoisySensors)
{
  NoisySensorFusionPlant plant;
  plant.setPosition(0.0, 0.0, -2.0);

  adhere_controller::StateDepthHold state(plant.ci, kModelParams);
  state.enter();

  plant.ci.control_inputs_.setpoint_command_type.store(1);  // MODE_DEPTH_ONLY
  plant.ci.control_inputs_.setpoint_target_pos_z.store(-3.00);
  plant.ci.control_inputs_.setpoint_seq.store(1);

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kInnerDt);

  for (int i = 0; i < 6000; ++i) {  // 24s settling budget @ 250Hz
    state.run(t0, period);
    plant.writeCommands();
    plant.stepInnerPeriod();
  }

  EXPECT_NEAR(plant.truePosition().z(), -3.00, 0.015);
}

// --- Test 3: closed-loop StatePidHold on sensor_fusion only ----------------------------------
TEST(SensorFusionClosedLoop, StatePidHoldLocksWithin15mmUsingOnlyNoisySensors)
{
  NoisySensorFusionPlant plant;
  plant.setPosition(0.5, 0.0, -2.0);

  adhere_controller::StatePidHold state(plant.ci, kModelParams);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kInnerDt);

  plant.ci.control_inputs_.setpoint_command_type.store(2);  // MODE_FULL_POSE
  plant.ci.control_inputs_.setpoint_target_pos_x.store(1.0);
  plant.ci.control_inputs_.setpoint_target_pos_y.store(0.0);
  plant.ci.control_inputs_.setpoint_target_pos_z.store(-2.0);
  plant.ci.control_inputs_.setpoint_target_quat_w.store(1.0);  // identity orientation (rpy=0)
  plant.ci.control_inputs_.setpoint_target_quat_x.store(0.0);
  plant.ci.control_inputs_.setpoint_target_quat_y.store(0.0);
  plant.ci.control_inputs_.setpoint_target_quat_z.store(0.0);
  plant.ci.control_inputs_.setpoint_seq.store(1);

  for (int i = 0; i < 6000; ++i) {  // 24s settling budget @ 250Hz
    state.run(t0, period);
    plant.writeCommands();
    plant.stepInnerPeriod();
  }

  EXPECT_NEAR(plant.truePosition().x(), 1.0, 0.015);
}

// --- Test 4: Xbox LT/RT -> pitch/roll teleop mapping (CLAUDE.md Stage 2.3/3.3) ---------------
// Mirrors xbox_teleop_node.cpp's applyTriggerDeadzone()/LB-RB-reverses-sign logic exactly
// (duplicated here since that file builds a standalone executable, not a library target).
namespace
{
double applyTriggerDeadzone(double raw, double deadzone)
{
  const double pressed = std::clamp((1.0 - raw) / 2.0, 0.0, 1.0);
  if (pressed <= deadzone) {
    return 0.0;
  }
  return std::min((pressed - deadzone) / (1.0 - deadzone), 1.0);
}
}  // namespace

TEST(XboxTriggerMapping, RestingTriggerProducesZero)
{
  EXPECT_DOUBLE_EQ(applyTriggerDeadzone(1.0, 0.08), 0.0);  // unpressed convention
}

TEST(XboxTriggerMapping, FullyPressedTriggerProducesOne)
{
  EXPECT_NEAR(applyTriggerDeadzone(-1.0, 0.08), 1.0, 1e-9);
}

TEST(XboxTriggerMapping, BumperReversesSign)
{
  const double lt_val = applyTriggerDeadzone(-1.0, 0.08);  // fully pressed -> 1.0
  const bool lb_pressed = true;
  const double pitch_rate = lb_pressed ? -lt_val : lt_val;
  EXPECT_NEAR(pitch_rate, -1.0, 1e-9);
}

// --- Test 5: StateOpenLoop / StatePidHold roll/pitch wrench & target-angle mapping -----------
TEST(RollPitchTeleopMapping, StateOpenLoopMapsRollAndPitchRateToTxTy)
{
  NoisySensorFusionPlant plant;
  adhere_controller::StateOpenLoop state(plant.ci, kModelParams);
  state.enter();

  plant.ci.control_inputs_.teleop_roll_rate.store(1.0);
  plant.ci.control_inputs_.teleop_pitch_rate.store(-1.0);
  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kInnerDt);
  state.run(t0, period);
  plant.writeCommands();

  static const Mat68 b =
    adhere_controller::buildThrusterAllocationMatrix(kModelParams.thruster_pos, kModelParams.thruster_dir);
  Vec8 u;
  for (int i = 0; i < 8; ++i) {
    u(i) = plant.ci.thruster_command_interface_[i].get().get_value();
  }
  const auto wrench = b * u;
  // Damped pseudo-inverse allocation is not exact (same pseudo-inverse used by the rest of the
  // test suite's 1e-3-tolerance assertions), so this checks reconstruction to 1e-3, not bitwise.
  EXPECT_NEAR(wrench(3), kModelParams.open_loop.max_wrench[3], 1e-3);   // Tx = +max*roll_rate
  EXPECT_NEAR(wrench(4), -kModelParams.open_loop.max_wrench[4], 1e-3);  // Ty = -max*pitch_rate_mag
}

TEST(RollPitchTeleopMapping, StatePidHoldIntegratesAndClampsTiltTargets)
{
  NoisySensorFusionPlant plant;
  plant.setPosition(0.0, 0.0, -2.0);
  adhere_controller::StatePidHold state(plant.ci, kModelParams);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kInnerDt);

  plant.ci.control_inputs_.teleop_active.store(true);
  plant.ci.control_inputs_.teleop_roll_rate.store(1.0);
  for (int i = 0; i < 5000; ++i) {  // long enough to hit the +-45deg clamp
    state.run(t0, period);
    plant.writeCommands();
    plant.stepInnerPeriod();
  }

  const double max_tilt_rad = kModelParams.pid_hold.max_tilt_deg * M_PI / 180.0;
  const Vec3 q_d_rpy = adhere_controller::rotMatToRPY(
    adhere_controller::quatToRotMat(state.targetPose().quat));
  EXPECT_NEAR(q_d_rpy(0), max_tilt_rad, 1e-3);  // roll target clamped at +max_tilt_deg
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
