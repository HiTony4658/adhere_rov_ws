// Closed-loop FSM state contract (CLAUDE.md Phase 3.2 Stage 3.2). Builds a fake CtrlInterfaces
// out of hardware_interface::StateInterface/CommandInterface backed by local doubles (no
// ResourceManager needed -- handle.hpp's public (name, interface_name, double*) constructor is
// enough), then drives each state's run() against a simple translational point-mass "plant"
// owned by the test itself (R fixed at identity throughout, since none of these 3 contracts
// exercise yaw -- full coupled rotational dynamics are validated separately in the real MuJoCo
// closed-loop smoke test, not re-derived here).
#include <array>
#include <deque>
#include <string>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "adhere_controller_common/common/ThrusterAllocator.h"
#include "adhere_controller_common/CtrlInterfaces.h"
#include "adhere_controller_common/FSM/StateDepthHold.h"
#include "adhere_controller_common/FSM/StateOpenLoop.h"
#include "adhere_controller_common/FSM/StatePidHold.h"

namespace
{

using adhere_controller_common::CtrlInterfaces;
using adhere_controller_common::Mat68;
using adhere_controller_common::Vec3;
using adhere_controller_common::Vec8;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;

constexpr std::array<const char *, 8> kThrusterNames = {
  "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};

// Same plant constants StatePidHold.cpp's dynamics feedforward assumes (fossen_hydrodynamics.hpp
// defaults), so the controller's internal model and this test's "ground truth" plant agree.
constexpr double kMass = 26.0;
constexpr double kNetBuoyancyZ = 1.962;  // N, rho*g*V - m_dry*g
const Vec3 kDLin(30.0, 20.0, 25.0);
const Vec3 kDQuad(140.0, 50.0, 70.0);

// Fakes a CtrlInterfaces with stable-address backing storage (std::deque never relocates
// existing elements on push_back, unlike std::vector) and a simple 3-DOF translational
// point-mass plant (orientation held fixed at identity -- see file header).
class FakePlant
{
public:
  FakePlant()
  {
    // Command interfaces: thr_N1..thr_T8 / effort.
    for (int i = 0; i < 8; ++i) {
      cmd_handles_.emplace_back(kThrusterNames[i], "effort", &thr_cmd_[i]);
      loaned_cmd_.emplace_back(cmd_handles_.back());
      ci.thruster_command_interface_.emplace_back(loaned_cmd_.back());
    }
    // State interfaces: rov_odom/{pos.x,y,z, quat.w,x,y,z, vel_body.x,y,z} (indices 0..9,
    // matching adhere_kamr_controller.cpp's documented CtrlInterfaces layout).
    static constexpr std::array<const char *, 10> kOdomSuffixes = {
      "pos.x", "pos.y", "pos.z", "quat.w", "quat.x", "quat.y", "quat.z", "vel_body.x",
      "vel_body.y", "vel_body.z"};
    for (int i = 0; i < 10; ++i) {
      state_handles_.emplace_back("rov_odom", kOdomSuffixes[i], &odom_[i]);
      loaned_state_.emplace_back(state_handles_.back());
      ci.odom_state_interface_.emplace_back(loaned_state_.back());
    }
    // State interfaces: imu_sensor/{orientation.*, angular_velocity.*, linear_acceleration.*}
    // (indices 0..9); only angular_velocity (4..6) is read by any state under test, held at 0.
    static constexpr std::array<const char *, 10> kImuSuffixes = {
      "orientation.w", "orientation.x", "orientation.y", "orientation.z", "angular_velocity.x",
      "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x",
      "linear_acceleration.y", "linear_acceleration.z"};
    for (int i = 0; i < 10; ++i) {
      state_handles_.emplace_back("imu_sensor", kImuSuffixes[i], &imu_[i]);
      loaned_state_.emplace_back(state_handles_.back());
      ci.imu_state_interface_.emplace_back(loaned_state_.back());
    }

    odom_[3] = 1.0;  // quat.w = 1 (identity orientation, held fixed for the test's lifetime)
  }

  void setPosition(double x, double y, double z)
  {
    pos_ = Vec3(x, y, z);
    odom_[0] = x;
    odom_[1] = y;
    odom_[2] = z;
  }

  // Advances the point-mass plant by dt using the thruster commands the state under test just
  // wrote: forward-allocates them through the real B matrix (so the test exercises the full
  // B -> B^dagger round trip, not just the controller's internal model), adds the net buoyancy
  // bias and quadratic Fossen drag, and integrates.
  void step(double dt)
  {
    Vec8 u;
    for (int i = 0; i < 8; ++i) {
      u(i) = thr_cmd_[i];
    }
    static const Mat68 b = adhere_controller_common::buildThrusterAllocationMatrix();
    const Vec3 f = (b * u).head<3>();
    const Vec3 buoy(0.0, 0.0, kNetBuoyancyZ);
    const Vec3 drag =
      kDLin.cwiseProduct(vel_) + kDQuad.cwiseProduct(vel_.cwiseProduct(vel_.cwiseAbs()));
    const Vec3 accel = (f + buoy - drag) / kMass;
    vel_ += accel * dt;
    pos_ += vel_ * dt;

    odom_[0] = pos_.x();
    odom_[1] = pos_.y();
    odom_[2] = pos_.z();
    odom_[7] = vel_.x();
    odom_[8] = vel_.y();
    odom_[9] = vel_.z();
  }

  const Vec3 & position() const { return pos_; }

  CtrlInterfaces ci;

private:
  std::array<double, 8> thr_cmd_{};
  std::array<double, 10> odom_{};
  std::array<double, 10> imu_{};

  std::deque<CommandInterface> cmd_handles_;
  std::deque<StateInterface> state_handles_;
  std::deque<LoanedCommandInterface> loaned_cmd_;
  std::deque<LoanedStateInterface> loaned_state_;

  Vec3 pos_ = Vec3::Zero();
  Vec3 vel_ = Vec3::Zero();
};

constexpr double kDt = 0.004;  // 250Hz inner loop

}  // namespace

TEST(FsmStates, StateDepthHoldTracksSetpointAndTeleopNudgeWithin5mm)
{
  FakePlant plant;
  plant.setPosition(0.0, 0.0, 1.0);

  adhere_controller_common::StateDepthHold state(plant.ci);
  state.enter();

  // Discrete setpoint: target_z = 0.40m.
  plant.ci.control_inputs_.setpoint_mode.store(2);
  plant.ci.control_inputs_.setpoint_lock_current_pose.store(false);
  plant.ci.control_inputs_.setpoint_target_z.store(0.40);
  plant.ci.control_inputs_.setpoint_seq.store(1);

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);
  for (int i = 0; i < 4000; ++i) {  // 16s settling budget
    state.run(t0, period);
    plant.step(kDt);
  }
  EXPECT_NEAR(plant.position().z(), 0.40, 0.005);

  // Xbox left-stick heave nudge: push target_z_ up by ~0.30m over 1.0s, then release.
  plant.ci.control_inputs_.teleop_active.store(true);
  plant.ci.control_inputs_.teleop_heave_z.store(1.0);
  for (int i = 0; i < 250; ++i) {  // 1.0s at 250Hz
    state.run(t0, period);
    plant.step(kDt);
  }
  plant.ci.control_inputs_.teleop_active.store(false);
  plant.ci.control_inputs_.teleop_heave_z.store(0.0);
  const double target_after_nudge = state.targetPose()[2];
  EXPECT_NEAR(target_after_nudge, 0.70, 0.02);

  for (int i = 0; i < 4000; ++i) {  // let it settle at the new, now-frozen target
    state.run(t0, period);
    plant.step(kDt);
  }
  EXPECT_NEAR(plant.position().z(), target_after_nudge, 0.005);
  EXPECT_NEAR(state.targetPose()[2], target_after_nudge, 1e-9);  // stays latched, no drift
}

TEST(FsmStates, StateOpenLoopPureSwayProducesZeroParasiticMoment)
{
  FakePlant plant;
  adhere_controller_common::StateOpenLoop state(plant.ci);
  state.enter();

  plant.ci.control_inputs_.teleop_sway_y.store(1.0);
  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);
  state.run(t0, period);

  static const Mat68 b = adhere_controller_common::buildThrusterAllocationMatrix();
  Vec8 u;
  for (int i = 0; i < 8; ++i) {
    u(i) = plant.ci.thruster_command_interface_[i].get().get_value();
  }
  const auto wrench = b * u;
  EXPECT_NEAR(wrench(3), 0.0, 1e-6);  // Tx
  EXPECT_NEAR(wrench(4), 0.0, 1e-6);  // Ty
  EXPECT_NEAR(wrench(5), 0.0, 1e-6);  // Tz
  EXPECT_LT(wrench(1), -1e-3);        // sway_y=1.0 -> Fy = -50*1.0 (CLAUDE.md Phase 3.2 Sec 2.2.3)
}

TEST(FsmStates, StatePidHoldMovesThenLocksAtReleasePointWithin5mm)
{
  FakePlant plant;
  plant.setPosition(0.5, 0.0, 0.6);

  adhere_controller_common::StatePidHold state(plant.ci);
  state.enter();

  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(kDt);

  // Drive surge_x = -1.0 (away from the wall, +X body) for 3.0s.
  plant.ci.control_inputs_.teleop_active.store(true);
  plant.ci.control_inputs_.teleop_surge_x.store(-1.0);
  for (int i = 0; i < 750; ++i) {  // 3.0s at 250Hz
    state.run(t0, period);
    plant.step(kDt);
  }
  EXPECT_GT(state.targetPose()[0], 0.5 + 0.3);  // setpoint actually advanced

  // Release: sticks return to center.
  plant.ci.control_inputs_.teleop_active.store(false);
  plant.ci.control_inputs_.teleop_surge_x.store(0.0);
  const double x_d_at_release = state.targetPose()[0];

  for (int i = 0; i < 2000; ++i) {  // 8.0s settling budget
    state.run(t0, period);
    plant.step(kDt);
  }

  EXPECT_NEAR(plant.position().x(), x_d_at_release, 0.005);
  EXPECT_NEAR(state.targetPose()[0], x_d_at_release, 1e-9);  // frozen, no drift after release
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
