#include "adhere_controller_common/FSM/StateDepthHold.h"

#include <cmath>

#include "adhere_controller_common/common/mathTools.h"
#include "adhere_controller_common/common/ThrusterAllocator.h"

namespace adhere_controller_common
{

namespace
{

// PID depth-hold gains (CLAUDE.md Phase 3.1 Stage 2.2/3.2 closed-loop tracking contract:
// <5mm steady-state error, minimal overshoot, converge within the 6s per-setpoint test
// window). Tuned against the plant's augmented mass (26 kg) and vertical Fossen damping
// (D_lin.z=25, D_quad.z=70 in fossen_hydrodynamics.hpp) with the max achievable allocated
// Fz (+-80N, see buildAllocationMatrix() below) comfortably inside actuator rails.
constexpr double kKpZ = 180.0;
constexpr double kKiZ = 95.0;
constexpr double kKdZ = 62.0;
constexpr double kIntegralMax = 180.0;  // N, anti-windup clamp on Ki*integral_z_

// Gentle attitude stabilization (angle-PD + rate damping) on roll/pitch/yaw. rov_wall.xml's
// tangential-thruster sites sit at body X=0.0, i.e. the SAME X coordinate as the CoM (rov_base's
// <inertial pos="0 0 0">) -- unlike the X=-0.10m lever-arm case CLAUDE.md Sec.5 describes for the
// full KAMR-MPPI wrench reconstruction, there is zero X-offset here, so tangential thrusters
// produce no parasitic pitch/yaw moment to compensate for; only the gentle restoring/damping
// terms below are needed.
constexpr double kKpAttitude = 6.0;
constexpr double kKdAttitude = 4.0;

constexpr double kThrusterMin = -40.0;
constexpr double kThrusterMax = 40.0;

// Xbox left-stick continuous heave nudge (CLAUDE.md Phase 3.2 Stage 2.2.2).
constexpr double kTeleopHeaveRate = 0.30;  // m/s, full-deflection target_z_ rate
constexpr double kTargetZMin = 0.10;       // m
constexpr double kTargetZMax = 0.95;       // m

}  // namespace

StateDepthHold::StateDepthHold(CtrlInterfaces & ctrl_interfaces)
: FSMState(FSMStateName::DEPTH_HOLD, "state_depth_hold", ctrl_interfaces)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(buildThrusterAllocationMatrix());
}

void StateDepthHold::enter()
{
  // Only reached via a transition INTO DEPTH_HOLD (checkChange() below never requests a
  // DEPTH_HOLD -> DEPTH_HOLD re-entry), so this is exactly the "entering fresh" case CLAUDE.md
  // Phase 3.1 Stage 2.2 says must reset the integrator; live target_z_ updates while already in
  // this state never call enter() again. Seeds target_z_ from the ROV's current world Z so the
  // hull does not lurch toward a stale leftover setpoint on entry.
  target_z_ = ctrl_interfaces_.odom_state_interface_[2].get().get_value();
  integral_z_ = 0.0;
}

void StateDepthHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  const double pos_z = ctrl_interfaces_.odom_state_interface_[2].get().get_value();
  const Quat quat(
    ctrl_interfaces_.odom_state_interface_[3].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[4].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[5].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[6].get().get_value());
  const Vec3 vel_body(
    ctrl_interfaces_.odom_state_interface_[7].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[8].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[9].get().get_value());
  const Vec3 omega_body(
    ctrl_interfaces_.imu_state_interface_[4].get().get_value(),
    ctrl_interfaces_.imu_state_interface_[5].get().get_value(),
    ctrl_interfaces_.imu_state_interface_[6].get().get_value());

  const RotMat r = quatToRotMat(quat);
  const Vec3 rpy = rotMatToRPY(r);
  const double vel_z_world = (r * vel_body)(2);

  // Discrete setpoint (CLAUDE.md Phase 3.2 Stage 2.1): edge-detected via setpoint_seq so the
  // same /rov/setpoint_cmd message is applied exactly once, never fighting the continuous
  // teleop nudge below on every intervening tick.
  const auto & ci = ctrl_interfaces_.control_inputs_;
  const uint64_t seq = ci.setpoint_seq.load();
  if (seq != last_seen_setpoint_seq_) {
    last_seen_setpoint_seq_ = seq;
    if (ci.setpoint_mode.load() == 2) {
      target_z_ = ci.setpoint_lock_current_pose.load() ? pos_z : ci.setpoint_target_z.load();
    }
  }

  // Xbox left-stick continuous heave nudge (CLAUDE.md Phase 3.2 Stage 2.2.2): integrates every
  // tick regardless of teleop_active (heave_z is already 0.0 when not deflected, including
  // after AdhereKamrController's 0.25s watchdog forces it to 0 on joystick disconnect), so the
  // stick returning to center naturally latches target_z_ at its current value with no special
  // "release" handling needed.
  target_z_ += kTeleopHeaveRate * ci.teleop_heave_z.load() * dt;
  target_z_ = saturation(target_z_, kTargetZMin, kTargetZMax);

  const double e_z = target_z_ - pos_z;

  // Clamped anti-windup integral, derivative-on-measurement (CLAUDE.md Phase 3.1 Stage 2.2):
  // Kd acts on measured world vertical velocity, not d(e_z)/dt, so a live target_z retarget
  // (a setpoint step) never produces a derivative-kick transient.
  integral_z_ = saturation(integral_z_ + e_z * dt, -kIntegralMax / kKiZ, kIntegralMax / kKiZ);
  const double f_z_world_des = kKpZ * e_z + kKiZ * integral_z_ - kKdZ * vel_z_world;

  // Rotate the world-frame vertical force command into the body frame, then superimpose gentle
  // PD attitude stabilization torques (angle -> 0, rate damping).
  const Vec3 f_world_des(0.0, 0.0, f_z_world_des);
  const Vec3 f_body_des = r.transpose() * f_world_des;
  const Vec3 tau_body_des = -kKpAttitude * rpy - kKdAttitude * omega_body;

  Vec6 wrench_des;
  wrench_des << f_body_des, tau_body_des;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), kThrusterMin, kThrusterMax));
  }
}

void StateDepthHold::exit()
{
}

FSMStateName StateDepthHold::checkChange()
{
  return checkFsmCommandChange();
}

std::array<double, 4> StateDepthHold::targetPose() const
{
  return {0.0, 0.0, target_z_, 0.0};
}

}  // namespace adhere_controller_common
