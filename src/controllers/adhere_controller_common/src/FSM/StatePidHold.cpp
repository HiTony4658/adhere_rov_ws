#include "adhere_controller_common/FSM/StatePidHold.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller_common/common/mathTools.h"
#include "adhere_controller_common/common/ThrusterAllocator.h"

namespace adhere_controller_common
{

namespace
{

// Stick-driven setpoint-move velocities (CLAUDE.md Phase 3.2 Stage 2.2.4).
constexpr double kVxMax = 0.25;      // m/s, body -X (surge_x > 0 => toward the wall)
constexpr double kVyMax = 0.25;      // m/s, body -Y (sway_y > 0 => right when facing the wall)
constexpr double kVzMax = 0.25;      // m/s
constexpr double kOmegaZMax = 0.6;   // rad/s
constexpr double kXdMin = 0.12;      // m, never command the setpoint inside the wall
constexpr double kZdMin = 0.10;      // m
constexpr double kZdMax = 0.95;      // m

// PID gains (CLAUDE.md Phase 3.2 Stage 2.2.4; no literal numeric gains were specified in the
// task for this state, so these are new Phase 3.2-tuned constants analogous in magnitude to
// StateDepthHold's already-verified Z-axis gains (kKpZ=180/kKiZ=95/kKdZ=62) and to this plant's
// Fossen damping profile (D_lin=(30,20,25), D_quad=(140,50,70) -- fossen_hydrodynamics.hpp):
// X gets the gentlest gains (heaviest quadratic drag, D_quad.x=140), Z keeps StateDepthHold's
// exact gains (reused plant/response), Y sits between the two.
const Vec3 kKpPos(150.0, 165.0, 180.0);
const Vec3 kKiPos(60.0, 75.0, 95.0);
const Vec3 kKdPos(70.0, 65.0, 62.0);
constexpr double kIntegralPosMax = 180.0;  // N, anti-windup clamp on Ki_pos[i]*integral_pos_body_

const Vec3 kKpAtt(8.0, 8.0, 10.0);
const Vec3 kKiAtt(2.0, 2.0, 3.0);
const Vec3 kKdAtt(5.0, 5.0, 6.0);
constexpr double kIntegralAttMax = 10.0;  // N*m, anti-windup clamp on Ki_att[i]*integral_att_body_

constexpr double kThrusterMin = -40.0;
constexpr double kThrusterMax = 40.0;

// Hydrostatic/drag constants, hardcoded here rather than depending on adhere_mujoco_hardware's
// FossenHydrodynamics (CLAUDE.md Phase 3.2 Stage 2.2.4 -- adhere_controller_common must stay
// decoupled from adhere_mujoco_hardware, see CLAUDE.md Sec.3's package-decoupling rule). Values
// mirror fossen_hydrodynamics.hpp::HydroParams's defaults exactly, so the resulting net buoyancy
// reproduces the same ~+1.962N figure: rho*g*V - m_dry*g = 1000*9.81*0.0202 - 20.0*9.81
// = 198.162 - 196.2 = +1.962N (gravity acts on the real 20kg dry mass, not the 26kg MJCF
// <inertial> mass, which is dry mass + hydrodynamic added mass -- an inertial reaction effect,
// not real weight).
constexpr double kRhoWater = 1000.0;
constexpr double kGravity = 9.81;
constexpr double kVehicleMassDry = 20.0;
constexpr double kDisplacedVolume = (1.01 * 20.0) / 1000.0;
const Vec3 kCobOffsetBody(0.0, 0.0, 0.05);
const Vec3 kDLin(30.0, 20.0, 25.0);
const Vec3 kDQuad(140.0, 50.0, 70.0);

// Wraps an angle to (-pi, pi].
double wrapToPi(double angle) { return std::remainder(angle, 2.0 * M_PI); }

}  // namespace

StatePidHold::StatePidHold(CtrlInterfaces & ctrl_interfaces)
: FSMState(FSMStateName::PID_HOLD, "state_pid_hold", ctrl_interfaces)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(buildThrusterAllocationMatrix());
}

void StatePidHold::enter()
{
  p_d_world_ = Vec3(
    ctrl_interfaces_.odom_state_interface_[0].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[1].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[2].get().get_value());
  const Quat quat(
    ctrl_interfaces_.odom_state_interface_[3].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[4].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[5].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[6].get().get_value());
  psi_d_ = rotMatToRPY(quatToRotMat(quat))(2);

  integral_pos_body_.setZero();
  integral_att_body_.setZero();
}

void StatePidHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;

  const Vec3 pos_world(
    ctrl_interfaces_.odom_state_interface_[0].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[1].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[2].get().get_value());
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
  const double psi = rpy(2);

  // Discrete setpoint (CLAUDE.md Phase 3.2 Stage 2.1): edge-detected via setpoint_seq, same
  // pattern as StateDepthHold.
  const uint64_t seq = ci.setpoint_seq.load();
  if (seq != last_seen_setpoint_seq_) {
    last_seen_setpoint_seq_ = seq;
    if (ci.setpoint_mode.load() == 4) {
      if (ci.setpoint_lock_current_pose.load()) {
        p_d_world_ = pos_world;
        psi_d_ = psi;
      } else {
        p_d_world_ = Vec3(
          ci.setpoint_target_x.load(), ci.setpoint_target_y.load(), ci.setpoint_target_z.load());
        psi_d_ = ci.setpoint_target_yaw.load();
      }
    }
  }

  // Stick-driven setpoint move / release-to-lock (CLAUDE.md Phase 3.2 Stage 2.2.4). v_d_body
  // and omega_zd stay exactly zero when teleop is inactive (including the watchdog-forced-zero
  // case), so e_v_body/drag-feedforward below automatically target zero velocity and p_d_world_/
  // psi_d_ are simply never advanced -- no separate "release" branch needed.
  Vec3 v_d_body = Vec3::Zero();
  double omega_zd = 0.0;
  if (ci.teleop_active.load()) {
    v_d_body = Vec3(
      -kVxMax * ci.teleop_surge_x.load(), -kVyMax * ci.teleop_sway_y.load(),
      kVzMax * ci.teleop_heave_z.load());
    omega_zd = kOmegaZMax * ci.teleop_yaw_rate.load();

    p_d_world_ += r * v_d_body * dt;
    p_d_world_.x() = std::max(p_d_world_.x(), kXdMin);
    p_d_world_.z() = saturation(p_d_world_.z(), kZdMin, kZdMax);
    psi_d_ = wrapToPi(psi_d_ + omega_zd * dt);
  }

  // Errors in the body frame.
  const Vec3 e_p_body = r.transpose() * (p_d_world_ - pos_world);
  const Vec3 e_v_body = v_d_body - vel_body;
  const Vec3 e_r(-rpy(0), -rpy(1), wrapToPi(psi_d_ - psi));
  const Vec3 e_omega_body = Vec3(0.0, 0.0, omega_zd) - omega_body;

  integral_pos_body_ += e_p_body * dt;
  integral_att_body_ += e_r * dt;
  for (int i = 0; i < 3; ++i) {
    integral_pos_body_(i) =
      saturation(integral_pos_body_(i), -kIntegralPosMax / kKiPos(i), kIntegralPosMax / kKiPos(i));
    integral_att_body_(i) =
      saturation(integral_att_body_(i), -kIntegralAttMax / kKiAtt(i), kIntegralAttMax / kKiAtt(i));
  }

  // Hydrostatic/drag dynamics feedforward (CLAUDE.md Phase 3.2 Stage 2.2.4).
  const double buoyancy_full = kRhoWater * kGravity * kDisplacedVolume;
  const Vec3 f_buoy_net_world(0.0, 0.0, buoyancy_full - kVehicleMassDry * kGravity);
  const Vec3 f_buoy_full_world(0.0, 0.0, buoyancy_full);
  const Vec3 f_buoy_ff_body = -(r.transpose() * f_buoy_net_world);
  const Vec3 tau_cob_ff_body = -kCobOffsetBody.cross(r.transpose() * f_buoy_full_world);
  const Vec3 f_drag_ff_body =
    kDLin.cwiseProduct(v_d_body) + kDQuad.cwiseProduct(v_d_body.cwiseProduct(v_d_body.cwiseAbs()));

  const Vec3 w_lin = kKpPos.cwiseProduct(e_p_body) + kKiPos.cwiseProduct(integral_pos_body_) +
    kKdPos.cwiseProduct(e_v_body) + f_buoy_ff_body + f_drag_ff_body;
  const Vec3 w_ang = kKpAtt.cwiseProduct(e_r) + kKiAtt.cwiseProduct(integral_att_body_) +
    kKdAtt.cwiseProduct(e_omega_body) + tau_cob_ff_body;

  Vec6 wrench_des;
  wrench_des << w_lin, w_ang;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), kThrusterMin, kThrusterMax));
  }
}

void StatePidHold::exit()
{
}

FSMStateName StatePidHold::checkChange()
{
  return checkFsmCommandChange();
}

std::array<double, 4> StatePidHold::targetPose() const
{
  return {p_d_world_.x(), p_d_world_.y(), p_d_world_.z(), psi_d_};
}

}  // namespace adhere_controller_common
