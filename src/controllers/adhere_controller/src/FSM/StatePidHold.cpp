#include "adhere_controller/FSM/StatePidHold.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/ThrusterAllocator.h"

namespace adhere_controller
{

namespace
{

// Anti-windup clamps (CLAUDE.md Phase 3.2 Stage 2.2.4): not part of robot_control.yaml's
// pid_hold.* schema, kept as fixed design constants independent of the live kp/ki/kd gains.
constexpr double kIntegralPosMax = 180.0;  // N, anti-windup clamp on ki_pos[i]*integral_pos_body_
constexpr double kIntegralAttMax = 10.0;   // N*m, anti-windup clamp on ki_att[i]*integral_att_body_

// Wraps an angle to (-pi, pi].
double wrapToPi(double angle) { return std::remainder(angle, 2.0 * M_PI); }

}  // namespace

StatePidHold::StatePidHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::PID_HOLD, "state_pid_hold", ctrl_interfaces, model_params)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
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
  const auto & gains = model_params_.pid_hold;

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
      -gains.max_teleop_vel[0] * ci.teleop_surge_x.load(),
      -gains.max_teleop_vel[1] * ci.teleop_sway_y.load(),
      gains.max_teleop_vel[2] * ci.teleop_heave_z.load());
    omega_zd = gains.max_teleop_vel[3] * ci.teleop_yaw_rate.load();

    p_d_world_ += r * v_d_body * dt;
    p_d_world_.x() = std::max(p_d_world_.x(), model_params_.workspace_x_min);
    p_d_world_.z() =
      saturation(p_d_world_.z(), model_params_.workspace_z_min, model_params_.workspace_z_max);
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
    integral_pos_body_(i) = saturation(
      integral_pos_body_(i), -kIntegralPosMax / gains.ki_pos(i), kIntegralPosMax / gains.ki_pos(i));
    integral_att_body_(i) = saturation(
      integral_att_body_(i), -kIntegralAttMax / gains.ki_att(i), kIntegralAttMax / gains.ki_att(i));
  }

  // Hydrostatic/drag dynamics feedforward (CLAUDE.md Phase 3.2 Stage 2.2.4), sourced from
  // ModelParams rather than depending on adhere_mujoco_hardware's FossenHydrodynamics (CLAUDE.md
  // Sec.3's package-decoupling rule). Default ModelParams values reproduce the same
  // ~+1.962N net buoyancy figure: rho*g*V - m_dry*g = 1000*9.81*0.0202 - 20.0*9.81
  // = 198.162 - 196.2 = +1.962N (gravity acts on the real dry mass, not the MJCF <inertial>
  // mass, which is dry mass + hydrodynamic added mass -- an inertial reaction effect, not real
  // weight).
  const double buoyancy_full = model_params_.rho_water * model_params_.gravity *
    model_params_.displaced_volume;
  const Vec3 f_buoy_net_world(0.0, 0.0, buoyancy_full - model_params_.vehicle_mass * model_params_.gravity);
  const Vec3 f_buoy_full_world(0.0, 0.0, buoyancy_full);
  const Vec3 f_buoy_ff_body = -(r.transpose() * f_buoy_net_world);
  const Vec3 tau_cob_ff_body =
    -model_params_.cob_offset_body.cross(r.transpose() * f_buoy_full_world);
  const Vec3 f_drag_ff_body = model_params_.D_lin.cwiseProduct(v_d_body) +
    model_params_.D_quad.cwiseProduct(v_d_body.cwiseProduct(v_d_body.cwiseAbs()));

  const Vec3 w_lin = gains.kp_pos.cwiseProduct(e_p_body) +
    gains.ki_pos.cwiseProduct(integral_pos_body_) + gains.kd_vel.cwiseProduct(e_v_body) +
    f_buoy_ff_body + f_drag_ff_body;
  const Vec3 w_ang = gains.kp_att.cwiseProduct(e_r) + gains.ki_att.cwiseProduct(integral_att_body_) +
    gains.kd_att.cwiseProduct(e_omega_body) + tau_cob_ff_body;

  Vec6 wrench_des;
  wrench_des << w_lin, w_ang;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), model_params_.thruster_min[i], model_params_.thruster_max[i]));
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

}  // namespace adhere_controller
