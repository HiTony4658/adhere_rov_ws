#include "adhere_controller/FSM/StateAdhereSmc.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

namespace
{

SixDofSmcController::Gains adhereSmcGainsToController(const ModelParams::AdhereSmcGains & g)
{
  SixDofSmcController::Gains out;
  out.lambda = g.lambda;
  out.kd = g.kd;
  out.k_switch = g.k_switch;
  out.phi = g.phi;
  out.wall_preload_fx = g.wall_preload_fx;
  return out;
}

}  // namespace

StateAdhereSmc::StateAdhereSmc(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::ADHERE_SMC, "state_adhere_smc", ctrl_interfaces, model_params),
  dynamics_(model_params),
  smc_(adhereSmcGainsToController(model_params.adhere_smc), dynamics_)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateAdhereSmc::enter()
{
  const auto & active = ctrl_interfaces_.active_state;
  p_d_world_ = Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  q_d_ = Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
  depth_aligned_ = false;
  docked_ = false;
}

void StateAdhereSmc::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;
  const auto & gains = model_params_.adhere_smc;

  const auto & active = ctrl_interfaces_.active_state;
  const Vec3 pos_world(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  const Quat q_est(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
  const Vec3 vel_body(active.vel_body[0], active.vel_body[1], active.vel_body[2]);
  const Vec3 omega_body(active.omega_body[0], active.omega_body[1], active.omega_body[2]);

  double sum_f = 0.0;
  for (double f : active.pad_force) {
    sum_f += f;
  }

  const RotMat r = quatToRotMat(q_est);
  const auto & stage = model_params_.adhere_stage;

  // Stage 1 -> Stage 2 gate (CLAUDE.md Phase 4.2): see StateAdherePid.cpp for the full
  // rationale -- identical sticky DEPTH_ALIGN -> WALL_ADHERE gate.
  if (!depth_aligned_) {
    const double depth_err = pos_world.z() - (-stage.target_depth_m);
    const double vz_world = (r * vel_body).z();
    if (std::abs(depth_err) <= stage.depth_tol && std::abs(vz_world) <= stage.vz_tol) {
      depth_aligned_ = true;
    }
  }

  if (depth_aligned_ && !docked_ && sum_f >= gains.dock_force_thresh &&
    pos_world.x() <= gains.x_wall + gains.x_dock_eps)
  {
    docked_ = true;
  }

  const uint64_t seq = ci.setpoint_seq.load();
  if (seq != last_seen_setpoint_seq_) {
    last_seen_setpoint_seq_ = seq;
    const uint8_t cmd_type = ci.setpoint_command_type.load();
    if (cmd_type == 2 /* MODE_FULL_POSE */) {
      p_d_world_ = Vec3(
        ci.setpoint_target_pos_x.load(), ci.setpoint_target_pos_y.load(),
        ci.setpoint_target_pos_z.load());
      q_d_ = normalizeQuat(Quat(
        ci.setpoint_target_quat_w.load(), ci.setpoint_target_quat_x.load(),
        ci.setpoint_target_quat_y.load(), ci.setpoint_target_quat_z.load()));
    } else if (cmd_type == 3 /* MODE_LOCK_CURRENT */) {
      p_d_world_ = pos_world;
      q_d_ = q_est;
    } else if (cmd_type == 4 /* MODE_LEVEL_ATTITUDE */) {
      const Vec3 cur_rpy = rotMatToRPY(quatToRotMat(q_d_));
      q_d_ = quatFromRPY(Vec3(0.0, 0.0, cur_rpy(2)));
    }
  }

  Vec3 v_d_body = Vec3::Zero();
  Vec3 omega_cmd_body = Vec3::Zero();
  if (ci.teleop_active.load()) {
    v_d_body = Vec3(
      0.0, -gains.max_teleop_vel[1] * ci.teleop_sway_y.load(),
      gains.max_teleop_vel[2] * ci.teleop_heave_z.load());
    omega_cmd_body = Vec3(
      gains.max_teleop_vel[3] * ci.teleop_roll_rate.load(),
      gains.max_teleop_vel[4] * ci.teleop_pitch_rate.load(),
      gains.max_teleop_vel[5] * ci.teleop_yaw_rate.load());

    p_d_world_ += r * v_d_body * dt;
    p_d_world_.z() =
      saturation(p_d_world_.z(), model_params_.workspace_z_min, model_params_.workspace_z_max);

    q_d_ = normalizeQuat(quatMul(q_d_, smallAngleQuat(omega_cmd_body * dt)));
    const double max_tilt_rad = gains.max_tilt_deg * M_PI / 180.0;
    Vec3 q_d_rpy = rotMatToRPY(quatToRotMat(q_d_));
    q_d_rpy(0) = saturation(q_d_rpy(0), -max_tilt_rad, max_tilt_rad);
    q_d_rpy(1) = saturation(q_d_rpy(1), -max_tilt_rad, max_tilt_rad);
    q_d_ = quatFromRPY(q_d_rpy);
  }

  if (!depth_aligned_) {
    p_d_world_.y() = stage.target_y;
    p_d_world_.z() = -stage.target_depth_m;
    p_d_world_.x() = gains.x_wall + stage.standoff_x;
  } else if (!docked_) {
    const double step = gains.approach_speed * dt;
    p_d_world_.x() = std::max(p_d_world_.x() - step, gains.x_wall);
  } else {
    p_d_world_.x() = pos_world.x();
  }

  SixDofSmcController::Inputs in;
  in.p_d_world = p_d_world_;
  in.q_d = q_d_;
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = v_d_body;
  in.omega_cmd_body = omega_cmd_body;
  in.v_est_body = vel_body;
  in.omega_est_body = omega_body;

  const Vec6 wrench = smc_.computeWrench(in, docked_);
  const Vec8 u = b_pinv_ * wrench;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.pending_thruster_cmd[i] = u(i);
  }
}

void StateAdhereSmc::exit()
{
}

FSMStateName StateAdhereSmc::checkChange()
{
  return checkFsmCommandChange();
}

PoseTarget StateAdhereSmc::targetPose() const
{
  PoseTarget t;
  t.pos = p_d_world_;
  t.quat = q_d_;
  return t;
}

}  // namespace adhere_controller
