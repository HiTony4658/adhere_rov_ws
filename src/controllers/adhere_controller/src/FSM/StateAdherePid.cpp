#include "adhere_controller/FSM/StateAdherePid.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

namespace
{

// Anti-windup clamps for the Y/Z/attitude SixDofPidController half of this state (same role as
// StatePidHold.cpp's kIntegralPosMax/kIntegralAttMax -- not part of robot_control.yaml's
// adhere_pid.* schema, kept as fixed design constants).
constexpr double kIntegralPosMax = 180.0;  // N
constexpr double kIntegralAttMax = 10.0;   // N*m

SixDofPidController::Gains adherePidGainsToSixDof(const ModelParams::AdherePidGains & g)
{
  SixDofPidController::Gains out;
  out.kp_pos = g.kp_pos;
  out.ki_pos = g.ki_pos;
  out.kd_vel = g.kd_vel;
  out.kp_att = g.kp_att;
  out.ki_att = g.ki_att;
  out.kd_att = g.kd_att;
  return out;
}

AdherePidController::Gains adherePidGainsToForcePid(const ModelParams::AdherePidGains & g)
{
  AdherePidController::Gains out;
  out.kp_force = g.kp_force;
  out.ki_force = g.ki_force;
  out.kd_x = g.kd_x;
  out.f_preload = g.f_preload;
  out.integral_force_max = g.integral_force_max;
  return out;
}

}  // namespace

StateAdherePid::StateAdherePid(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::ADHERE_PID, "state_adhere_pid", ctrl_interfaces, model_params),
  dynamics_(model_params),
  pid_(
    adherePidGainsToSixDof(model_params.adhere_pid), dynamics_, kIntegralPosMax, kIntegralAttMax),
  force_pid_(adherePidGainsToForcePid(model_params.adhere_pid))
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateAdherePid::enter()
{
  const auto & active = ctrl_interfaces_.active_state;
  p_d_world_ = Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  q_d_ = Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);

  pid_.resetIntegrators();
  force_pid_.resetIntegrator();
  depth_aligned_ = false;
  docked_ = false;
}

void StateAdherePid::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;
  const auto & gains = model_params_.adhere_pid;

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

  // Stage 1 -> Stage 2 gate (CLAUDE.md Phase 4.2): DEPTH_ALIGN must stabilize depth/vertical
  // speed before WALL_ADHERE's APPROACH/ADHERED ramp-then-dock logic (below) is allowed to run.
  // Sticky, like docked_ below -- never un-aligns itself once reached.
  if (!depth_aligned_) {
    const double depth_err = pos_world.z() - (-stage.target_depth_m);
    const double vz_world = (r * vel_body).z();
    if (std::abs(depth_err) <= stage.depth_tol && std::abs(vz_world) <= stage.vz_tol) {
      depth_aligned_ = true;
    }
  }

  // APPROACH -> ADHERED gate (CLAUDE.md Phase 3.8 Stage 2.2.1): sticky -- never un-docks itself.
  // Gated on depth_aligned_ (Phase 4.2) so the sum-of-pad-forces/x_dock_eps check is never even
  // evaluated while the ROV is still away from the wall at the DEPTH_ALIGN standoff.
  if (depth_aligned_ && !docked_ && sum_f >= gains.dock_force_thresh &&
    pos_world.x() <= gains.x_wall + gains.x_dock_eps)
  {
    docked_ = true;
  }

  // Discrete setpoint (same edge-detection pattern as StatePidHold): Y/Z/attitude only matter
  // here, X is always re-latched below regardless of docked_.
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

  // Stick-driven Y/Z/attitude setpoint move (surge is ignored while docked_: sliding along the
  // wall is Y/Z + attitude only; while APPROACHing, surge input is also ignored -- the ramp below
  // owns X until docking).
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
    // Stage 1 DEPTH_ALIGN (CLAUDE.md Phase 4.2): hold Y/Z at the shared target and X at a safe
    // standoff distance from the wall -- not pressing in yet. Overrides whatever the
    // setpoint/teleop handling above just computed for Y/Z (X is untouched by that handling in
    // this state already).
    p_d_world_.y() = stage.target_y;
    p_d_world_.z() = -stage.target_depth_m;
    p_d_world_.x() = gains.x_wall + stage.standoff_x;
  } else if (!docked_) {
    // Stage 2 APPROACH: ramp p_d_world_.x() toward the wall; SixDofPidController drives all 6 DOF.
    const double step = gains.approach_speed * dt;
    p_d_world_.x() = std::max(p_d_world_.x() - step, gains.x_wall);
  } else {
    // Stage 2 ADHERED: re-latch X every tick so SixDofPidController's position loop contributes
    // ~0 on Fx, leaving the force PI law as the sole driver of that axis.
    p_d_world_.x() = pos_world.x();
  }

  SixDofPidController::Inputs in;
  in.p_d_world = p_d_world_;
  in.q_d = q_d_;
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = v_d_body;
  in.omega_cmd_body = omega_cmd_body;
  in.v_est_body = vel_body;
  in.omega_est_body = omega_body;

  Vec6 wrench = pid_.computeWrench(in, dt);
  if (docked_) {
    wrench(0) = force_pid_.computeNormalForceWrench(gains.f_des, sum_f, vel_body.x(), dt);
  }

  const Vec8 u = b_pinv_ * wrench;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.pending_thruster_cmd[i] = u(i);
  }
}

void StateAdherePid::exit()
{
}

FSMStateName StateAdherePid::checkChange()
{
  return checkFsmCommandChange();
}

PoseTarget StateAdherePid::targetPose() const
{
  PoseTarget t;
  t.pos = p_d_world_;
  t.quat = q_d_;
  return t;
}

}  // namespace adhere_controller
