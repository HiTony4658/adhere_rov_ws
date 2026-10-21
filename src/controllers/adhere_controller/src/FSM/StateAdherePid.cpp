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
: FSMState(FSMStateName::ADHERE_PID_WBC, "state_adhere_pid_wbc", ctrl_interfaces, model_params),
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
  standoff_homed_ = false;
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

  // This phase: live wall-target retarget bridge (replaces the static stage.target_y/
  // target_depth_m read everywhere below). If the target just moved by more than
  // stage.retarget_threshold, force a fresh STANDOFF_HOMING pass instead of dragging the ROV
  // sideways/vertically while still pressed against the wall.
  const double live_target_y = ctrl_interfaces_.adhere_target_.target_y.load();
  const double live_target_z = -ctrl_interfaces_.adhere_target_.target_depth_m.load();
  if (ctrl_interfaces_.adhere_target_.changed_pending.load()) {
    standoff_homed_ = false;
    docked_ = false;
    ctrl_interfaces_.adhere_target_.changed_pending.store(false);
  }

  // Stage 1 -> Stage 2 gate (CLAUDE.md Phase 4.2, extended this phase): STANDOFF_HOMING must
  // converge BOTH lateral Y and depth Z (plus planar speed) before SOFT_DOCKING/WALL_ADHERED's
  // ramp-then-dock logic (below) is allowed to run -- previously gated on depth alone, which let
  // wall preload engage before Y had converged, pinning the ROV at whatever Y it first touched
  // the wall at (friction then overpowers the Y-PID). Sticky, like docked_ below -- never
  // un-homes itself except via the retarget check above.
  if (!standoff_homed_) {
    const double depth_err = pos_world.z() - live_target_z;
    const double y_err = pos_world.y() - live_target_y;
    const Vec3 vel_world = r * vel_body;
    const double planar_speed = std::sqrt(vel_world.y() * vel_world.y() + vel_world.z() * vel_world.z());
    if (std::abs(depth_err) <= stage.depth_tol && std::abs(y_err) <= stage.y_tol &&
      planar_speed <= stage.vel_tol)
    {
      standoff_homed_ = true;
    }
  }

  // SOFT_DOCKING -> WALL_ADHERED gate (CLAUDE.md Phase 3.8 Stage 2.2.1): sticky -- never
  // un-docks itself. Gated on standoff_homed_ (Phase 4.2) so the sum-of-pad-forces/x_dock_eps
  // check is never even evaluated while the ROV is still away from the wall at the
  // STANDOFF_HOMING standoff.
  if (standoff_homed_ && !docked_ && sum_f >= gains.dock_force_thresh &&
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

  if (!standoff_homed_) {
    // Stage 1 STANDOFF_HOMING (CLAUDE.md Phase 4.2): hold Y/Z at the live wall target and X at a
    // safe standoff distance from the wall -- not pressing in yet. Overrides whatever the
    // setpoint/teleop handling above just computed for Y/Z (X is untouched by that handling in
    // this state already).
    p_d_world_.y() = live_target_y;
    p_d_world_.z() = live_target_z;
    p_d_world_.x() = gains.x_wall + stage.standoff_x;
  } else if (!docked_) {
    // Stage 2 SOFT_DOCKING: ramp p_d_world_.x() toward the wall; SixDofPidController drives all
    // 6 DOF.
    const double step = gains.approach_speed * dt;
    p_d_world_.x() = std::max(p_d_world_.x() - step, gains.x_wall);
  } else {
    // Stage 2 WALL_ADHERED: re-latch X every tick so SixDofPidController's position loop
    // contributes ~0 on Fx, leaving the force PI law as the sole driver of that axis.
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
  } else if (standoff_homed_) {
    // Phase 4.3 soft-landing: decelerate into the wall during the pre-dock APPROACH ramp
    // instead of impacting at a constant approach_speed (the Kelvin-Voigt pads' zeta~0.2 would
    // otherwise bounce the ROV off on first contact).
    wrench(0) -= stage.approach_vx_damp * std::max(0.0, vel_body.x());
  }

  // Phase 4.3 lever-arm moment feedforward (CLAUDE.md Sec.5.2's k_ff=0.1m): thr_T5..8 sit at
  // body X=-0.10m, so any Fy/Fz command induces a parasitic overturning moment this cancels.
  wrench(4) += 0.1 * wrench(2);
  wrench(5) -= 0.1 * wrench(1);

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
