#include "adhere_controller/FSM/StatePidHold.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

namespace
{

// Anti-windup clamps (CLAUDE.md Phase 3.2 Stage 2.2.4): not part of robot_control.yaml's
// pid_hold.* schema, kept as fixed design constants independent of the live kp/ki/kd gains.
constexpr double kIntegralPosMax = 180.0;  // N, anti-windup clamp on ki_pos[i]*integral_pos_body_
constexpr double kIntegralAttMax = 10.0;   // N*m, anti-windup clamp on ki_att[i]*integral_att_body_

SixDofPidController::Gains pidHoldGainsToSixDof(const ModelParams::PidHoldGains & g)
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

}  // namespace

StatePidHold::StatePidHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::PID_HOLD, "state_pid_hold", ctrl_interfaces, model_params),
  dynamics_(model_params),
  pid_(pidHoldGainsToSixDof(model_params.pid_hold), dynamics_, kIntegralPosMax, kIntegralAttMax)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StatePidHold::enter()
{
  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state.
  const auto & active = ctrl_interfaces_.active_state;
  p_d_world_ = Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  q_d_ = Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);

  pid_.resetIntegrators();
}

void StatePidHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;
  const auto & gains = model_params_.pid_hold;

  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state, which
  // AdhereController populates from either ground truth or RovStateEstimator depending on
  // estimation_mode -- this state has no way to tell which.
  const auto & active = ctrl_interfaces_.active_state;
  const Vec3 pos_world(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  const Quat q_est(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
  const Vec3 vel_body(active.vel_body[0], active.vel_body[1], active.vel_body[2]);
  const Vec3 omega_body(active.omega_body[0], active.omega_body[1], active.omega_body[2]);

  const RotMat r = quatToRotMat(q_est);

  // Discrete setpoint (CLAUDE.md Phase 3.6 Sec.2.1): edge-detected via setpoint_seq, same
  // pattern as StateDepthHold, now carrying the full 6-DOF pose as position + quaternion.
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
      // Reset roll/pitch to 0, keep yaw & position (round-trip through RPY to isolate yaw).
      const Vec3 cur_rpy = rotMatToRPY(quatToRotMat(q_d_));
      q_d_ = quatFromRPY(Vec3(0.0, 0.0, cur_rpy(2)));
    }
  }

  // Stick-driven setpoint move / release-to-lock (CLAUDE.md Phase 3.2 Stage 2.2.4, extended to
  // roll/pitch in Phase 3.5 via Xbox LT/RT, quaternion-integrated in Phase 3.6). v_d_body and
  // omega_cmd_body stay exactly zero when teleop is inactive (including the watchdog-forced-zero
  // case), so e_v_body/drag-feedforward below automatically target zero velocity and
  // p_d_world_/q_d_ are simply never advanced -- no separate "release" branch needed.
  Vec3 v_d_body = Vec3::Zero();
  Vec3 omega_cmd_body = Vec3::Zero();
  if (ci.teleop_active.load()) {
    v_d_body = Vec3(
      -gains.max_teleop_vel[0] * ci.teleop_surge_x.load(),
      -gains.max_teleop_vel[1] * ci.teleop_sway_y.load(),
      gains.max_teleop_vel[2] * ci.teleop_heave_z.load());
    omega_cmd_body = Vec3(
      gains.max_teleop_vel[3] * ci.teleop_roll_rate.load(),
      gains.max_teleop_vel[4] * ci.teleop_pitch_rate.load(),
      gains.max_teleop_vel[5] * ci.teleop_yaw_rate.load());

    p_d_world_ += r * v_d_body * dt;
    p_d_world_.x() = std::max(p_d_world_.x(), model_params_.workspace_x_min);
    p_d_world_.z() =
      saturation(p_d_world_.z(), model_params_.workspace_z_min, model_params_.workspace_z_max);

    // Quaternion kinematics integration (CLAUDE.md Phase 3.6 Sec.2.1): q_d <- normalize(q_d (x)
    // dq), dq = smallAngleQuat(omega_cmd_body*dt) -- singularity-free setpoint advancement.
    // Tilt clamp still goes through a cheap RPY round-trip (preserves the existing +-max_tilt_deg
    // contract exactly; yaw is never clamped so it passes through unaffected).
    q_d_ = normalizeQuat(quatMul(q_d_, smallAngleQuat(omega_cmd_body * dt)));
    const double max_tilt_rad = gains.max_tilt_deg * M_PI / 180.0;
    Vec3 q_d_rpy = rotMatToRPY(quatToRotMat(q_d_));
    q_d_rpy(0) = saturation(q_d_rpy(0), -max_tilt_rad, max_tilt_rad);
    q_d_rpy(1) = saturation(q_d_rpy(1), -max_tilt_rad, max_tilt_rad);
    q_d_ = quatFromRPY(q_d_rpy);
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

  const Vec6 wrench_des = pid_.computeWrench(in, dt);
  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.pending_thruster_cmd[i] = u(i);
  }
}

void StatePidHold::exit()
{
}

FSMStateName StatePidHold::checkChange()
{
  return checkFsmCommandChange();
}

PoseTarget StatePidHold::targetPose() const
{
  PoseTarget t;
  t.pos = p_d_world_;
  t.quat = q_d_;
  return t;
}

}  // namespace adhere_controller
