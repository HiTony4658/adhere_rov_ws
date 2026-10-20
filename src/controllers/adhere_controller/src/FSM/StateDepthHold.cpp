#include "adhere_controller/FSM/StateDepthHold.h"

#include <cmath>

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

namespace
{

// Same anti-windup clamp StateDepthHold.cpp used pre-Phase-3.7 (kIntegralMax=180N on the Z
// position integrator); the attitude integral clamp is unused here (depth_hold.ki_att is always
// 0 -- this state never had an attitude integral term) but SixDofPidController needs a non-zero
// divisor-safe value regardless.
constexpr double kIntegralPosMax = 180.0;  // N
constexpr double kIntegralAttMax = 10.0;   // N*m, unused (ki_att == 0)

SixDofPidController::Gains depthHoldGainsToSixDof(const ModelParams::DepthHoldGains & g)
{
  SixDofPidController::Gains out;
  out.kp_pos = Vec3(0.0, 0.0, g.kp_z);
  out.ki_pos = Vec3(0.0, 0.0, g.ki_z);
  out.kd_vel = Vec3(0.0, 0.0, g.kd_z);
  // Isotropic scalar attitude damping (CLAUDE.md Phase 3.1/3.2): the original law was
  // `-kp_att*rpy - kd_att*omega_body` with kp_att/kd_att plain scalars, equivalent at this
  // state's small-attitude operating point to SixDofPidController's `+kp_att*e_r` against an
  // identity attitude setpoint (quatError(q_est, identity) ~= -rpy to leading order).
  out.kp_att = Vec3::Constant(g.kp_att);
  out.ki_att = Vec3::Zero();
  out.kd_att = Vec3::Constant(g.kd_att);
  return out;
}

}  // namespace

StateDepthHold::StateDepthHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::HOLD_PID_DEPTH, "state_hold_pid_depth", ctrl_interfaces, model_params),
  dynamics_(model_params),
  pid_(
    depthHoldGainsToSixDof(model_params.depth_hold), dynamics_, kIntegralPosMax, kIntegralAttMax)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateDepthHold::enter()
{
  // Only reached via a transition INTO HOLD_PID_DEPTH (checkChange() below never requests a
  // HOLD_PID_DEPTH -> HOLD_PID_DEPTH re-entry), so this is exactly the "entering fresh" case CLAUDE.md
  // Phase 3.1 Stage 2.2 says must reset the integrator; live target_z_ updates while already in
  // this state never call enter() again. Seeds target_z_ from the ROV's current world Z so the
  // hull does not lurch toward a stale leftover setpoint on entry.
  target_z_ = ctrl_interfaces_.active_state.pos_w[2];
  pid_.resetIntegrators();
}

void StateDepthHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state, which
  // AdhereController populates from either ground truth or RovStateEstimator depending on
  // estimation_mode -- this state has no way to tell which.
  const auto & active = ctrl_interfaces_.active_state;
  const Vec3 pos_world(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  const Quat q_est(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
  const Vec3 vel_body(active.vel_body[0], active.vel_body[1], active.vel_body[2]);
  const Vec3 omega_body(active.omega_body[0], active.omega_body[1], active.omega_body[2]);
  last_actual_x_ = pos_world.x();
  last_actual_y_ = pos_world.y();

  // Discrete setpoint (CLAUDE.md Phase 3.2 Stage 2.1): edge-detected via setpoint_seq so the
  // same /rov/setpoint_cmd message is applied exactly once, never fighting the continuous
  // teleop nudge below on every intervening tick.
  const auto & ci = ctrl_interfaces_.control_inputs_;
  const uint64_t seq = ci.setpoint_seq.load();
  if (seq != last_seen_setpoint_seq_) {
    last_seen_setpoint_seq_ = seq;
    const uint8_t cmd_type = ci.setpoint_command_type.load();
    if (cmd_type == 1 /* MODE_DEPTH_ONLY */) {
      target_z_ = ci.setpoint_target_pos_z.load();
    } else if (cmd_type == 3 /* MODE_LOCK_CURRENT */) {
      target_z_ = pos_world.z();
    }
  }

  // Xbox left-stick continuous heave nudge (CLAUDE.md Phase 3.2 Stage 2.2.2): integrates every
  // tick regardless of teleop_active (heave_z is already 0.0 when not deflected, including
  // after AdhereController's 0.25s watchdog forces it to 0 on joystick disconnect), so the
  // stick returning to center naturally latches target_z_ at its current value with no special
  // "release" handling needed.
  const auto & gains = model_params_.depth_hold;
  target_z_ += gains.max_heave_vel * ci.teleop_heave_z.load() * dt;
  target_z_ = saturation(target_z_, model_params_.workspace_z_min, model_params_.workspace_z_max);

  // Reduced "depth-only" mode (CLAUDE.md Phase 3.7 Stage 1.3): lateral (X/Y) setpoint is
  // re-latched to the current estimate every tick (zero lateral authority), attitude setpoint
  // is identity (see depthHoldGainsToSixDof()'s comment for why that reproduces the pre-3.7
  // Euler damping law at this state's small-attitude operating point).
  SixDofPidController::Inputs in;
  in.p_d_world = Vec3(pos_world.x(), pos_world.y(), target_z_);
  in.q_d = Quat(1.0, 0.0, 0.0, 0.0);
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = Vec3::Zero();
  in.omega_cmd_body = Vec3::Zero();
  in.v_est_body = vel_body;
  in.omega_est_body = omega_body;

  const Vec6 wrench_des = pid_.computeWrench(in, dt);
  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.pending_thruster_cmd[i] = u(i);
  }
}

void StateDepthHold::exit()
{
}

FSMStateName StateDepthHold::checkChange()
{
  return checkFsmCommandChange();
}

PoseTarget StateDepthHold::targetPose() const
{
  PoseTarget t;
  t.pos = Vec3(last_actual_x_, last_actual_y_, target_z_);
  return t;
}

}  // namespace adhere_controller
