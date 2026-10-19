#include "adhere_controller/FSM/StateBaselineMppi.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

StateBaselineMppi::StateBaselineMppi(
  CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params,
  baseline_mppi::BaselineMppiController & planner)
: FSMState(FSMStateName::BASELINE_MPPI, "state_baseline_mppi", ctrl_interfaces, model_params),
  planner_(planner)
{
}

void StateBaselineMppi::enter()
{
  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state.
  const auto & active = ctrl_interfaces_.active_state;
  p_d_world_ = Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  q_d_ = Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);

  // Resets the async worker's persistent nominal sequence to the hydrostatic hover wrench
  // (CLAUDE.md Phase 4.1 Sec.3.1) and invalidates any stale ZOH output from a previous activation
  // so run() never writes a leftover command from before this enter().
  planner_.warmStart(p_d_world_, q_d_);
  ctrl_interfaces_.outer_loop_cmd_.valid.store(false);

  // Safe-default contract (CLAUDE.md Phase 3.1 Stage 3.1, same as StatePassive::enter()): zeros
  // out any stale command left behind by the previous FSM state so the thrusters command 0 for
  // the handful of ticks between entering this state and the worker's first valid solve above.
  ctrl_interfaces_.pending_thruster_cmd.fill(0.0);
}

void StateBaselineMppi::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;
  // Reuses PID_HOLD's teleop-shaping/tilt-clamp gains (CLAUDE.md Phase 4.1 Sec.3.2): this state
  // only decides WHAT pose to track (identical setpoint/teleop grammar to StatePidHold), the CUDA
  // planner decides HOW -- there is no baseline_mppi-specific teleop shaping to configure.
  const auto & gains = model_params_.pid_hold;

  const auto & active = ctrl_interfaces_.active_state;
  const Vec3 pos_world(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
  const Quat q_est(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
  const RotMat r = quatToRotMat(q_est);

  // Discrete setpoint (same MODE_* grammar as StatePidHold/StateAdherePid/StateAdhereSmc).
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

  // Stick-driven setpoint move / release-to-lock (identical to StatePidHold.cpp).
  if (ci.teleop_active.load()) {
    const Vec3 v_d_body(
      -gains.max_teleop_vel[0] * ci.teleop_surge_x.load(),
      -gains.max_teleop_vel[1] * ci.teleop_sway_y.load(),
      gains.max_teleop_vel[2] * ci.teleop_heave_z.load());
    const Vec3 omega_cmd_body(
      gains.max_teleop_vel[3] * ci.teleop_roll_rate.load(),
      gains.max_teleop_vel[4] * ci.teleop_pitch_rate.load(),
      gains.max_teleop_vel[5] * ci.teleop_yaw_rate.load());

    p_d_world_ += r * v_d_body * dt;
    p_d_world_.x() = std::max(p_d_world_.x(), model_params_.workspace_x_min);
    p_d_world_.z() =
      saturation(p_d_world_.z(), model_params_.workspace_z_min, model_params_.workspace_z_max);

    q_d_ = normalizeQuat(quatMul(q_d_, smallAngleQuat(omega_cmd_body * dt)));
    const double max_tilt_rad = gains.max_tilt_deg * M_PI / 180.0;
    Vec3 q_d_rpy = rotMatToRPY(quatToRotMat(q_d_));
    q_d_rpy(0) = saturation(q_d_rpy(0), -max_tilt_rad, max_tilt_rad);
    q_d_rpy(1) = saturation(q_d_rpy(1), -max_tilt_rad, max_tilt_rad);
    q_d_ = quatFromRPY(q_d_rpy);
  }

  // Strict Zero-Order Hold (CLAUDE.md Phase 4.1 Sec.3.3): writes the latest async 25Hz CUDA MPPI
  // output verbatim into pending_thruster_cmd. If the worker hasn't produced a first valid
  // solve yet (e.g. immediately after enter()), pending_thruster_cmd is simply left untouched --
  // it was zeroed at enter()'s preceding FSM transition / defaults to the all-zero safe state.
  if (ctrl_interfaces_.outer_loop_cmd_.valid.load()) {
    for (int i = 0; i < 8; ++i) {
      ctrl_interfaces_.pending_thruster_cmd[i] = ctrl_interfaces_.outer_loop_cmd_.u_star[i].load();
    }
  }
}

void StateBaselineMppi::exit()
{
}

FSMStateName StateBaselineMppi::checkChange()
{
  return checkFsmCommandChange();
}

PoseTarget StateBaselineMppi::targetPose() const
{
  PoseTarget t;
  t.pos = p_d_world_;
  t.quat = q_d_;
  return t;
}

}  // namespace adhere_controller
