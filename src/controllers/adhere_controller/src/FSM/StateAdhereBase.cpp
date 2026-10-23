#include "adhere_controller/FSM/StateAdhereBase.h"

#include <algorithm>
#include <cmath>

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

StateAdhereBase::StateAdhereBase(
  FSMStateName state_name, std::string state_name_string, CtrlInterfaces & ctrl_interfaces,
  const ModelParams & model_params)
: FSMState(state_name, std::move(state_name_string), ctrl_interfaces, model_params)
{
}

void StateAdhereBase::enter()
{
  const auto & active = ctrl_interfaces_.active_state;
  setpoint_.initPose(
    Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]),
    Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]));
  standoff_homed_ = false;
  docked_ = false;
}

void StateAdhereBase::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  auto & ci = ctrl_interfaces_.control_inputs_;
  const auto & stage = model_params_.adhere_stage;

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

  // Live wall-target retarget bridge (CLAUDE.md Phase 4.5): if the resolved /rov/adhere_target_cmd
  // target just moved by more than stage.retarget_threshold, force a fresh STANDOFF_HOMING pass
  // instead of dragging the ROV sideways/vertically while still pressed against the wall.
  live_target_y_ = ctrl_interfaces_.adhere_target_.target_y.load();
  live_target_z_ = -ctrl_interfaces_.adhere_target_.target_depth_m.load();
  if (ctrl_interfaces_.adhere_target_.changed_pending.load()) {
    standoff_homed_ = false;
    docked_ = false;
    ctrl_interfaces_.adhere_target_.changed_pending.store(false);
  }

  // Stage 1 -> Stage 2 gate (CLAUDE.md Phase 4.2, extended Phase 4.5): STANDOFF_HOMING must
  // converge BOTH lateral Y and depth Z (plus planar speed) before SOFT_DOCKING/WALL_ADHERED's
  // ramp-then-dock logic is allowed to run. Sticky -- never un-homes itself except via the
  // retarget check above.
  if (!standoff_homed_) {
    const double depth_err = pos_world.z() - live_target_z_;
    const double y_err = pos_world.y() - live_target_y_;
    const Vec3 vel_world = r * vel_body;
    const double planar_speed =
      std::sqrt(vel_world.y() * vel_world.y() + vel_world.z() * vel_world.z());
    if (std::abs(depth_err) <= stage.depth_tol && std::abs(y_err) <= stage.y_tol &&
      planar_speed <= stage.vel_tol)
    {
      standoff_homed_ = true;
    }
  }

  // SOFT_DOCKING -> WALL_ADHERED gate (CLAUDE.md Phase 3.8, gated on standoff_homed_ since Phase
  // 4.2): sticky -- never un-docks itself. Not evaluated at all for algorithms with no separate
  // dock sub-phase (StateBaselineMppi).
  if (hasDockPhase() && standoff_homed_ && !docked_ && sum_f >= dockForceThresh() &&
    pos_world.x() <= xWall() + xDockEps())
  {
    docked_ = true;
  }

  // Discrete setpoint + stick-driven Y/Z/attitude advancement (surge never drives X here -- X is
  // owned by the 3-stage override below throughout every phase).
  setpoint_.applyDiscreteSetpoint(ci, pos_world, q_est);
  teleop_ = setpoint_.integrateTeleop(
    ci, maxTeleopVel(), maxTiltDeg(), dt, r, /*drive_surge=*/false,
    model_params_.workspace_x_min, model_params_.workspace_z_min, model_params_.workspace_z_max);

  if (!standoff_homed_) {
    // Stage 1 STANDOFF_HOMING: hold Y/Z at the live wall target and X at a safe standoff
    // distance from the wall -- overrides whatever the setpoint/teleop handling above just
    // computed for Y/Z.
    setpoint_.p_d_world_.y() = live_target_y_;
    setpoint_.p_d_world_.z() = live_target_z_;
    setpoint_.p_d_world_.x() = xWall() + stage.standoff_x;
  } else if (hasDockPhase() && !docked_) {
    // Stage 2 SOFT_DOCKING: ramp p_d_world_.x() toward the wall.
    const double step = approachSpeed() * dt;
    setpoint_.p_d_world_.x() = std::max(setpoint_.p_d_world_.x() - step, xWall());
  } else if (hasDockPhase()) {
    // Stage 2 WALL_ADHERED: re-latch X every tick so the position loop contributes ~0 on Fx,
    // leaving the force-regulation law as the sole driver of that axis.
    setpoint_.p_d_world_.x() = pos_world.x();
  } else {
    // StateBaselineMppi: once homed, the target simply moves onto the wall plane -- the CUDA
    // planner's own rollout cost (wall contact penalty + preload) does the actual approach/press.
    setpoint_.p_d_world_.x() = xWall();
  }

  Vec6 wrench = Vec6::Zero();
  if (computeAdhesionWrench(pos_world, q_est, vel_body, omega_body, sum_f, dt, wrench)) {
    if (standoff_homed_ && !docked_) {
      // Phase 4.3 soft-landing: decelerate into the wall during the pre-dock approach ramp
      // instead of impacting at a constant approach_speed (the Kelvin-Voigt pads' zeta~0.2
      // would otherwise bounce the ROV off on first contact).
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
}

void StateAdhereBase::exit()
{
}

PoseTarget StateAdhereBase::targetPose() const
{
  PoseTarget t;
  t.pos = setpoint_.p_d_world_;
  t.quat = setpoint_.q_d_;
  return t;
}

}  // namespace adhere_controller
