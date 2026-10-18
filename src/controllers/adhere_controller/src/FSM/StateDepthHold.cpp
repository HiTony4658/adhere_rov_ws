#include "adhere_controller/FSM/StateDepthHold.h"

#include <cmath>

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/ThrusterAllocator.h"

namespace adhere_controller
{

StateDepthHold::StateDepthHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::DEPTH_HOLD, "state_depth_hold", ctrl_interfaces, model_params)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StateDepthHold::enter()
{
  // Only reached via a transition INTO DEPTH_HOLD (checkChange() below never requests a
  // DEPTH_HOLD -> DEPTH_HOLD re-entry), so this is exactly the "entering fresh" case CLAUDE.md
  // Phase 3.1 Stage 2.2 says must reset the integrator; live target_z_ updates while already in
  // this state never call enter() again. Seeds target_z_ from the ROV's current world Z so the
  // hull does not lurch toward a stale leftover setpoint on entry.
  target_z_ = ctrl_interfaces_.active_state.pos_w[2];
  integral_z_ = 0.0;
}

void StateDepthHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state, which
  // AdhereController populates from either ground truth or RovStateEstimator depending on
  // estimation_mode -- this state has no way to tell which.
  const auto & active = ctrl_interfaces_.active_state;
  const double pos_z = active.pos_w[2];
  const Vec3 rpy(active.rpy[0], active.rpy[1], active.rpy[2]);
  const Vec3 vel_body(active.vel_body[0], active.vel_body[1], active.vel_body[2]);
  const Vec3 omega_body(active.omega_body[0], active.omega_body[1], active.omega_body[2]);

  const RotMat r = rpyToRotMat(rpy);
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
  // after AdhereController's 0.25s watchdog forces it to 0 on joystick disconnect), so the
  // stick returning to center naturally latches target_z_ at its current value with no special
  // "release" handling needed.
  const auto & gains = model_params_.depth_hold;
  target_z_ += gains.max_heave_vel * ci.teleop_heave_z.load() * dt;
  target_z_ = saturation(target_z_, model_params_.workspace_z_min, model_params_.workspace_z_max);

  const double e_z = target_z_ - pos_z;

  // Clamped anti-windup integral, derivative-on-measurement (CLAUDE.md Phase 3.1 Stage 2.2):
  // Kd acts on measured world vertical velocity, not d(e_z)/dt, so a live target_z retarget
  // (a setpoint step) never produces a derivative-kick transient.
  constexpr double kIntegralMax = 180.0;  // N, anti-windup clamp on ki_z*integral_z_
  integral_z_ = saturation(
    integral_z_ + e_z * dt, -kIntegralMax / gains.ki_z, kIntegralMax / gains.ki_z);
  const double f_z_world_des = gains.kp_z * e_z + gains.ki_z * integral_z_ - gains.kd_z * vel_z_world;

  // Rotate the world-frame vertical force command into the body frame, then superimpose gentle
  // PD attitude stabilization torques (angle -> 0, rate damping).
  const Vec3 f_world_des(0.0, 0.0, f_z_world_des);
  const Vec3 f_body_des = r.transpose() * f_world_des;
  const Vec3 tau_body_des = -gains.kp_att * rpy - gains.kd_att * omega_body;

  Vec6 wrench_des;
  wrench_des << f_body_des, tau_body_des;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), model_params_.thruster_min[i], model_params_.thruster_max[i]));
  }
}

void StateDepthHold::exit()
{
}

FSMStateName StateDepthHold::checkChange()
{
  return checkFsmCommandChange();
}

std::array<double, 6> StateDepthHold::targetPose() const
{
  return {0.0, 0.0, target_z_, 0.0, 0.0, 0.0};
}

}  // namespace adhere_controller
