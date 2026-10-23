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

}  // namespace

StatePidHold::StatePidHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params)
: FSMState(FSMStateName::HOLD_PID_6DOF, "state_hold_pid_6dof", ctrl_interfaces, model_params),
  dynamics_(model_params),
  // model_params.pid_hold inherits SixDofPidController::Gains directly (CLAUDE.md Phase 5.0
  // Stage 2.4) -- no copy function needed.
  pid_(model_params.pid_hold, dynamics_, kIntegralPosMax, kIntegralAttMax)
{
  b_pinv_ = buildThrusterAllocationPseudoInverse(
    buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir));
}

void StatePidHold::enter()
{
  // Phase 3.5 strict ground-truth isolation: reads ONLY ctrl_interfaces_.active_state.
  const auto & active = ctrl_interfaces_.active_state;
  setpoint_.initPose(
    Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]),
    Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]));

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

  setpoint_.applyDiscreteSetpoint(ci, pos_world, q_est);

  // Stick-driven setpoint move / release-to-lock (CLAUDE.md Phase 3.2 Stage 2.2.4, extended to
  // roll/pitch in Phase 3.5 via Xbox LT/RT, quaternion-integrated in Phase 3.6, extracted into
  // PoseSetpointIntegrator in Phase 5.0 Stage 2.2). drive_surge=true: surge drives p_d_world_.x().
  const auto teleop = setpoint_.integrateTeleop(
    ci, gains.max_teleop_vel, gains.max_tilt_deg, dt, r, /*drive_surge=*/true,
    model_params_.workspace_x_min, model_params_.workspace_z_min, model_params_.workspace_z_max);

  SixDofPidController::Inputs in;
  in.p_d_world = setpoint_.p_d_world_;
  in.q_d = setpoint_.q_d_;
  in.p_est_world = pos_world;
  in.q_est = q_est;
  in.v_d_body = teleop.v_d_body;
  in.omega_cmd_body = teleop.omega_cmd_body;
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

PoseTarget StatePidHold::targetPose() const
{
  PoseTarget t;
  t.pos = setpoint_.p_d_world_;
  t.quat = setpoint_.q_d_;
  return t;
}

}  // namespace adhere_controller
