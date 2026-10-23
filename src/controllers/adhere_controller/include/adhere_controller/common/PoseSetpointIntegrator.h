#ifndef ADHERE_CONTROLLER__COMMON__POSESETPOINTINTEGRATOR_H_
#define ADHERE_CONTROLLER__COMMON__POSESETPOINTINTEGRATOR_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/CtrlInterfaces.h"

namespace adhere_controller
{

// Shared reference-pose setpoint/teleop integrator (CLAUDE.md Phase 5.0 Stage 2.2): extracted
// out of StatePidHold/StateAdherePid/StateAdhereSmc/StateBaselineMppi, which all pasted this
// exact block (discrete /rov/setpoint_cmd MODE_* edge-detection + /rov/teleop_cmd stick-driven
// pose integration + tilt clamp). ROS-free (no rclcpp types), gtest-able in isolation like
// common/OutputSafety.h. The one real behavioral difference between callers -- whether surge
// (teleop linear.x) drives p_d_world_.x() -- is the drive_surge parameter of integrateTeleop();
// StatePidHold/StateBaselineMppi pass true, StateAdherePid/StateAdhereSmc pass false (X is owned
// by the wall-homing sequence instead).
class PoseSetpointIntegrator
{
public:
  // Seeds the held reference pose (called from FSMState::enter()). Deliberately does NOT reset
  // last_seen_setpoint_seq_ -- a setpoint seen before this state was last active must stay
  // "already applied" so it is not silently replayed on re-entry.
  void initPose(const Vec3 & pos_world, const Quat & quat)
  {
    p_d_world_ = pos_world;
    q_d_ = quat;
  }

  // Discrete setpoint handling (/rov/setpoint_cmd), edge-detected via ci.setpoint_seq so the
  // same message is not re-applied every tick (which would fight the teleop stick's continuous
  // nudge/move integration below). cmd_type: 2=MODE_FULL_POSE, 3=MODE_LOCK_CURRENT,
  // 4=MODE_LEVEL_ATTITUDE (1=MODE_DEPTH_ONLY has no FSM-layer consumer left since Phase 5.0
  // deleted HOLD_PID_DEPTH).
  void applyDiscreteSetpoint(const ControlInputs & ci, const Vec3 & pos_est, const Quat & q_est)
  {
    const uint64_t seq = ci.setpoint_seq.load();
    if (seq == last_seen_setpoint_seq_) {
      return;
    }
    last_seen_setpoint_seq_ = seq;
    const uint8_t cmd_type = ci.setpoint_command_type.load();
    if (cmd_type == 2) {
      p_d_world_ = Vec3(
        ci.setpoint_target_pos_x.load(), ci.setpoint_target_pos_y.load(),
        ci.setpoint_target_pos_z.load());
      q_d_ = normalizeQuat(Quat(
        ci.setpoint_target_quat_w.load(), ci.setpoint_target_quat_x.load(),
        ci.setpoint_target_quat_y.load(), ci.setpoint_target_quat_z.load()));
    } else if (cmd_type == 3) {
      p_d_world_ = pos_est;
      q_d_ = q_est;
    } else if (cmd_type == 4) {
      // Reset roll/pitch to 0, keep yaw (round-trip through RPY to isolate yaw).
      const Vec3 cur_rpy = rotMatToRPY(quatToRotMat(q_d_));
      q_d_ = quatFromRPY(Vec3(0.0, 0.0, cur_rpy(2)));
    }
  }

  struct TeleopResult
  {
    Vec3 v_d_body = Vec3::Zero();
    Vec3 omega_cmd_body = Vec3::Zero();
  };

  // Stick-driven setpoint advancement / release-to-lock: v_d_body/omega_cmd_body stay exactly
  // zero when teleop is inactive (including the watchdog-forced-zero case), so callers'
  // error/drag-feedforward laws automatically target zero velocity and p_d_world_/q_d_ are
  // simply never advanced -- no separate "release" branch needed. drive_surge selects whether
  // linear.x (surge) drives p_d_world_.x() (and whether the result is clamped to
  // workspace_x_min) -- false callers leave X untouched here, since it is owned by their own
  // wall-homing/ramp sequence instead.
  TeleopResult integrateTeleop(
    const ControlInputs & ci, const std::array<double, 6> & max_teleop_vel, double max_tilt_deg,
    double dt, const RotMat & r, bool drive_surge, double workspace_x_min,
    double workspace_z_min, double workspace_z_max)
  {
    TeleopResult result;
    if (!ci.teleop_active.load()) {
      return result;
    }

    result.v_d_body = Vec3(
      drive_surge ? -max_teleop_vel[0] * ci.teleop_surge_x.load() : 0.0,
      -max_teleop_vel[1] * ci.teleop_sway_y.load(),
      max_teleop_vel[2] * ci.teleop_heave_z.load());
    result.omega_cmd_body = Vec3(
      max_teleop_vel[3] * ci.teleop_roll_rate.load(),
      max_teleop_vel[4] * ci.teleop_pitch_rate.load(),
      max_teleop_vel[5] * ci.teleop_yaw_rate.load());

    p_d_world_ += r * result.v_d_body * dt;
    if (drive_surge) {
      p_d_world_.x() = std::max(p_d_world_.x(), workspace_x_min);
    }
    p_d_world_.z() = saturation(p_d_world_.z(), workspace_z_min, workspace_z_max);

    // Quaternion kinematics integration (CLAUDE.md Phase 3.6 Sec.2.1): q_d <- normalize(q_d (x)
    // dq), dq = smallAngleQuat(omega_cmd_body*dt) -- singularity-free setpoint advancement. The
    // tilt clamp still goes through a cheap RPY round-trip (preserves the +-max_tilt_deg contract
    // exactly; yaw is never clamped so it passes through unaffected).
    q_d_ = normalizeQuat(quatMul(q_d_, smallAngleQuat(result.omega_cmd_body * dt)));
    const double max_tilt_rad = max_tilt_deg * M_PI / 180.0;
    Vec3 q_d_rpy = rotMatToRPY(quatToRotMat(q_d_));
    q_d_rpy(0) = saturation(q_d_rpy(0), -max_tilt_rad, max_tilt_rad);
    q_d_rpy(1) = saturation(q_d_rpy(1), -max_tilt_rad, max_tilt_rad);
    q_d_ = quatFromRPY(q_d_rpy);

    return result;
  }

  Vec3 p_d_world_ = Vec3::Zero();
  Quat q_d_ = Quat(1.0, 0.0, 0.0, 0.0);

private:
  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__POSESETPOINTINTEGRATOR_H_
