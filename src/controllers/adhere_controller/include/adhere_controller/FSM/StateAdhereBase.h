#ifndef ADHERE_CONTROLLER__FSM__STATEADHEREBASE_H_
#define ADHERE_CONTROLLER__FSM__STATEADHEREBASE_H_

#include <array>
#include <cstdint>

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/common/PoseSetpointIntegrator.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Shared wall-adhesion sequence (CLAUDE.md Phase 5.0 Stage 2.3): extracted out of
// StateAdherePid/StateAdhereSmc/StateBaselineMppi, which were ~90% copy-pasted (live
// wall-target retarget bridge, the sticky STANDOFF_HOMING convergence gate, the SOFT_DOCKING
// dock gate, MODE_* setpoint edge-detection, teleop stick->pose integration + tilt clamp, the
// 3-stage p_d_world_ override, and the lever-arm moment feedforward). The only real
// per-algorithm difference is HOW the wrench pressing the wall is computed, captured in the
// single computeAdhesionWrench() hook.
class StateAdhereBase : public FSMState
{
public:
  StateAdhereBase(
    FSMStateName state_name, std::string state_name_string, CtrlInterfaces & ctrl_interfaces,
    const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  void exit() override;

  PoseTarget targetPose() const override;

  bool targetValid() const override { return true; }

  // Exposed for closed-loop gtests (CLAUDE.md Phase 3.8 Stage 3.1).
  bool isDocked() const { return docked_; }

  // Phase 4.2 (renamed): whether Stage 1 (STANDOFF_HOMING) has completed.
  bool isDepthAligned() const { return standoff_homed_; }

  const char * adhereStageName() const override
  {
    if (!standoff_homed_) {
      return "STANDOFF_HOMING";
    }
    if (!hasDockPhase()) {
      return "WALL_ADHERED";
    }
    return docked_ ? "WALL_ADHERED" : "SOFT_DOCKING";
  }

protected:
  // --- Per-algorithm configuration hooks (static config, cheap to call every tick) ---

  // World X of the wall plane this state homes/docks onto.
  virtual double xWall() const = 0;

  // True for StateAdherePid/StateAdhereSmc (ramp-then-dock sub-phase via dockForceThresh()/
  // xDockEps()/approachSpeed()); false for StateBaselineMppi (no separate docked_ sub-phase --
  // the CUDA rollout cost handles the entire approach+press as one continuous process).
  virtual bool hasDockPhase() const = 0;
  virtual double approachSpeed() const { return 0.0; }
  virtual double dockForceThresh() const { return 0.0; }
  virtual double xDockEps() const { return 0.0; }

  virtual const std::array<double, 6> & maxTeleopVel() const = 0;
  virtual double maxTiltDeg() const = 0;

  // --- Per-algorithm wrench hook ---

  // Computes the 6-DOF body-frame wrench to press/hold the wall this tick, using
  // setpoint_.p_d_world_/q_d_ (already stage-overridden by run() before this is called),
  // standoff_homed_/docked_, and teleop_ (this tick's stick-driven v_d_body/omega_cmd_body).
  // Returns true if wrench_out was filled and should be lever-arm-feedforward-corrected and
  // allocated through b_pinv_ by run() (StateAdherePid/StateAdhereSmc); returns false if the
  // subclass already wrote ctrl_interfaces_.pending_thruster_cmd itself and run() should skip
  // allocation entirely (StateBaselineMppi's strict Zero-Order-Hold ZOH of the async CUDA
  // planner's output, which has no wrench/allocator at all).
  virtual bool computeAdhesionWrench(
    const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
    double sum_f, double dt, Vec6 & wrench_out) = 0;

  Mat86 b_pinv_ = Mat86::Zero();
  PoseSetpointIntegrator setpoint_;
  PoseSetpointIntegrator::TeleopResult teleop_;

  // Phase 4.2 two-stage adhere sequence (renamed/extended Phase 4.5): Stage 1 STANDOFF_HOMING
  // (hold standoff_x/live Y+Z target) must complete before Stage 2 (SOFT_DOCKING/WALL_ADHERED
  // ramp-then-dock logic) begins -- docked_ can only become true once standoff_homed_ is.
  bool standoff_homed_ = false;
  bool docked_ = false;

private:
  double live_target_y_ = 0.0;
  double live_target_z_ = 0.0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEADHEREBASE_H_
