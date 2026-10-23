#ifndef ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_
#define ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_

#include "adhere_controller/algorithms/pid/AdherePidController.h"
#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/StateAdhereBase.h"

namespace adhere_controller
{

// Wall-Adhesion PID baseline (CLAUDE.md Phase 3.8 Stage 2.2.1): a classical two-stage
// force/pose hybrid controller, the first of two fair wall-adhesion baselines for the Phase 4
// MPPI comparison (free-swimming StatePidHold cannot serve this role -- it never regulates
// wall-normal preload). The shared STANDOFF_HOMING/SOFT_DOCKING/WALL_ADHERED sequence lives in
// StateAdhereBase (CLAUDE.md Phase 5.0 Stage 2.3); this class only supplies the wall-pressing
// wrench law: SixDofPidController drives Y/Z/attitude throughout, and once docked_ the dedicated
// AdherePidController overwrites the wrench's Fx component with a wall-normal force PI law
// (preload + force-error P/I - receding-velocity damping, see AdherePidController.h for the
// exact sign convention).
class StateAdherePid final : public StateAdhereBase
{
public:
  StateAdherePid(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

protected:
  double xWall() const override { return model_params_.adhere_pid.x_wall; }
  bool hasDockPhase() const override { return true; }
  double approachSpeed() const override { return model_params_.adhere_pid.approach_speed; }
  double dockForceThresh() const override { return model_params_.adhere_pid.dock_force_thresh; }
  double xDockEps() const override { return model_params_.adhere_pid.x_dock_eps; }
  const std::array<double, 6> & maxTeleopVel() const override
  {
    return model_params_.adhere_pid.max_teleop_vel;
  }
  double maxTiltDeg() const override { return model_params_.adhere_pid.max_tilt_deg; }

  bool computeAdhesionWrench(
    const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
    double sum_f, double dt, Vec6 & wrench_out) override;

private:
  FossenDynamics dynamics_;
  SixDofPidController pid_;
  AdherePidController force_pid_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEADHEREPID_H_
