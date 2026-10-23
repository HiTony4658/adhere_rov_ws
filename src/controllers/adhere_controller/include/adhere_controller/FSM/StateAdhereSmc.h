#ifndef ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_
#define ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_

#include "adhere_controller/algorithms/smc/SixDofSmcController.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/StateAdhereBase.h"

namespace adhere_controller
{

// Wall-Adhesion SMC baseline (CLAUDE.md Phase 3.8 Stage 2.2.2): the second classical
// wall-adhesion baseline, expected to show stronger disturbance rejection than StateAdherePid at
// the cost of more chattering/energy. The shared STANDOFF_HOMING/SOFT_DOCKING/WALL_ADHERED
// sequence lives in StateAdhereBase (CLAUDE.md Phase 5.0 Stage 2.3); unlike StateAdherePid (which
// swaps controllers between SOFT_DOCKING and WALL_ADHERED), a single SixDofSmcController instance
// drives all 6 DOF throughout both phases -- its own position-tracking term on X already handles
// the approach ramp, so there is no need for a second controller instance just for SOFT_DOCKING.
// The constant wall_preload_fx pressing bias is only added once docked_
// (SixDofSmcController::computeWrench()'s apply_wall_preload argument).
class StateAdhereSmc final : public StateAdhereBase
{
public:
  StateAdhereSmc(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

protected:
  double xWall() const override { return model_params_.adhere_smc.x_wall; }
  bool hasDockPhase() const override { return true; }
  double approachSpeed() const override { return model_params_.adhere_smc.approach_speed; }
  double dockForceThresh() const override { return model_params_.adhere_smc.dock_force_thresh; }
  double xDockEps() const override { return model_params_.adhere_smc.x_dock_eps; }
  const std::array<double, 6> & maxTeleopVel() const override
  {
    return model_params_.adhere_smc.max_teleop_vel;
  }
  double maxTiltDeg() const override { return model_params_.adhere_smc.max_tilt_deg; }

  bool computeAdhesionWrench(
    const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
    double sum_f, double dt, Vec6 & wrench_out) override;

private:
  FossenDynamics dynamics_;
  SixDofSmcController smc_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_
