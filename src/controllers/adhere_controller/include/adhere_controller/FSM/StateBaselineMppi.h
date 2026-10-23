#ifndef ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_
#define ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_

#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiController.h"
#include "adhere_controller/FSM/StateAdhereBase.h"

namespace adhere_controller
{

// Outer-loop-only Baseline MPPI FSM state (CLAUDE.md Phase 4.1 Sec.3): unlike StateAdherePid/
// StateAdhereSmc, run()'s computeAdhesionWrench() hook performs ZERO inner-loop control law --
// it is a strict Zero-Order-Hold pass-through of whatever AdhereController::outerWorkerLoop()
// last computed via the shared baseline_mppi::BaselineMppiController (running asynchronously at
// the 25Hz outer-loop rate) and staged into ctrl_interfaces_.outer_loop_cmd_. It therefore writes
// ctrl_interfaces_.pending_thruster_cmd itself and returns false, telling StateAdhereBase::run()
// to skip the lever-arm-feedforward + b_pinv_ allocation path entirely (there is no wrench here).
// The shared STANDOFF_HOMING/WALL_ADHERED sequence (2 stages -- no separate docked_ sub-phase,
// see hasDockPhase()) lives in StateAdhereBase (CLAUDE.md Phase 5.0 Stage 2.3). Setpoint/teleop
// handling reuses StatePidHold's gains (gains.pid_hold.*): this state only decides WHAT pose to
// track, the CUDA planner decides HOW.
class StateBaselineMppi final : public StateAdhereBase
{
public:
  StateBaselineMppi(
    CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params,
    baseline_mppi::BaselineMppiController & planner);

  void enter() override;

protected:
  double xWall() const override { return planner_.xWall(); }
  bool hasDockPhase() const override { return false; }
  const std::array<double, 6> & maxTeleopVel() const override
  {
    return model_params_.pid_hold.max_teleop_vel;
  }
  double maxTiltDeg() const override { return model_params_.pid_hold.max_tilt_deg; }

  bool computeAdhesionWrench(
    const Vec3 & pos_world, const Quat & q_est, const Vec3 & vel_body, const Vec3 & omega_body,
    double sum_f, double dt, Vec6 & wrench_out) override;

private:
  baseline_mppi::BaselineMppiController & planner_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_
