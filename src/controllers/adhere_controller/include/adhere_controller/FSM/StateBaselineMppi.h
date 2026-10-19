#ifndef ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_
#define ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_

#include <array>
#include <cstdint>

#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiController.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Outer-loop-only Baseline MPPI FSM state (CLAUDE.md Phase 4.1 Sec.3): unlike every other FSM
// state, run() at 250Hz performs ZERO inner-loop control law -- it is a strict Zero-Order-Hold
// pass-through of whatever AdhereController::outerWorkerLoop() last computed via the shared
// baseline_mppi::BaselineMppiController (running asynchronously at the 25Hz outer-loop rate on
// its own worker thread) and staged into ctrl_interfaces_.outer_loop_cmd_ (CtrlInterfaces.h's
// lock-free bridge). Setpoint/teleop handling (target pose integration) is otherwise identical to
// StatePidHold's grammar -- this state only decides WHAT to track, the CUDA planner decides HOW.
class StateBaselineMppi final : public FSMState
{
public:
  StateBaselineMppi(
    CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params,
    baseline_mppi::BaselineMppiController & planner);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  PoseTarget targetPose() const override;

private:
  baseline_mppi::BaselineMppiController & planner_;

  Vec3 p_d_world_ = Vec3::Zero();
  Quat q_d_ = Quat(1.0, 0.0, 0.0, 0.0);

  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEBASELINEMPPI_H_
