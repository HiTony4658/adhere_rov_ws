#ifndef ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_
#define ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_

#include <array>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "adhere_controller_common/common/enumClass.h"
#include "adhere_controller_common/CtrlInterfaces.h"

namespace adhere_controller_common
{

class FSMState
{
public:
  virtual ~FSMState() = default;

  FSMState(FSMStateName state_name, std::string state_name_string, CtrlInterfaces & ctrl_interfaces)
  : state_name(state_name),
    state_name_string(std::move(state_name_string)),
    ctrl_interfaces_(ctrl_interfaces)
  {
  }

  virtual void enter() = 0;

  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual void exit() = 0;

  // Default implementation (CLAUDE.md Phase 3.2 Stage 2.2): maps the live
  // control_inputs_.command atomic (set by AdhereKamrController's /rov/fsm_command
  // subscription, 0=NO_CHANGE..4=PID_HOLD) onto the requested FSMStateName, or this state's own
  // name if no change is requested. All 4 concrete states share this identical mapping, so each
  // one's checkChange() override is just `return checkFsmCommandChange();` -- replaces the
  // Phase 3.1 pattern of each state hardcoding its own `== 1`/`== 2` comparison, which does not
  // scale cleanly to 4 states.
  FSMStateName checkFsmCommandChange() const
  {
    switch (ctrl_interfaces_.control_inputs_.command.load()) {
      case 1:
        return FSMStateName::PASSIVE;
      case 2:
        return FSMStateName::DEPTH_HOLD;
      case 3:
        return FSMStateName::OPEN_LOOP;
      case 4:
        return FSMStateName::PID_HOLD;
      default:
        return state_name;
    }
  }

  virtual FSMStateName checkChange() { return checkFsmCommandChange(); }

  // Live reference-pose report for /rov/fsm_state (CLAUDE.md Phase 3.2 Stage 2.1's
  // FsmState.msg::target_pose), published by AdhereKamrController on every outer-loop tick.
  // Meaning is per-state: PASSIVE/OPEN_LOOP have no reference pose (default all-zero);
  // StateDepthHold overrides index 2 (Z) only; StatePidHold overrides the full [x,y,z,yaw].
  virtual std::array<double, 4> targetPose() const { return {0.0, 0.0, 0.0, 0.0}; }

  FSMStateName state_name;
  std::string state_name_string;

protected:
  CtrlInterfaces & ctrl_interfaces_;
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_
