#ifndef ADHERE_CONTROLLER__FSM__FSMSTATE_H_
#define ADHERE_CONTROLLER__FSM__FSMSTATE_H_

#include <array>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "adhere_controller/common/enumClass.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"

namespace adhere_controller
{

// Live reference pose (CLAUDE.md Phase 3.6: replaces the old Euler float64[6] -- quaternion
// orientation avoids gimbal lock and matches ControllerState.msg::target_pose's
// geometry_msgs/Pose). Default is position [0,0,0] + identity orientation.
struct PoseTarget
{
  Vec3 pos = Vec3::Zero();
  Quat quat = Quat(1.0, 0.0, 0.0, 0.0);
};

class FSMState
{
public:
  virtual ~FSMState() = default;

  FSMState(
    FSMStateName state_name, std::string state_name_string, CtrlInterfaces & ctrl_interfaces,
    const ModelParams & model_params)
  : state_name(state_name),
    state_name_string(std::move(state_name_string)),
    ctrl_interfaces_(ctrl_interfaces),
    model_params_(model_params)
  {
  }

  virtual void enter() = 0;

  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual void exit() = 0;

  // Default implementation (CLAUDE.md Phase 3.2 Stage 2.2, extended Phase 3.8): maps the live
  // control_inputs_.command atomic (set by AdhereController's /rov/fsm_command
  // subscription, 0=NO_CHANGE..7=BASELINE_MPPI) onto the requested FSMStateName, or this state's own
  // name if no change is requested. All concrete states share this identical mapping, so each
  // one's checkChange() override is just `return checkFsmCommandChange();` -- replaces the
  // Phase 3.1 pattern of each state hardcoding its own `== 1`/`== 2` comparison, which does not
  // scale cleanly to many states.
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
      case 5:
        return FSMStateName::ADHERE_PID;
      case 6:
        return FSMStateName::ADHERE_SMC;
      case 7:
        return FSMStateName::BASELINE_MPPI;
      default:
        return state_name;
    }
  }

  virtual FSMStateName checkChange() { return checkFsmCommandChange(); }

  // Live reference-pose report for /rov/controller_state (CLAUDE.md Phase 3.6's
  // ControllerState.msg::target_pose, geometry_msgs/Pose), published by AdhereController on
  // every outer-loop tick. Meaning is per-state: PASSIVE/OPEN_LOOP have no reference pose
  // (default identity); StateDepthHold overrides pos.z only; StatePidHold overrides the full
  // 6-DOF pose (position + quaternion).
  virtual PoseTarget targetPose() const { return PoseTarget{}; }

  // Phase 4.2 two-stage adhere sequence: "" for every state except ADHERE_PID/ADHERE_SMC/
  // BASELINE_MPPI, which report "DEPTH_ALIGN" or "WALL_ADHERE" (see each state's depth_aligned_
  // member). Read by AdhereController to populate ControllerState.msg::adhere_stage, consumed
  // downstream by MujocoVisBridge's target-pose overlay/green target sphere (only shown while
  // this string is non-empty).
  virtual const char * adhereStageName() const { return ""; }

  FSMStateName state_name;
  std::string state_name_string;

protected:
  CtrlInterfaces & ctrl_interfaces_;
  const ModelParams & model_params_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__FSMSTATE_H_
