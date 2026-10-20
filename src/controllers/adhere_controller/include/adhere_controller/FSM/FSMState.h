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
  // subscription, 0=NO_CHANGE..7=ADHERE_MPPI_BASELINE) onto the requested FSMStateName, or this state's own
  // name if no change is requested. All concrete states share this identical mapping, so each
  // one's checkChange() override is just `return checkFsmCommandChange();` -- replaces the
  // Phase 3.1 pattern of each state hardcoding its own `== 1`/`== 2` comparison, which does not
  // scale cleanly to many states.
  FSMStateName checkFsmCommandChange() const
  {
    switch (ctrl_interfaces_.control_inputs_.command.load()) {
      case 1:
        return FSMStateName::IDLE_PASSIVE_NONE;
      case 2:
        return FSMStateName::HOLD_PID_DEPTH;
      case 3:
        return FSMStateName::TELEOP_OPENLOOP_6DOF;
      case 4:
        return FSMStateName::HOLD_PID_6DOF;
      case 5:
        return FSMStateName::ADHERE_PID_WBC;
      case 6:
        return FSMStateName::ADHERE_SMC_WBC;
      case 7:
        return FSMStateName::ADHERE_MPPI_BASELINE;
      default:
        return state_name;
    }
  }

  virtual FSMStateName checkChange() { return checkFsmCommandChange(); }

  // Live reference-pose report for /rov/controller_state (CLAUDE.md Phase 3.6's
  // ControllerState.msg::target_pose, geometry_msgs/Pose), published by AdhereController on
  // every outer-loop tick. Meaning is per-state: IDLE_PASSIVE_NONE/TELEOP_OPENLOOP_6DOF have no
  // reference pose (default identity, targetValid()==false); StateDepthHold overrides pos.z (and,
  // Phase 4.3, the actual x/y so the target sphere sits on the ROV's vertical axis);
  // StatePidHold/StateAdhere* override the full 6-DOF pose (position + quaternion).
  virtual PoseTarget targetPose() const { return PoseTarget{}; }

  // Phase 4.3: whether targetPose()/ControllerState.msg::target_pose is actually meaningful for
  // this state -- gates the universal green target-sphere/overlay display (previously hardcoded
  // to only the three ADHERE_* states). Default false (IDLE_PASSIVE_NONE/TELEOP_OPENLOOP_6DOF);
  // overridden true by every state that returns a real targetPose().
  virtual bool targetValid() const { return false; }

  // Phase 4.2 two-stage adhere sequence: "" for every state except ADHERE_PID_WBC/
  // ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE, which report "DEPTH_ALIGN" or "WALL_ADHERE" (see each
  // state's depth_aligned_ member). Read by AdhereController to populate
  // ControllerState.msg::adhere_stage.
  virtual const char * adhereStageName() const { return ""; }

  FSMStateName state_name;
  std::string state_name_string;

protected:
  CtrlInterfaces & ctrl_interfaces_;
  const ModelParams & model_params_;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__FSMSTATE_H_
