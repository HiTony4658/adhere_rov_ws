#ifndef ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_
#define ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_

namespace adhere_controller
{

// FUNCTION_ALGORITHM_CATEGORY naming convention (CLAUDE.md, Phase 4.3). Phase 5.0: HOLD_PID_
// DEPTH deleted (redundant with HOLD_PID_6DOF) and numeric FsmCommand IDs compacted to 1-7
// (see FsmCommand.msg). ADHERE_MPPI_KAMR (FsmCommand ID 7) remains a rename-only placeholder
// with no concrete FSMState/enum entry yet -- commanding it is a no-op (falls through to
// checkFsmCommandChange()'s default case), identical to the pre-5.0 behavior of commanding the
// old ID 8.
enum class FSMStateName
{
  INVALID,
  IDLE_PASSIVE_NONE,
  TELEOP_OPENLOOP_6DOF,
  HOLD_PID_6DOF,
  ADHERE_PID_WBC,
  ADHERE_SMC_WBC,
  ADHERE_MPPI_BASELINE
};

enum class FSMMode
{
  NORMAL,
  CHANGE
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_
