#ifndef ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_
#define ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_

namespace adhere_controller
{

// FUNCTION_ALGORITHM_CATEGORY naming convention (CLAUDE.md, Phase 4.3); numeric FsmCommand IDs
// (1-7 here, 8=ADHERE_MPPI_KAMR not yet implemented) are unchanged from the pre-rename names.
enum class FSMStateName
{
  INVALID,
  IDLE_PASSIVE_NONE,
  HOLD_PID_DEPTH,
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
