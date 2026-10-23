#ifndef ADHERE_CONTROLLER__COMMON__FSMREGISTRY_H_
#define ADHERE_CONTROLLER__COMMON__FSMREGISTRY_H_

#include <array>
#include <cstdint>

#include "adhere_controller/common/enumClass.h"

namespace adhere_controller
{

// Single source of truth for the FsmCommand numeric ID <-> FSMStateName <-> display-string
// mapping (CLAUDE.md Phase 5.0 Stage 2.1: replaces the former 4 independently-hand-maintained
// mappings -- FSMState::checkFsmCommandChange()'s switch, AdhereController::fsmStateNameToInt()'s
// switch, AdhereController::getNextState()'s switch, and FSMStateList's named fields). Adding a
// new FSM state means adding exactly one row here (plus one constructor call in
// AdhereController::on_activate()'s fsm_states_ table) instead of editing 6+ call sites.
struct FsmRegistryEntry
{
  int32_t id;
  FSMStateName name;
  const char * name_string;
};

inline constexpr std::array<FsmRegistryEntry, 6> kFsmRegistry{{
  {1, FSMStateName::IDLE_PASSIVE_NONE, "state_idle_passive_none"},
  {2, FSMStateName::TELEOP_OPENLOOP_6DOF, "state_teleop_openloop_6dof"},
  {3, FSMStateName::HOLD_PID_6DOF, "state_hold_pid_6dof"},
  {4, FSMStateName::ADHERE_PID_WBC, "state_adhere_pid_wbc"},
  {5, FSMStateName::ADHERE_SMC_WBC, "state_adhere_smc_wbc"},
  {6, FSMStateName::ADHERE_MPPI_BASELINE, "state_adhere_mppi_baseline"},
}};

// Inverse of the id->name direction: unknown/0 (NO_CHANGE) resolves to FSMStateName::INVALID,
// which every caller treats as "no change requested" (FSMState::checkFsmCommandChange() falls
// back to its own current state_name in that case).
inline FSMStateName fsmIdToStateName(int32_t id)
{
  for (const auto & entry : kFsmRegistry) {
    if (entry.id == id) {
      return entry.name;
    }
  }
  return FSMStateName::INVALID;
}

inline int32_t fsmStateNameToInt(FSMStateName name)
{
  for (const auto & entry : kFsmRegistry) {
    if (entry.name == name) {
      return entry.id;
    }
  }
  return 0;
}

inline const char * fsmStateNameToString(FSMStateName name)
{
  for (const auto & entry : kFsmRegistry) {
    if (entry.name == name) {
      return entry.name_string;
    }
  }
  return "";
}

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__FSMREGISTRY_H_
