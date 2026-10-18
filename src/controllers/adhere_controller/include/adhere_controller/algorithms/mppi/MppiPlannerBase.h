#ifndef ADHERE_CONTROLLER__ALGORITHMS__MPPI__MPPIPLANNERBASE_H_
#define ADHERE_CONTROLLER__ALGORITHMS__MPPI__MPPIPLANNERBASE_H_

#include "adhere_controller/algorithms/wbc/WbcAllocatorBase.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Data-contract-only abstract interface (CLAUDE.md Phase 3.7 Stage 1.5 / Sec.5.1): no
// implementation here -- Phase 4's async CUDA Baseline/KAMR-MPPI outer-loop worker (see
// AdhereController::outerWorkerLoop()'s current trivial pass-through "solve") will be the
// first concrete subclass. solve() takes the same (ActiveState, PoseTarget) snapshot the
// existing OuterLoopInput hand-off already carries, so wiring a real planner in only requires
// replacing outerWorkerLoop()'s body, not the threading/hand-off architecture.
class MppiPlannerBase
{
public:
  virtual ~MppiPlannerBase() = default;

  virtual NominalWrenchCmd solve(const ActiveState & state, const PoseTarget & target) = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__MPPI__MPPIPLANNERBASE_H_
