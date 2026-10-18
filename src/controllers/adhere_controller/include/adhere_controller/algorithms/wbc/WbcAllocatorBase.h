#ifndef ADHERE_CONTROLLER__ALGORITHMS__WBC__WBCALLOCATORBASE_H_
#define ADHERE_CONTROLLER__ALGORITHMS__WBC__WBCALLOCATORBASE_H_

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Phase 4 landing point (CLAUDE.md Sec.5.2, 12D Elastic Rhombic-Kern OSQP): a nominal body-frame
// wrench command handed down from the outer-loop planner (StatePidHold today, KAMR-MPPI's outer
// loop in Phase 4) to a whole-body-control allocator.
struct NominalWrenchCmd
{
  Vec6 wrench_body = Vec6::Zero();
  double timestamp_sec = 0.0;
};

// Result of allocating a NominalWrenchCmd onto the 8 thrusters subject to the Rhombic Kern
// constraint (C_kern*u<=0) and any elastic slack relaxation (CLAUDE.md Sec.5.2's 12D decision
// variable x=[u;s]).
struct WbcAllocationResult
{
  Vec8 u_cmd = Vec8::Zero();
  Vec4 slack = Vec4::Zero();
  bool feasible = true;
  double kern_margin = 1.0;
};

// Data-contract-only abstract interface (CLAUDE.md Phase 3.7 Stage 1.5): no implementation here
// -- Phase 4's 12D Elastic Rhombic-Kern OSQP solver will be the first concrete subclass. Kept
// deliberately thin (just allocate()) so today's FSMState's direct B^dagger allocation and
// Phase 4's OSQP-constrained allocation can be swapped behind the same call site later without
// another FSM-layer refactor.
class WbcAllocatorBase
{
public:
  virtual ~WbcAllocatorBase() = default;

  virtual WbcAllocationResult allocate(const NominalWrenchCmd & cmd, const Vec8 & u_prev) = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__WBC__WBCALLOCATORBASE_H_
