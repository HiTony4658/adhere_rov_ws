#ifndef ADHERE_CONTROLLER__COMMON__ATOMICBRIDGE_H_
#define ADHERE_CONTROLLER__COMMON__ATOMICBRIDGE_H_

#include <atomic>
#include <cstdint>

namespace adhere_controller
{

// Shared lock-free cross-thread bridge bookkeeping (CLAUDE.md Phase 5.0 Stage 2.5):
// CtrlInterfaces.h's OuterLoopCommand and AdhereTargetState each independently reinvented a
// valid/seq idiom (AdhereTargetState additionally bolted on a deferred-read command_seq for its
// USE_CURRENT case). Both now derive from this one base so there is a single idiom, not two --
// `valid`/`seq` keep their exact names via public inheritance, so existing
// `ctrl_interfaces_.outer_loop_cmd_.valid.load()`-style call sites are unaffected.
struct AtomicBridgeBase
{
  // Set true once the writer thread has produced at least one real value (false immediately
  // after a (re-)activation so a stale previous-activation's value is never read as current).
  std::atomic<bool> valid{false};
  // Monotonically-incrementing counter the writer bumps on every new value -- lets a reader
  // edge-detect "a new value arrived since I last looked" without a changed-value comparison.
  std::atomic<uint64_t> seq{0};
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__ATOMICBRIDGE_H_
