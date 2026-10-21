#ifndef ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_
#define ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_

#include <cstdint>

#include "adhere_controller/algorithms/smc/SixDofSmcController.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Wall-Adhesion SMC baseline (CLAUDE.md Phase 3.8 Stage 2.2.2): the second classical
// wall-adhesion baseline, expected to show stronger disturbance rejection than StateAdherePid at
// the cost of more chattering/energy (verified by the comparative gtest, not asserted).
//
// Unlike StateAdherePid (which swaps controllers between SOFT_DOCKING and WALL_ADHERED), a
// single SixDofSmcController instance drives all 6 DOF throughout both phases -- its own
// position-tracking term on X already handles the approach ramp, so there is no need for a
// second controller instance just for SOFT_DOCKING. The only phase-dependent behavior is: (1)
// p_d_world_.x() ramps toward adhere_smc.x_wall during SOFT_DOCKING, then re-latches to the
// current position estimate every tick once WALL_ADHERED (same rationale as StateAdherePid:
// keeps the SMC's own X position-error term ~0 so it never fights the preload bias); (2) the
// constant wall_preload_fx pressing bias is only added to the commanded wrench once
// WALL_ADHERED (SixDofSmcController::computeWrench()'s apply_wall_preload argument). This phase:
// the sliding surface also gained a bounded Y/Z integral term (SixDofSmcController's own ki
// gain) -- a pure P+switching surface had nothing to "ratchet up" against static Coulomb pad
// friction once wall preload engaged, pinning the ROV at whatever Y it first touched the wall
// at.
//
// Docking gate, setpoint (/rov/setpoint_cmd), and teleop (/rov/teleop_cmd) handling mirror
// StateAdherePid exactly (sticky SOFT_DOCKING->WALL_ADHERED transition, Y/Z/attitude-only
// sliding while docked, STANDOFF_HOMING gated on both Y and Z, live-retargetable via
// /rov/adhere_target_cmd).
class StateAdhereSmc final : public FSMState
{
public:
  StateAdhereSmc(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  PoseTarget targetPose() const override;

  bool targetValid() const override { return true; }

  bool isDocked() const { return docked_; }

  // Phase 4.2 (renamed this phase): whether Stage 1 (STANDOFF_HOMING) has completed and Stage 2
  // (SOFT_DOCKING/WALL_ADHERED) has begun. Name kept for test/caller source compatibility.
  bool isDepthAligned() const { return standoff_homed_; }

  const char * adhereStageName() const override
  {
    if (!standoff_homed_) {
      return "STANDOFF_HOMING";
    }
    return docked_ ? "WALL_ADHERED" : "SOFT_DOCKING";
  }

private:
  Mat86 b_pinv_;

  FossenDynamics dynamics_;
  SixDofSmcController smc_;

  Vec3 p_d_world_ = Vec3::Zero();
  Quat q_d_ = Quat(1.0, 0.0, 0.0, 0.0);

  // Phase 4.2 two-stage adhere sequence (see StateAdherePid.h for the full rationale).
  bool standoff_homed_ = false;
  bool docked_ = false;
  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEADHERESMC_H_
