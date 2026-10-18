#ifndef ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_
#define ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_

#include <array>
#include <cstdint>

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/FSM/FSMState.h"

namespace adhere_controller
{

// Closed-loop 6-DOF position + heading hold (CLAUDE.md Phase 3.2 Stage 2.2.4). On enter(),
// latches the ROV's current (x, y, z, yaw) as the hold setpoint. While /rov/teleop_cmd is
// active, advances that setpoint at commanded body-aligned velocities; the instant the sticks
// return to center (including via AdhereController's 0.25s watchdog forcing teleop_active
// false on joystick disconnect), the setpoint freezes exactly where it was released. Tracks the
// frozen/moving setpoint with a PID + hydrostatic/drag dynamics feedforward wrench, allocated
// onto the 8 thrusters through the shared ThrusterAllocator's damped pseudoinverse.
class StatePidHold final : public FSMState
{
public:
  StatePidHold(CtrlInterfaces & ctrl_interfaces, const ModelParams & model_params);

  void enter() override;

  void run(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void exit() override;

  FSMStateName checkChange() override;

  // Reports the live hold setpoint [x_d, y_d, z_d, phi_d, theta_d, psi_d].
  std::array<double, 6> targetPose() const override;

private:
  Mat86 b_pinv_;

  Vec3 p_d_world_ = Vec3::Zero();
  double phi_d_ = 0.0;    // roll target, rad (Phase 3.5)
  double theta_d_ = 0.0;  // pitch target, rad (Phase 3.5)
  double psi_d_ = 0.0;

  // Body-frame PID integrator states (CLAUDE.md Phase 3.2 Stage 2.2.4's I_p^B / I_R^B), cleared
  // only on enter().
  Vec3 integral_pos_body_ = Vec3::Zero();
  Vec3 integral_att_body_ = Vec3::Zero();

  uint64_t last_seen_setpoint_seq_ = 0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__FSM__STATEPIDHOLD_H_
