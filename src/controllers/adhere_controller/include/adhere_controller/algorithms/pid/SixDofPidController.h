#ifndef ADHERE_CONTROLLER__ALGORITHMS__PID__SIXDOFPIDCONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__PID__SIXDOFPIDCONTROLLER_H_

#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"

namespace adhere_controller
{

// Decoupled 6-DOF position + SO(3) unit-quaternion attitude PID (CLAUDE.md Phase 3.7 Stage 1.3),
// extracted from the error->wrench law previously inline in StatePidHold.cpp. Setpoint
// integration (stick-driven advancement, discrete MODE_* handling) stays in the FSM layer --
// this class only turns a (desired, estimated) pose/twist pair into a body-frame wrench, with:
//   - hemisphere-aligned quaternion attitude error (mathTools.h's alignQuaternionHemisphere() +
//     quatError());
//   - conditional integral anti-windup (freezes an axis's integration once that axis's integral
//     term is already at its clamp AND the new error would push it further past that clamp);
//   - derivative-on-measurement (kd terms multiply measured body velocity/angular-velocity
//     error, never a differenced setpoint, so a setpoint step never causes a derivative kick);
//   - hydrostatic buoyancy/drag feedforward via an injected FossenDynamics.
// StateDepthHold (Phase 3.7) drives this same class in a reduced "depth-only" mode: zero
// lateral position gains, q_d=identity, DepthHoldGains mapped onto this class's Gains struct --
// see StateDepthHold.cpp for the exact mapping and the rationale for why that is behaviorally
// equivalent to the pre-Phase-3.7 inline law.
class SixDofPidController
{
public:
  struct Gains
  {
    Vec3 kp_pos = Vec3::Zero();
    Vec3 ki_pos = Vec3::Zero();
    Vec3 kd_vel = Vec3::Zero();
    Vec3 kp_att = Vec3::Zero();
    Vec3 ki_att = Vec3::Zero();
    Vec3 kd_att = Vec3::Zero();
  };

  struct Inputs
  {
    Vec3 p_d_world = Vec3::Zero();
    Quat q_d = Quat(1.0, 0.0, 0.0, 0.0);
    Vec3 p_est_world = Vec3::Zero();
    Quat q_est = Quat(1.0, 0.0, 0.0, 0.0);
    Vec3 v_d_body = Vec3::Zero();       // desired body-frame linear velocity (setpoint rate)
    Vec3 omega_cmd_body = Vec3::Zero();  // desired body-frame angular velocity (setpoint rate)
    Vec3 v_est_body = Vec3::Zero();
    Vec3 omega_est_body = Vec3::Zero();
  };

  // integral_pos_max/integral_att_max are the same anti-windup clamp constants StatePidHold.cpp
  // already used (kIntegralPosMax=180N, kIntegralAttMax=10N*m), now parameters of this class
  // instead of file-local constants so StateDepthHold's reduced mode can supply its own.
  SixDofPidController(
    const Gains & gains, const FossenDynamics & dynamics, double integral_pos_max,
    double integral_att_max);

  Vec6 computeWrench(const Inputs & in, double dt);

  void resetIntegrators();

private:
  Gains gains_;
  const FossenDynamics & dynamics_;
  double integral_pos_max_;
  double integral_att_max_;
  Vec3 integral_pos_body_ = Vec3::Zero();
  Vec3 integral_att_body_ = Vec3::Zero();
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__PID__SIXDOFPIDCONTROLLER_H_
