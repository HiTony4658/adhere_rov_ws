#ifndef ADHERE_CONTROLLER__ALGORITHMS__PID__SIXDOFPIDCONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__PID__SIXDOFPIDCONTROLLER_H_

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Forward-declared (not included): this class only ever holds/passes a const reference to
// FossenDynamics, never its definition. Avoids a header cycle now that ModelParams.h (Phase 5.0
// Stage 2.4) includes this header so ModelParams::PidHoldGains/AdherePidGains can inherit Gains
// directly instead of hand-copying fields -- FossenDynamics.h itself includes ModelParams.h.
class FossenDynamics;

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
  // instead of file-local constants.
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
