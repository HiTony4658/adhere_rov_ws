#ifndef ADHERE_CONTROLLER__ALGORITHMS__PID__ADHEREPIDCONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__PID__ADHEREPIDCONTROLLER_H_

namespace adhere_controller
{

// Wall-normal (body +X) force PI regulator (CLAUDE.md Phase 3.8), the piece of
// FSM::StateAdherePid's wrench that SixDofPidController does not cover. Deliberately NOT a
// wrapper around SixDofPidController -- the wall-normal force loop is an orthogonal concern
// (scalar force error -> scalar wrench component) from the Y/Z/attitude position loop, and
// mixing the two inside one class would require SixDofPidController to grow force-mode-specific
// branching it has no other use for.
//
// Sign convention (CLAUDE.md Sec.4.3/Sec.5, project convention verified against
// ThrusterAllocator/rov_wall.xml): the wall sits at world X=0 with outward normal +X; body +X
// points away from the wall. A thruster wrench with NEGATIVE Fx commands net pressing force
// (toward the wall). All gains/magnitudes here (f_preload, f_des, kp_force, ki_force, kd_x) are
// stored and passed as POSITIVE magnitudes; computeNormalForceWrench() applies the sign that
// turns "need more pressing force" into "more negative Fx" internally:
//   A = f_preload + kp_force*(f_des - sum_f_meas) + ki_force*integral_force_
//   Fx_wrench = -A - kd_x * max(0, vx_body)
// The kd_x term's sign matches CLAUDE.md Sec.5.2's own "-D_vx*max(0, vx_b)" term exactly: as the
// hull recedes from the wall (vx_body > 0, body +X velocity), Fx_wrench grows more negative,
// commanding MORE pressing force -- the opposite of receding. The P/I terms likewise make
// Fx_wrench more negative (more pressing) whenever the measured sum of pad forces falls short of
// f_des.
class AdherePidController
{
public:
  struct Gains
  {
    double kp_force = 2.0;
    double ki_force = 0.5;
    double kd_x = 20.0;
    double f_preload = 30.0;            // N, baseline pressing magnitude
    double integral_force_max = 80.0;   // N, anti-windup clamp on integral_force_
  };

  explicit AdherePidController(const Gains & gains) : gains_(gains) {}

  // f_des/sum_f_meas: N (desired / measured-or-estimated sum of the 4 pad normal forces);
  // vx_body: m/s, body-frame +X velocity (+ = receding from the wall). Returns the commanded
  // body-frame +X wrench component (see class comment for the sign convention).
  double computeNormalForceWrench(double f_des, double sum_f_meas, double vx_body, double dt);

  void resetIntegrator() { integral_force_ = 0.0; }

private:
  Gains gains_;
  double integral_force_ = 0.0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__PID__ADHEREPIDCONTROLLER_H_
