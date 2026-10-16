#ifndef ADHERE_MUJOCO_HARDWARE__RHOMBIC_KERN_HPP_
#define ADHERE_MUJOCO_HARDWARE__RHOMBIC_KERN_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace adhere_mujoco_hardware
{

// Center-of-Pressure + Rhombic Kern admissibility classification for one 500Hz physics tick's 4
// corner pad normal forces (CLAUDE.md Sec.4.4). Factored out of
// AdhereMujocoHardware::write()/read() into a header-only pure function (hydrodynamic-accuracy
// audit) so it is unit-testable without a live mjModel/pluginlib/ResourceManager context -- see
// test/test_rhombic_kern_and_thruster_model.cpp.
struct RhombicKernResult
{
  double cop_y = 0.0;
  double cop_z = 0.0;
  double kern_val = 2.0;  // K_val; > 1.0 means the CoP is outside the admissible Rhombic Kern
  double m_kern = -1.0;   // signed margin, 1 - K_val
  bool is_liftoff = false;      // any pad at/below the contact-force noise floor
  bool is_corner_trap = false;  // CoP inside the support rectangle but outside the diamond
  int liftoff_pad_count = 0;
};

// pad_force: 4 measured normal contact forces [TL, TR, BL, BR] (negative/noise values are
// clamped to >= 0 internally). pad_yz: body-local (y, z) of the same 4 pads, same order.
// liftoff_force_floor: contact-force noise floor below which a pad counts as lifted off.
//
// Guards hydrodynamic-accuracy audit finding 5.3: L_y/L_z (the Rhombic Kern's half-width/
// half-height, derived from the pads' own geometry rather than hardcoded) are floored at
// kMinSupportExtent before being used as a divisor, so a hypothetical future MJCF with
// degenerate (collinear) pad placement produces a large-but-finite kern_val instead of a silent
// NaN/Inf that would otherwise propagate into every consumer of this value (viewer CoP color,
// cumulative liftoff/corner-trap ratios, etc).
inline RhombicKernResult classifyRhombicKern(
  const std::array<double, 4> & pad_force, const std::array<std::pair<double, double>, 4> & pad_yz,
  double liftoff_force_floor = 1e-3)
{
  constexpr double kMinSupportExtent = 1e-6;  // m, divide-by-zero guard (finding 5.3)

  double L_y = 0.0, L_z = 0.0;
  for (const auto & yz : pad_yz) {
    L_y = std::max(L_y, std::abs(yz.first));
    L_z = std::max(L_z, std::abs(yz.second));
  }
  L_y = std::max(L_y, kMinSupportExtent);
  L_z = std::max(L_z, kMinSupportExtent);

  RhombicKernResult r;
  double F_sum = 0.0, Fy_sum = 0.0, Fz_sum = 0.0;
  double F_min = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pad_force.size(); ++i) {
    const double F_i = std::max(0.0, pad_force[i]);
    F_sum += F_i;
    Fy_sum += F_i * pad_yz[i].first;
    Fz_sum += F_i * pad_yz[i].second;
    F_min = std::min(F_min, F_i);
    if (F_i <= liftoff_force_floor) {
      ++r.liftoff_pad_count;
    }
  }
  if (F_sum > 1e-3) {
    r.cop_y = Fy_sum / F_sum;
    r.cop_z = Fz_sum / F_sum;
    r.kern_val = std::abs(r.cop_y) / L_y + std::abs(r.cop_z) / L_z;
    r.m_kern = 1.0 - r.kern_val;
  }
  r.is_liftoff = (F_min <= liftoff_force_floor);
  r.is_corner_trap = (F_sum > 1e-3) && (std::abs(r.cop_y) <= L_y) &&
    (std::abs(r.cop_z) <= L_z) && (r.kern_val > 1.0);
  return r;
}

// First-order ESC lag filter step (CLAUDE.md Sec.3.1 "thruster lag"): alpha = 1 - exp(-dt/tau_T),
// precomputed by the caller since it is constant for a fixed dt/tau_T.
inline double applyThrusterLagStep(double previous_applied, double target, double alpha)
{
  return (1.0 - alpha) * previous_applied + alpha * target;
}

// ESC null-band deadzone (CLAUDE.md Sec.3.1 "deadzone"): below deadzone_n, forces exactly zero.
inline double applyThrusterDeadzone(double u_target, double deadzone_n)
{
  return std::abs(u_target) < deadzone_n ? 0.0 : u_target;
}

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__RHOMBIC_KERN_HPP_
