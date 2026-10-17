#ifndef ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_
#define ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

namespace adhere_mujoco_hardware
{

// Half-sine velocity pulse in the world frame (e.g. a wave-impact gust). Peaks at the
// midpoint of [t_start, t_end] and tapers to zero at both edges. Exact port of
// mujoco_mppi_ws/src/env/hydrodynamics.py::GustEvent.
struct GustEvent
{
  double t_start = 0.0;
  double t_end = 0.0;
  double peak_y = 0.0;  // m/s, world +Y
  double peak_x = 0.0;  // m/s, world +X (normal "suction" component)

  Eigen::Vector3d velocity(double t) const
  {
    if (t < t_start || t > t_end) {
      return Eigen::Vector3d::Zero();
    }
    const double phase = (t - t_start) / (t_end - t_start) * M_PI;
    const double s = std::sin(phase);
    return Eigen::Vector3d(peak_x * s, peak_y * s, 0.0);
  }

  // Analytic d/dt of velocity(t): same half-sine envelope, scaled by its own angular
  // frequency pi/(t_end-t_start). Zero outside [t_start, t_end] (velocity() is C0 but not C1
  // at the edges -- the gust switching on/off is a modeled discontinuity, not a numerical
  // artifact). Used by the unsteady Froude-Krylov/added-mass force term in
  // HydroModelMode::kStonefishGeometryDriven.
  Eigen::Vector3d acceleration(double t) const
  {
    if (t < t_start || t > t_end) {
      return Eigen::Vector3d::Zero();
    }
    const double omega = M_PI / (t_end - t_start);
    const double c = std::cos((t - t_start) * omega);
    return Eigen::Vector3d(peak_x * omega * c, peak_y * omega * c, 0.0);
  }
};

// Exact port of mujoco_mppi_ws/src/env/hydrodynamics.py::HydroParams. Field defaults match
// the Python calm-water baseline bit-for-bit.
struct HydroParams
{
  double rho_water = 1000.0;      // kg/m^3
  double gravity = 9.81;          // m/s^2
  double vehicle_mass = 20.0;     // kg, dry mass
  double augmented_mass = 26.0;   // kg, MJCF <inertial> mass (dry + added mass)
  double displaced_volume = (1.01 * 20.0) / 1000.0;  // m^3, 1% net positive buoyancy

  Eigen::Vector3d cob_offset_body = Eigen::Vector3d(0.0, 0.0, 0.05);

  double z_surface = 1.0;      // m
  double z_transition = 0.4;   // m

  Eigen::Vector3d D_lin = Eigen::Vector3d(30.0, 20.0, 25.0);
  Eigen::Vector3d D_quad = Eigen::Vector3d(140.0, 50.0, 70.0);
  Eigen::Vector3d D_ang_lin = Eigen::Vector3d(8.0, 8.0, 10.0);
  Eigen::Vector3d D_ang_quad = Eigen::Vector3d(15.0, 15.0, 20.0);

  Eigen::Vector3d current_steady_world = Eigen::Vector3d::Zero();
  std::vector<GustEvent> gust_events;

  double C_venturi = 0.45;
  double h_gap_nominal = 0.025;  // m
  double A_plate = 0.24;         // m^2
  double h0_decay = 0.05;        // m
  double wall_x_offset = 0.1;    // m

  // Stonefish-style per-patch coefficients -- only consumed by
  // HydroModelMode::kStonefishGeometryDriven (Section B4); the parametric-lumped model never
  // reads these, so they have zero cost/effect in the default mode. Placeholder values per
  // the Phase 2.10 task spec, pending future tuning against real tow-tank or CFD data.
  double Cd_normal = 1.2;    // per-patch normal pressure-drag coefficient (subcritical baseline)
  double Cf_tangent = 0.02;  // per-patch tangential skin-friction coefficient (nominal baseline)
  double C_squeeze = 64.0;   // near-wall squeeze-film lubrication coefficient

  // Reynolds-number-dependent drag/friction enhancement (hydrodynamic-accuracy audit, Finding
  // D): this ROV's working speed range (0.1-1 m/s over a ~0.2-0.6m hull) spans Re ~ 9e4-6e5 in
  // water, straddling the subcritical/critical drag-crisis transition for SMOOTH curved bluff
  // bodies (sphere/cylinder). Cf_tangent is corrected via the classical Schlichting turbulent
  // flat-plate correlation (valid for any surface, curved or sharp-edged). Cd_normal's
  // Re-dependent "drag crisis" (Cd dropping from Cd_normal to Cd_normal_supercritical) is
  // PHYSICALLY ONLY APPLICABLE to smooth curved surfaces, where the separation point itself
  // migrates with the laminar-to-turbulent boundary-layer transition -- a sharp-edged surface
  // (this hull's box chassis, or any future mesh with hard edges) has its separation point
  // fixed by the geometry regardless of Re, so Cd_normal stays exactly constant there by
  // design (see SurfacePatch::is_curved below), not as an unmodeled simplification.
  double nu_water = 1.0e-6;             // m^2/s, kinematic viscosity (fresh water ~20C)
  double Cd_normal_supercritical = 0.3; // post-drag-crisis plateau Cd, curved surfaces only
  double Re_crit_low = 1.5e5;           // drag-crisis transition band start (log10(Re) blend)
  double Re_crit_high = 3.5e5;          // drag-crisis transition band end

  // Wall-proximity translation-rotation added-mass coupling (hydrodynamic-accuracy audit,
  // Finding B): UNVALIDATED ENGINEERING PLACEHOLDER, same epistemic status as C_venturi/
  // C_squeeze/Cd_normal above. In unbounded fluid, translation-rotation cross terms in the
  // added-mass kinetic-energy quadratic form vanish exactly by symmetry (no cited closed-form
  // inviscid potential-flow wall-image multipole solution for this cross term was found in the
  // literature during this audit); the wall breaks that symmetry and must in principle induce a
  // nonzero coupling, but only the STRUCTURAL form below (vanishes far from the wall, bounded
  // near it, dimensionally consistent) is defensible -- this scalar gain is a free parameter
  // pending CFD/tow-tank calibration, not a derived coefficient.
  double k_coupling_wall = 0.1;  // dimensionless

  // Frequency-dependent radiation damping / Cummins-equation memory effect (hydrodynamic-
  // accuracy audit, Finding A): a first-order washout (high-pass) filter approximation -- see
  // RadiationMemoryState's doc comment for the full derivation. K_radiation_memory defaults to
  // Vector3d::Zero() (feature OFF / zero-regression default): this term is pending calibration
  // against real frequency-response (panel-method/CFD or tow-tank) data that this project does
  // not have, same epistemic status as the other placeholder coefficients in this file.
  Eigen::Vector3d K_radiation_memory = Eigen::Vector3d::Zero();  // N*s/m, per linear-velocity axis
  double tau_radiation_memory = 0.3;  // s, washout time constant (order of the gust pulse width)

  // Ambient current velocity in the world frame at time t: steady crossflow + superposed
  // wave-gust pulses.
  Eigen::Vector3d currentWorld(double t) const
  {
    Eigen::Vector3d v = current_steady_world;
    for (const auto & gust : gust_events) {
      v += gust.velocity(t);
    }
    return v;
  }

  // Ambient current ACCELERATION in the world frame at time t: steady current contributes
  // zero (constant velocity); gusts contribute the analytic derivative of their half-sine
  // profile. Only used by the unsteady Froude-Krylov/added-mass force term in
  // kStonefishGeometryDriven mode -- kParametricLumped never calls this.
  Eigen::Vector3d currentAccelerationWorld(double t) const
  {
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    for (const auto & gust : gust_events) {
      a += gust.acceleration(t);
    }
    return a;
  }

  // Optional per-model override hook (CLAUDE.md Phase 2.12 Section 4): scans m->nnumeric for
  // <custom><numeric name="hydro_..." data="..."/> entries by name and overwrites the
  // corresponding field in params if present, leaving params completely untouched for any
  // name absent from the MJCF (true for rov_wall.xml today -- zero regression). Lets a future
  // ROV model embed its own dry mass/displaced volume/drag coefficients/etc directly in its
  // own MJCF without editing this header's hardcoded defaults.
  static void applyMjModelOverrides(const mjModel * m, HydroParams & params);
};

// Rigid-body kinematic state needed to evaluate the hydrodynamic wrench, expressed exactly
// as MuJoCo exposes it for a free-jointed body: world-frame CoM position/orientation,
// world-frame angular velocity (as in mjData::cvel[0:3]) and world-frame CoM linear
// velocity (as in mjData::cvel[3:6]).
struct RigidBodyState
{
  Eigen::Vector3d com_pos_world = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // body -> world
  Eigen::Vector3d ang_vel_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d lin_vel_world = Eigen::Vector3d::Zero();
};

struct Wrench
{
  Eigen::Vector3d force_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque_world = Eigen::Vector3d::Zero();
};

// Selects which hydrodynamic model FossenHydrodynamics::compute() evaluates.
enum class HydroModelMode
{
  kParametricLumped = 0,        // existing point-CoM Morison/Venturi model (parity-locked)
  kStonefishGeometryDriven = 1  // geometry/mesh-tessellated distributed model (Section B)
};

// One tessellated/decimated surface element used for per-patch pressure+drag integration in
// kStonefishGeometryDriven mode. All fields are body-local (rov_base frame), fixed for the
// lifetime of the model (the robot's geometry never deforms) -- computed once in
// buildFromMjModel() and never reallocated in the 500Hz hot loop.
struct SurfacePatch
{
  Eigen::Vector3d centroid_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_body = Eigen::Vector3d::Zero();  // outward unit normal
  double area = 0.0;                                      // m^2

  // Reynolds-number-dependent Cd_normal "drag crisis" eligibility (Finding D): true for
  // patches tessellated from a smooth curved geom (sphere/ellipsoid/cylinder/capsule), whose
  // flow-separation point migrates with the boundary layer's laminar-to-turbulent transition;
  // false for sharp-edged geoms (box/mesh), whose separation point is geometrically fixed at
  // the edge and is therefore Re-independent by physical law, not by omission. Set once per
  // geom in HydroGeometryModel::buildFromMjModel(), never mutated afterwards.
  bool is_curved = false;
};

// Cached once-per-model geometric hydrodynamic description, built directly from the
// mjModel's own geoms/meshes (no separately authored hydrodynamic asset needed). Stonefish-
// style: approximates distributed pressure/drag/skin-friction/added-mass over the real hull
// shape instead of FossenHydrodynamics' single lumped-body Morison/Venturi terms, so the
// model stays correct if the robot's geometry ever changes (box -> CAD mesh, etc).
struct HydroGeometryModel
{
  std::vector<SurfacePatch> patches;                             // <= 256 patches
  Eigen::Vector3d bbox_half_extents = Eigen::Vector3d::Zero();    // body-local AABB half-extents
  Eigen::Matrix3d added_mass_linear = Eigen::Matrix3d::Zero();    // M_A (3x3), Lamb-ellipsoid est.
  Eigen::Matrix3d added_mass_angular = Eigen::Matrix3d::Zero();   // I_A (3x3), Lamb-ellipsoid est.

  // Characteristic diameter of the first smooth curved geom encountered during tessellation
  // (Finding D), used as the Cd_normal Reynolds-number length scale for is_curved patches.
  // Stays 0.0 (unused -- no is_curved patch ever reads it) when the hull has no curved geoms,
  // true for rov_wall.xml's pure-box chassis today.
  double cd_diameter = 0.0;

  // Tessellates every geom belonging to root_body_id's rigid hull subtree into surface patches
  // (box: 6 faces x 4x4 grid; sphere/capsule/cylinder/ellipsoid: latitude/longitude grids;
  // mesh: one patch per triangle), decimates to <=256 patches via area-weighted spatial-grid
  // bucketing if needed, and estimates the anisotropic added-mass tensors from the resulting
  // bounding-box half-extents via Lamb's classical ellipsoid approximation.
  //
  // "Rigid hull subtree" (CLAUDE.md Phase 2.12 Section 2) includes root_body_id itself plus any
  // body reachable from it through a chain of welded (body_jntnum == 0) child bodies -- e.g. a
  // future CAD model's hull fairing or buoyancy foam attached as a welded <body> rather than a
  // direct <geom> of root_body_id. Each such child body's geoms are composed through its own
  // welded parent chain (via m->body_pos/body_quat) into root_body_id's frame before
  // tessellation, so every patch's centroid_body/normal_body stays in the one consistent root-
  // body frame FossenHydrodynamics::compute() expects. For rov_wall.xml (root_body_id has no
  // child bodies) the composed transform is always the identity, reproducing the exact same 96
  // patches as before this generalization.
  //
  // rho_water (hydrodynamic-accuracy audit fix): the fluid density used to scale the Lamb-
  // ellipsoid added-mass tensors below. Previously hardcoded to 1000.0 kg/m^3 independent of
  // whatever HydroParams::rho_water the caller's FlowEnvironmentPlugin actually configured --
  // inconsistent with F_fk_am_body's Froude-Krylov term in compute(), which always reads the
  // live params.rho_water. The caller should pass the vehicle's one canonical water density
  // (water density is an environmental constant, not a per-flow-profile property, so every
  // FlowEnvironmentPlugin is expected to agree on it); defaults to 1000.0 so every existing call
  // site/test that omits this argument reproduces prior behavior bit-for-bit.
  static HydroGeometryModel buildFromMjModel(
    const mjModel * m, int root_body_id, double rho_water = 1000.0);
};

// Persistent cross-tick state for the frequency-dependent radiation-damping/memory-effect
// approximation (hydrodynamic-accuracy audit, Finding A). A full Cummins-equation convolution
// force = -Integral_0^t K(t-tau) v(tau) dtau needs a real retardation kernel K(t) that this
// project has no panel-method/CFD data to calibrate; instead this implements a first-order
// washout (high-pass) filter -- the simplest structurally-correct approximation that produces
// EXTRA damping only for velocity components changing faster than tau_radiation_memory allows
// lagged_vel_body to track, and contributes EXACTLY ZERO at true steady state (so it never
// double-counts with the already-tuned quasi-steady D_lin/D_quad terms):
//   d(lagged_vel_body)/dt = (v_rel_body - lagged_vel_body) / tau_radiation_memory
//   F_memory_body = -K_radiation_memory .* (v_rel_body - lagged_vel_body)
// Like t_munk_body/the Finding B coupling terms, this is NOT guaranteed pointwise-dissipative
// (a washout filter can transiently return energy, same as added-mass terms in general) -- it is
// bucketed into its own HydroWrenchBreakdown field, excluded from the Tier4 passivity check.
// Owned by the CALLER (AdhereMujocoHardware holds one instance for the lifetime of its mjData,
// reset alongside it) and passed by pointer into compute(); default nullptr means "feature off",
// reproducing pre-Finding-A behavior exactly regardless of HydroParams::K_radiation_memory.
struct RadiationMemoryState
{
  Eigen::Vector3d lagged_vel_body = Eigen::Vector3d::Zero();
  bool initialized = false;  // first call seeds lagged_vel_body from the current v_rel_body,
                              // avoiding a spurious startup transient spike
};

// Per-term decomposition of the wrench FossenHydrodynamics::compute() returns (Phase 2.13),
// for live inspection (MujocoViewer's "Hydro Breakdown (N)" scope) and for the Tier 4
// thermodynamic-passivity V&V test. All vectors are in the *_body/_world frame their name says;
// each already includes the same submerged_ratio scaling applied to the term it mirrors inside
// compute(), so summing every f_*_body (rotated to world) plus f_buoy_world reproduces
// wrench.force_world exactly. t_drag_body is not part of the CLAUDE.md Phase 2.13 struct
// contract verbatim but is required to express the Tier 4 dissipative-power law
// (v^B . (f_drag+f_squeeze) + w^B . tau_drag < 0): it is the dissipative rotational counterpart
// of f_drag_body (patch pressure/skin/squeeze torques + whole-body D_ang_lin/D_ang_quad angular
// damping), deliberately excluding the workless Munk term t_munk_body.
struct HydroWrenchBreakdown
{
  Eigen::Vector3d f_drag_body = Eigen::Vector3d::Zero();     // pressure + skin + linear drag
  Eigen::Vector3d f_venturi_body = Eigen::Vector3d::Zero();  // near-wall Venturi suction
  Eigen::Vector3d f_squeeze_body = Eigen::Vector3d::Zero();  // near-wall squeeze-film damping
  Eigen::Vector3d f_fk_am_body = Eigen::Vector3d::Zero();    // unsteady Froude-Krylov + added mass
  Eigen::Vector3d f_buoy_world = Eigen::Vector3d::Zero();    // net buoyancy + added-mass bias
  Eigen::Vector3d t_munk_body = Eigen::Vector3d::Zero();     // Munk overturning moment (workless)
  Eigen::Vector3d t_drag_body = Eigen::Vector3d::Zero();     // dissipative drag/skin/squeeze torque

  // Wall-proximity translation-rotation coupling (Finding B, UNVALIDATED placeholder -- see
  // HydroParams::k_coupling_wall). Unlike t_munk_body's two self-coupling terms (v.(vxMv)==0 and
  // w.(wxI_Aw)==0 by the triple-product identity for ANY tensor), this cross term is NOT
  // guaranteed workless at every instant -- it is deliberately excluded from the Tier4
  // dissipative-power check for that reason, same bucket as t_munk_body.
  Eigen::Vector3d f_coupling_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_coupling_body = Eigen::Vector3d::Zero();

  // Frequency-dependent radiation-damping/memory-effect contribution (Finding A). Zero whenever
  // RadiationMemoryState* is nullptr or HydroParams::K_radiation_memory is zero (the default).
  Eigen::Vector3d f_radiation_memory_body = Eigen::Vector3d::Zero();
};

// Port of mujoco_mppi_ws/src/env/hydrodynamics.py::setup_hydrodynamics_callback's per-tick
// passive_callback body. Stateless/pure WITH RESPECT TO ITS ARGUMENTS: every per-step quantity
// (including, since Finding A, the radiation-memory filter state) is passed in by the caller
// rather than held internally, so the same implementation can be shared by AdhereMujocoHardware
// (at 500 Hz) and any future rollout/controller code -- possibly at a different, non-constant
// dt -- without duplicating the physics or silently diverging on timestep assumptions.
class FossenHydrodynamics
{
public:
  // Returns the total hydrodynamic force/torque (world frame, applied at the body CoM) for
  // the given rigid-body state, simulation time t, and hydrodynamic parameters.
  //
  // mode selects the physical model; geo is required (non-null) only for
  // kStonefishGeometryDriven and ignored otherwise. When mode == kParametricLumped (the
  // default) or geo == nullptr, this executes exactly the original point-CoM model,
  // byte-identical to the Phase 2 Python-reference-matched implementation -- a hard parity
  // requirement that must never regress. The radiation-memory term (memory/dt below) is ONLY
  // ever evaluated in the kStonefishGeometryDriven branch -- kParametricLumped never reads
  // memory or dt, preserving that parity lock absolutely.
  //
  // breakdown is an optional output (Phase 2.13): when non-null, it is overwritten with this
  // call's per-term decomposition (see HydroWrenchBreakdown). Defaults to nullptr so every
  // existing caller needs zero changes.
  //
  // memory/dt (Finding A): memory is the caller-owned persistent washout-filter state (see
  // RadiationMemoryState); dt is the physical time elapsed since the caller's PREVIOUS call to
  // compute() (needed to integrate memory's internal ODE one explicit-Euler step forward). The
  // radiation-memory term is entirely skipped -- contributing exactly zero, byte-identical to
  // every pre-Finding-A call site -- whenever memory == nullptr or dt <= 0.0, which is the
  // default for every existing caller that does not pass these two trailing arguments.
  static Wrench compute(
    const RigidBodyState & state, double t, const HydroParams & params,
    HydroModelMode mode = HydroModelMode::kParametricLumped,
    const HydroGeometryModel * geo = nullptr, HydroWrenchBreakdown * breakdown = nullptr,
    RadiationMemoryState * memory = nullptr, double dt = 0.0);
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_
