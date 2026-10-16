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
  double Cd_normal = 1.2;    // per-patch normal pressure-drag coefficient
  double Cf_tangent = 0.02;  // per-patch tangential skin-friction coefficient
  double C_squeeze = 64.0;   // near-wall squeeze-film lubrication coefficient

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

  // Tessellates every geom belonging to root_body_id into surface patches (box: 6 faces x
  // 4x4 grid; sphere/capsule/cylinder/ellipsoid: latitude/longitude grids; mesh: one patch
  // per triangle), decimates to <=256 patches via area-weighted spatial-grid bucketing if
  // needed, and estimates the anisotropic added-mass tensors from the resulting bounding-box
  // half-extents via Lamb's classical ellipsoid approximation. root_body_id's geoms are
  // assumed to all be direct geoms of that one body (no welded child-body recursion -- true
  // for the current rov_wall.xml; extend via m->body_parentid if a future model adds welded
  // children).
  static HydroGeometryModel buildFromMjModel(const mjModel * m, int root_body_id);
};

// Exact C++ port of mujoco_mppi_ws/src/env/hydrodynamics.py::setup_hydrodynamics_callback's
// per-tick passive_callback body. Stateless/pure: all per-step quantities are passed in, so
// the same implementation can be shared by AdhereMujocoHardware (at 500 Hz) and any future
// rollout/controller code without duplicating the physics.
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
  // requirement that must never regress.
  static Wrench compute(
    const RigidBodyState & state, double t, const HydroParams & params,
    HydroModelMode mode = HydroModelMode::kParametricLumped,
    const HydroGeometryModel * geo = nullptr);
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_
