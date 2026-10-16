#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace adhere_mujoco_hardware
{

namespace
{
// Minimum body->wall gap floor, used to avoid a divide-by-zero/singularity in the near-wall
// suction/squeeze force terms below (hydrodynamic-accuracy audit finding 5.1). The two
// hydrodynamic modes measure "gap" from two different reference points and are deliberately
// NOT unified to a single numeric value:
//   - kWallGapFloorComRef (kParametricLumped's suction term, and the wall-proximity added-mass
//     correction below): measured from the single lumped CoM proxy point. This exact literal
//     (0.001) is part of kParametricLumped's byte-identical-to-Python-reference parity
//     contract (see the "parity-locked" comment on that branch) and must never change.
//   - kWallGapFloorPatchRef (kStonefishGeometryDriven's per-patch suction/squeeze loop): measured
//     from each wall-facing patch's own true surface position, which is already much closer to
//     the wall than the CoM by construction, so it needs a correspondingly looser floor to keep
//     the cubic squeeze-film term (h_gap_nominal/h_gap)^3 from growing unreasonably large.
// Silently forcing these to the same value would either break kParametricLumped's parity lock or
// change kStonefishGeometryDriven's near-contact squeeze-force ceiling in a way not re-validated
// against the V&V suite in test/test_fossen_hydrodynamics_geometry.cpp -- hence two named,
// documented constants instead of one.
constexpr double kWallGapFloorComRef = 0.001;    // m, CoM-referenced
constexpr double kWallGapFloorPatchRef = 0.005;  // m, patch-surface-referenced
}  // namespace

void HydroParams::applyMjModelOverrides(const mjModel * m, HydroParams & params)
{
  // Looks up a <custom><numeric name="name" data="..."/> entry by name; returns nullptr (and
  // leaves the corresponding field untouched) if the MJCF declares no numeric of that name --
  // true for every name below on rov_wall.xml today, which is exactly the required zero-
  // regression default.
  auto findNumeric = [m](const char * name, int min_size) -> const double * {
    const int id = mj_name2id(m, mjOBJ_NUMERIC, name);
    if (id < 0 || m->numeric_size[id] < min_size) {
      return nullptr;
    }
    return m->numeric_data + m->numeric_adr[id];
  };
  auto scalarOverride = [&](const char * name, double & field) {
    if (const double * d = findNumeric(name, 1)) {
      field = d[0];
    }
  };
  auto vec3Override = [&](const char * name, Eigen::Vector3d & field) {
    if (const double * d = findNumeric(name, 3)) {
      field = Eigen::Vector3d(d[0], d[1], d[2]);
    }
  };

  scalarOverride("hydro_vehicle_mass", params.vehicle_mass);
  scalarOverride("hydro_augmented_mass", params.augmented_mass);
  scalarOverride("hydro_displaced_volume", params.displaced_volume);
  vec3Override("hydro_cob_offset_body", params.cob_offset_body);
  scalarOverride("hydro_z_surface", params.z_surface);
  vec3Override("hydro_D_lin", params.D_lin);
  vec3Override("hydro_D_quad", params.D_quad);
  vec3Override("hydro_D_ang_lin", params.D_ang_lin);
  vec3Override("hydro_D_ang_quad", params.D_ang_quad);
  scalarOverride("hydro_C_venturi", params.C_venturi);
  scalarOverride("hydro_h_gap_nominal", params.h_gap_nominal);
  scalarOverride("hydro_A_plate", params.A_plate);
  scalarOverride("hydro_C_squeeze", params.C_squeeze);
  scalarOverride("hydro_Cd_normal", params.Cd_normal);
  scalarOverride("hydro_Cf_tangent", params.Cf_tangent);
}

Wrench FossenHydrodynamics::compute(
  const RigidBodyState & state, double t, const HydroParams & params, HydroModelMode mode,
  const HydroGeometryModel * geo, HydroWrenchBreakdown * breakdown)
{
  if (breakdown) {
    *breakdown = HydroWrenchBreakdown{};
  }

  if (mode == HydroModelMode::kParametricLumped || geo == nullptr) {
    // --- HydroModelMode::kParametricLumped: unchanged from Phase 2, byte-identical to the
    // Python-reference-matched implementation. Do not edit this branch without re-verifying
    // numerical parity (see test/test_fossen_hydrodynamics_geometry.cpp). ---
    const Eigen::Matrix3d R = state.orientation.toRotationMatrix();  // body -> world

    const Eigen::Vector3d & v_world = state.lin_vel_world;
    const Eigen::Vector3d & w_world = state.ang_vel_world;
    const Eigen::Vector3d w_body = R.transpose() * w_world;

    const Eigen::Vector3d current_world_t = params.currentWorld(t);
    const Eigen::Vector3d v_rel_world = v_world - current_world_t;
    const Eigen::Vector3d v_rel_body = R.transpose() * v_rel_world;

    // Coupled Morison drag, body frame: quadratic term scales by the total relative speed.
    const double V_rel_norm = v_rel_body.norm();
    const Eigen::Vector3d F_drag_body = -(
      params.D_lin.cwiseProduct(v_rel_body) + params.D_quad.cwiseProduct(v_rel_body) * V_rel_norm);
    const Eigen::Vector3d tau_drag_body = -(
      params.D_ang_lin.cwiseProduct(w_body) +
      params.D_ang_quad.cwiseProduct(w_body.cwiseProduct(w_body.cwiseAbs())));

    const Eigen::Vector3d F_drag_world = R * F_drag_body;
    const Eigen::Vector3d tau_drag_world = R * tau_drag_body;

    // Free-surface taper.
    const double com_z = state.com_pos_world.z();
    const double submerged_ratio = std::clamp(
      (params.z_surface + params.z_transition - com_z) / params.z_transition, 0.0, 1.0);

    // Buoyancy + added-mass gravity-bias correction (CLAUDE.md's 58.86 N figure at m_aug=26,
    // m=20, g=9.81).
    const double F_buoy_full = params.rho_water * params.displaced_volume * params.gravity;
    const double added_mass_gravity_bias =
      (params.augmented_mass - params.vehicle_mass) * params.gravity;
    const Eigen::Vector3d F_buoy_world(
      0.0, 0.0, (F_buoy_full + added_mass_gravity_bias) * submerged_ratio);

    // Restoring moment: r_(CoB<-CoM) x F_B.
    const Eigen::Vector3d r_cob_world = R * params.cob_offset_body;
    const Eigen::Vector3d tau_restore_world = r_cob_world.cross(F_buoy_world);

    // Venturi/Bernoulli wall-suction.
    const double x_com = state.com_pos_world.x();
    const double h_gap = std::max(x_com - params.wall_x_offset, kWallGapFloorComRef);
    const double v_tangent_sq = v_rel_body.y() * v_rel_body.y() + v_rel_body.z() * v_rel_body.z();
    const double F_suction_x = -0.5 * params.rho_water * params.C_venturi * params.A_plate *
      v_tangent_sq * std::exp(-(h_gap - params.h_gap_nominal) / params.h0_decay);
    const Eigen::Vector3d F_suction_body(F_suction_x, 0.0, 0.0);
    const Eigen::Vector3d F_suction_world = R * F_suction_body;

    Wrench wrench;
    wrench.force_world =
      F_drag_world * submerged_ratio + F_buoy_world + F_suction_world * submerged_ratio;
    wrench.torque_world = tau_drag_world * submerged_ratio + tau_restore_world;

    if (breakdown) {
      breakdown->f_drag_body = F_drag_body * submerged_ratio;
      breakdown->f_venturi_body = F_suction_body * submerged_ratio;
      breakdown->f_buoy_world = F_buoy_world;
      breakdown->t_drag_body = tau_drag_body * submerged_ratio;
      // f_squeeze_body, f_fk_am_body, t_munk_body: kParametricLumped has no squeeze-film,
      // unsteady Froude-Krylov/added-mass, or Munk term -- left at HydroWrenchBreakdown{}'s zero
      // default above.
    }
    return wrench;
  }

  // --- HydroModelMode::kStonefishGeometryDriven: distributed per-patch integration (Section
  // B4) + unsteady Froude-Krylov/Munk inertial terms (Section B6), new for Phase 2.10. ---
  const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
  const Eigen::Vector3d current_world_t = params.currentWorld(t);
  const Eigen::Vector3d v_rel_world = state.lin_vel_world - current_world_t;
  const Eigen::Vector3d v_cog_body = R.transpose() * v_rel_world;
  const Eigen::Vector3d w_body = R.transpose() * state.ang_vel_world;

  Eigen::Vector3d F_total_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d T_total_body = Eigen::Vector3d::Zero();

  // Per-term accumulators (Phase 2.13 HydroWrenchBreakdown), summed alongside F_total_body/
  // T_total_body above without changing their values -- purely additional bookkeeping.
  Eigen::Vector3d acc_f_drag = Eigen::Vector3d::Zero();     // patch normal pressure + skin
  Eigen::Vector3d acc_f_venturi = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_f_squeeze = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_t_drag = Eigen::Vector3d::Zero();     // torque from the 3 terms above

  for (const SurfacePatch & p : geo->patches) {
    const Eigen::Vector3d & r_p = p.centroid_body;
    // Rigid-body kinematics: velocity at this patch relative to the ambient flow, assuming
    // CoM sits at the body-frame origin (rov_wall.xml's <inertial pos="0 0 0">).
    const Eigen::Vector3d v_p_body = v_cog_body + w_body.cross(r_p);
    const double v_n = v_p_body.dot(p.normal_body);
    const Eigen::Vector3d v_t = v_p_body - v_n * p.normal_body;
    const double v_t_norm = v_t.norm();

    // Windward-only distributed quadratic pressure drag (normal direction): bluff-body
    // Cd_normal is defined on single-sided frontal projected area (already encompassing
    // windward stagnation + leeward wake suction), so applying it on both sides double-counts
    // the pressure drag. Leeward patches (v_n <= 0) contribute zero normal pressure drag here.
    Eigen::Vector3d F_drag_n = Eigen::Vector3d::Zero();
    if (v_n > 0.0) {
      F_drag_n = -0.5 * params.rho_water * params.Cd_normal * p.area * v_n * v_n * p.normal_body;
    }
    // Skin friction (tangential direction), both directions.
    const Eigen::Vector3d F_skin_t =
      -0.5 * params.rho_water * params.Cf_tangent * p.area * v_t_norm * v_t;

    Eigen::Vector3d F_suction_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d F_squeeze_n = Eigen::Vector3d::Zero();
    if (p.normal_body.x() < -0.5) {
      // Wall-facing patch: per-patch near-wall Venturi suction + squeeze-film lubrication,
      // using this patch's own X position instead of one whole-body CoM X (Section B4).
      // r_p is already on the hull's exterior surface (e.g. -0.10 m for the front wall-facing
      // plate), so x_world_p is already the true distance from the patch surface to the World
      // X=0 wall -- do NOT subtract params.wall_x_offset a second time.
      const double x_world_p = state.com_pos_world.x() + (R * r_p).x();
      const double h_gap_p = std::max(x_world_p, kWallGapFloorPatchRef);
      // v_t is already the exact 3D tangential velocity on the patch plane (v_p_body minus
      // its normal component) -- squaredNorm() is identical to y^2+z^2 for a flat [-1,0,0]^T
      // patch (rov_wall.xml) and generalizes correctly to curved 3D CAD hull patches whose
      // normal is not axis-aligned (Phase 2.12 Item 5A).
      const double v_tan_sq_p = v_t.squaredNorm();
      const double decay =
        std::exp(-std::max(0.0, h_gap_p - params.h_gap_nominal) / params.h0_decay);
      // n_p = [-1,0,0]^T here, so the +sign below yields a -X (wall-attractive) suction force.
      F_suction_n =
        0.5 * params.rho_water * params.C_venturi * p.area * v_tan_sq_p * decay * p.normal_body;

      // v_n > 0 means this patch is closing in on the wall (moving along body -X); the
      // squeeze-film term below then points along -n_p = +X (resisting approach), and along
      // +n_p = -X when v_n < 0 (separating), resisting separation either way.
      F_squeeze_n = -params.C_squeeze * p.area *
        std::pow(params.h_gap_nominal / h_gap_p, 3) * v_n * decay * p.normal_body;
    }

    const Eigen::Vector3d F_patch = F_drag_n + F_skin_t + F_suction_n + F_squeeze_n;
    F_total_body += F_patch;
    T_total_body += r_p.cross(F_patch);

    acc_f_drag += F_drag_n + F_skin_t;
    acc_f_venturi += F_suction_n;
    acc_f_squeeze += F_squeeze_n;
    acc_t_drag += r_p.cross(F_drag_n + F_skin_t + F_squeeze_n);
  }

  // Low-speed linear viscous damping (whole-body): purely quadratic patch drag has zero
  // derivative at v_rel=0, losing damping stiffness during station-keeping. Quadratic angular
  // damping (Phase 2.13 Fix 3) is also whole-body, not per-patch: patch lever arms are small
  // (r_p ~ 0.15m => r_p^3 ~ 0.0034 m^3), so surface-patch normal drag alone under-predicts
  // bluff-body rotational vortex-shedding damping by an order of magnitude at high angular
  // rates -- matching kParametricLumped's tau_drag_body above.
  const Eigen::Vector3d F_lin_damp_body = -params.D_lin.cwiseProduct(v_cog_body);
  const Eigen::Vector3d T_ang_damp_body = -params.D_ang_lin.cwiseProduct(w_body) -
    params.D_ang_quad.cwiseProduct(w_body.cwiseProduct(w_body.cwiseAbs()));
  F_total_body += F_lin_damp_body;
  T_total_body += T_ang_damp_body;
  acc_f_drag += F_lin_damp_body;
  acc_t_drag += T_ang_damp_body;

  // Wall-proximity added-mass enhancement (hydrodynamic-accuracy audit): geo->added_mass_linear
  // is an unbounded-fluid Lamb-ellipsoid estimate (Section B3) that ignores the rigid World X=0
  // wall's own image-flow reaction, which is exactly the regime this vehicle spends most of its
  // time operating in. Leading-order potential-flow method-of-images correction for a sphere of
  // radius a translating at CoM-to-wall distance h near a rigid plane (Lamb, Hydrodynamics,
  // Art. 98; summarized in Newman, Marine Hydrodynamics (1977) Sec. 4.14): motion normal to the
  // wall is amplified by a factor 1 + 3*(a/2h)^3 + O((a/h)^6); motion parallel to the wall by the
  // much weaker 1 + (3/16)*(a/h)^3 + O(...). Generalized per-axis here using this hull's own
  // Lamb-ellipsoid characteristic semi-axes (geo->bbox_half_extents) in place of a true sphere's
  // single radius -- an engineering approximation consistent with the rest of this file's
  // ellipsoid treatment, NOT an exact solution for a non-spherical box hull; pending validation
  // against CFD/tow-tank data for this specific geometry. Only the translational tensor is
  // corrected here: a consistent rotational (added-inertia) wall correction requires a
  // higher-order multipole image analysis beyond this leading-order linear treatment and is left
  // as a documented future enhancement, not fabricated here. Clamped at
  // kMaxWallAmplification since the inviscid image series itself diverges as h -> 0; the
  // physical near-contact regime is already separately captured by the per-patch squeeze-film
  // term F_squeeze_n above.
  constexpr double kMaxWallAmplification = 6.0;
  const double h_gap_wall =
    std::max(state.com_pos_world.x() - params.wall_x_offset, kWallGapFloorComRef);
  const double a_normal = std::max(geo->bbox_half_extents.x(), 1e-6);
  const double a_tangent =
    std::max(0.5 * (geo->bbox_half_extents.y() + geo->bbox_half_extents.z()), 1e-6);
  const double amp_normal = std::min(
    1.0 + 3.0 * std::pow(a_normal / (2.0 * h_gap_wall), 3), kMaxWallAmplification);
  const double amp_tangent = std::min(
    1.0 + (3.0 / 16.0) * std::pow(a_tangent / h_gap_wall, 3), kMaxWallAmplification);
  Eigen::Matrix3d added_mass_linear_wall = geo->added_mass_linear;
  added_mass_linear_wall(0, 0) *= amp_normal;
  added_mass_linear_wall(1, 1) *= amp_tangent;
  added_mass_linear_wall(2, 2) *= amp_tangent;

  // Unsteady Froude-Krylov (displaced-fluid inertia) + added-mass reaction force: fluid
  // accelerating around the body (gusts) exerts a reaction force proportional to both the
  // displaced volume and the added-mass tensor. This is distinct from the body's own-
  // acceleration added-mass reaction (which mj_step does not know about and which is out of
  // scope here -- it belongs in the controller's effective mass matrix, not as an explicit
  // applied force).
  const Eigen::Vector3d a_fluid_body = R.transpose() * params.currentAccelerationWorld(t);
  const Eigen::Vector3d F_fk_am_body =
    (params.rho_water * params.displaced_volume * Eigen::Matrix3d::Identity() +
     added_mass_linear_wall) *
    a_fluid_body;
  F_total_body += F_fk_am_body;

  // Munk moment: classical rigid-body-in-ideal-fluid moment from added-mass anisotropy
  // coupling relative linear/angular velocity into a destabilizing moment.
  const Eigen::Vector3d T_munk_body = -v_cog_body.cross(added_mass_linear_wall * v_cog_body) -
    w_body.cross(geo->added_mass_angular * w_body);
  T_total_body += T_munk_body;

  // Buoyancy/gravity-bias and the free-surface taper are whole-body effects independent of
  // the distributed-drag model; reuse the same lumped-model terms rather than re-deriving a
  // per-patch buoyancy distribution (out of scope for this phase).
  const double com_z = state.com_pos_world.z();
  const double submerged_ratio = std::clamp(
    (params.z_surface + params.z_transition - com_z) / params.z_transition, 0.0, 1.0);
  const double F_buoy_full = params.rho_water * params.displaced_volume * params.gravity;
  const double added_mass_gravity_bias =
    (params.augmented_mass - params.vehicle_mass) * params.gravity;
  const Eigen::Vector3d F_buoy_world(
    0.0, 0.0, (F_buoy_full + added_mass_gravity_bias) * submerged_ratio);
  const Eigen::Vector3d r_cob_world = R * params.cob_offset_body;
  const Eigen::Vector3d tau_restore_world = r_cob_world.cross(F_buoy_world);

  Wrench wrench;
  wrench.force_world = R * F_total_body * submerged_ratio + F_buoy_world;
  wrench.torque_world = R * T_total_body * submerged_ratio + tau_restore_world;

  if (breakdown) {
    breakdown->f_drag_body = acc_f_drag * submerged_ratio;
    breakdown->f_venturi_body = acc_f_venturi * submerged_ratio;
    breakdown->f_squeeze_body = acc_f_squeeze * submerged_ratio;
    breakdown->f_fk_am_body = F_fk_am_body * submerged_ratio;
    breakdown->f_buoy_world = F_buoy_world;
    breakdown->t_munk_body = T_munk_body * submerged_ratio;
    breakdown->t_drag_body = acc_t_drag * submerged_ratio;
  }
  return wrench;
}

namespace
{

Eigen::Vector3d quatRotate(const double quat_wxyz[4], const Eigen::Vector3d & v)
{
  const Eigen::Quaterniond q(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
  return q * v;
}

void appendPatch(
  std::vector<SurfacePatch> & raw, const double geom_pos[3], const double geom_quat[4],
  const Eigen::Vector3d & centroid_local, const Eigen::Vector3d & normal_local, double area)
{
  if (area <= 0.0) {
    return;
  }
  SurfacePatch patch;
  patch.centroid_body =
    Eigen::Vector3d(geom_pos[0], geom_pos[1], geom_pos[2]) + quatRotate(geom_quat, centroid_local);
  patch.normal_body = quatRotate(geom_quat, normal_local).normalized();
  patch.area = area;
  raw.push_back(patch);
}

void tessellateBox(
  std::vector<SurfacePatch> & raw, const double geom_pos[3], const double geom_quat[4],
  const double half[3], int grid)
{
  const double hx = half[0], hy = half[1], hz = half[2];
  // Face enumeration: (fixed axis, fixed sign, two free axes). For each face, subdivide the
  // free rectangle into grid x grid sub-patches.
  struct Face
  {
    int fixed_axis;
    double sign;
  };
  const Face faces[6] = {{0, 1}, {0, -1}, {1, 1}, {1, -1}, {2, 1}, {2, -1}};
  const double halfv[3] = {hx, hy, hz};
  for (const Face & f : faces) {
    const int a0 = f.fixed_axis;
    const int a1 = (a0 + 1) % 3;
    const int a2 = (a0 + 2) % 3;
    const double extent1 = 2.0 * halfv[a1];
    const double extent2 = 2.0 * halfv[a2];
    const double cell1 = extent1 / grid;
    const double cell2 = extent2 / grid;
    const double area = cell1 * cell2;
    Eigen::Vector3d normal_local = Eigen::Vector3d::Zero();
    normal_local[a0] = f.sign;
    for (int i = 0; i < grid; ++i) {
      const double c1 = -halfv[a1] + (i + 0.5) * cell1;
      for (int j = 0; j < grid; ++j) {
        const double c2 = -halfv[a2] + (j + 0.5) * cell2;
        Eigen::Vector3d centroid_local = Eigen::Vector3d::Zero();
        centroid_local[a0] = f.sign * halfv[a0];
        centroid_local[a1] = c1;
        centroid_local[a2] = c2;
        appendPatch(raw, geom_pos, geom_quat, centroid_local, normal_local, area);
      }
    }
  }
}

void tessellateSphereLike(
  std::vector<SurfacePatch> & raw, const double geom_pos[3], const double geom_quat[4],
  const Eigen::Vector3d & center_local, double a, double b, double c, int lat_bands,
  int lon_segs, double theta_min, double theta_max)
{
  const double dtheta = (theta_max - theta_min) / lat_bands;
  const double dphi = 2.0 * M_PI / lon_segs;
  const double r_eff = std::cbrt(std::max(a * b * c, 1e-12));
  for (int i = 0; i < lat_bands; ++i) {
    const double theta = theta_min + (i + 0.5) * dtheta;
    const double st = std::sin(theta), ct = std::cos(theta);
    for (int j = 0; j < lon_segs; ++j) {
      const double phi = (j + 0.5) * dphi;
      const double cp = std::cos(phi), sp = std::sin(phi);
      const Eigen::Vector3d dir_unit(st * cp, st * sp, ct);  // unit sphere direction
      const Eigen::Vector3d centroid_local =
        center_local + Eigen::Vector3d(a * dir_unit.x(), b * dir_unit.y(), c * dir_unit.z());
      // Ellipsoid outward normal = normalize(x/a^2, y/b^2, z/c^2); reduces to dir_unit for a
      // true sphere (a==b==c).
      Eigen::Vector3d normal_local(
        dir_unit.x() / (a * a), dir_unit.y() / (b * b), dir_unit.z() / (c * c));
      const double area = r_eff * r_eff * std::max(st, 1e-6) * dtheta * dphi;
      appendPatch(raw, geom_pos, geom_quat, centroid_local, normal_local, area);
    }
  }
}

void tessellateCylinderLateral(
  std::vector<SurfacePatch> & raw, const double geom_pos[3], const double geom_quat[4],
  double radius, double half_height, int height_bands, int circ_segs)
{
  const double dz = 2.0 * half_height / height_bands;
  const double dphi = 2.0 * M_PI / circ_segs;
  for (int i = 0; i < height_bands; ++i) {
    const double z = -half_height + (i + 0.5) * dz;
    for (int j = 0; j < circ_segs; ++j) {
      const double phi = (j + 0.5) * dphi;
      const Eigen::Vector3d normal_local(std::cos(phi), std::sin(phi), 0.0);
      const Eigen::Vector3d centroid_local(radius * normal_local.x(), radius * normal_local.y(), z);
      const double area = radius * dphi * dz;
      appendPatch(raw, geom_pos, geom_quat, centroid_local, normal_local, area);
    }
  }
}

void tessellateDisk(
  std::vector<SurfacePatch> & raw, const double geom_pos[3], const double geom_quat[4],
  double radius, double z, double normal_sign, int wedges)
{
  const double dphi = 2.0 * M_PI / wedges;
  const double area = M_PI * radius * radius / wedges;
  const Eigen::Vector3d normal_local(0.0, 0.0, normal_sign);
  for (int j = 0; j < wedges; ++j) {
    const double phi = (j + 0.5) * dphi;
    const double rc = radius * 2.0 / 3.0;  // centroid of a circular wedge sector, approx
    const Eigen::Vector3d centroid_local(rc * std::cos(phi), rc * std::sin(phi), z);
    appendPatch(raw, geom_pos, geom_quat, centroid_local, normal_local, area);
  }
}

void tessellateMesh(
  std::vector<SurfacePatch> & raw, const mjModel * m, int geom_id, const double geom_pos[3],
  const double geom_quat[4])
{
  const int dataid = m->geom_dataid[geom_id];
  if (dataid < 0) {
    return;
  }
  const int vertadr = m->mesh_vertadr[dataid];
  const int faceadr = m->mesh_faceadr[dataid];
  const int facenum = m->mesh_facenum[dataid];
  for (int f = 0; f < facenum; ++f) {
    const int i0 = m->mesh_face[3 * (faceadr + f) + 0];
    const int i1 = m->mesh_face[3 * (faceadr + f) + 1];
    const int i2 = m->mesh_face[3 * (faceadr + f) + 2];
    const Eigen::Vector3d v0(
      m->mesh_vert[3 * (vertadr + i0) + 0], m->mesh_vert[3 * (vertadr + i0) + 1],
      m->mesh_vert[3 * (vertadr + i0) + 2]);
    const Eigen::Vector3d v1(
      m->mesh_vert[3 * (vertadr + i1) + 0], m->mesh_vert[3 * (vertadr + i1) + 1],
      m->mesh_vert[3 * (vertadr + i1) + 2]);
    const Eigen::Vector3d v2(
      m->mesh_vert[3 * (vertadr + i2) + 0], m->mesh_vert[3 * (vertadr + i2) + 1],
      m->mesh_vert[3 * (vertadr + i2) + 2]);
    const Eigen::Vector3d cross = (v1 - v0).cross(v2 - v0);
    const double area = 0.5 * cross.norm();
    if (area <= 0.0) {
      continue;
    }
    const Eigen::Vector3d centroid_local = (v0 + v1 + v2) / 3.0;
    const Eigen::Vector3d normal_local = cross.normalized();
    appendPatch(raw, geom_pos, geom_quat, centroid_local, normal_local, area);
  }
}

// Maps a unit-ish normal to one of 6 dominant-axis-direction buckets (+X,-X,+Y,-Y,+Z,-Z),
// indexed by the axis of largest |component| and its sign. Folded into decimatePatches()'s
// voxel hash key so spatially-coincident but oppositely-facing patches (e.g. all 48 patches of
// one closed contact-pad sphere, or two orthogonal box faces sharing a voxel) are never merged
// into the same bucket -- merging them would sum their area-weighted normals to near zero
// (closed-surface normals integrate to the zero vector) while still dumping their full area
// onto whichever direction survives.
int normalDirIndex(const Eigen::Vector3d & n)
{
  int axis = 0;
  double max_abs = std::abs(n.x());
  if (std::abs(n.y()) > max_abs) {
    axis = 1;
    max_abs = std::abs(n.y());
  }
  if (std::abs(n.z()) > max_abs) {
    axis = 2;
  }
  return axis * 2 + (n[axis] >= 0.0 ? 0 : 1);
}

// Decimates `raw` to at most kMaxPatches entries via area-weighted spatial-grid bucketing.
// No-op (returns raw unchanged) if already within budget.
constexpr int kMaxPatches = 256;

std::vector<SurfacePatch> decimatePatches(const std::vector<SurfacePatch> & raw)
{
  if (static_cast<int>(raw.size()) <= kMaxPatches) {
    return raw;
  }

  Eigen::Vector3d aabb_min = raw.front().centroid_body;
  Eigen::Vector3d aabb_max = raw.front().centroid_body;
  for (const auto & p : raw) {
    aabb_min = aabb_min.cwiseMin(p.centroid_body);
    aabb_max = aabb_max.cwiseMax(p.centroid_body);
  }
  const Eigen::Vector3d extent = (aabb_max - aabb_min).cwiseMax(1e-6);
  // 3^3 spatial cells x 6 normal-direction buckets = 162 <= kMaxPatches, leaving headroom
  // versus the 256 budget while keeping every bucket's occupants normal-direction-consistent.
  constexpr int kGridN = 3;
  const Eigen::Vector3d cell_size = extent / static_cast<double>(kGridN);

  struct Bucket
  {
    double area_sum = 0.0;
    Eigen::Vector3d centroid_weighted_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_weighted_sum = Eigen::Vector3d::Zero();
    double largest_area = 0.0;
    Eigen::Vector3d largest_normal = Eigen::Vector3d::Zero();
  };
  std::unordered_map<int64_t, Bucket> buckets;
  buckets.reserve(static_cast<size_t>(kGridN) * kGridN * kGridN);

  for (const auto & p : raw) {
    const Eigen::Vector3d rel = p.centroid_body - aabb_min;
    const int ix = std::clamp(static_cast<int>(rel.x() / cell_size.x()), 0, kGridN - 1);
    const int iy = std::clamp(static_cast<int>(rel.y() / cell_size.y()), 0, kGridN - 1);
    const int iz = std::clamp(static_cast<int>(rel.z() / cell_size.z()), 0, kGridN - 1);
    const int ndir = normalDirIndex(p.normal_body);
    const int64_t key =
      ((static_cast<int64_t>(ix) * kGridN + iy) * kGridN + iz) * 6 + ndir;
    Bucket & b = buckets[key];
    b.area_sum += p.area;
    b.centroid_weighted_sum += p.area * p.centroid_body;
    b.normal_weighted_sum += p.area * p.normal_body;
    if (p.area > b.largest_area) {
      b.largest_area = p.area;
      b.largest_normal = p.normal_body;
    }
  }

  std::vector<SurfacePatch> decimated;
  decimated.reserve(buckets.size());
  for (const auto & kv : buckets) {
    const Bucket & b = kv.second;
    SurfacePatch merged;
    merged.area = b.area_sum;
    merged.centroid_body = b.centroid_weighted_sum / b.area_sum;
    const double normal_mag = b.normal_weighted_sum.norm();
    merged.normal_body =
      normal_mag > 1e-6 ? (b.normal_weighted_sum / normal_mag) : b.largest_normal.normalized();
    decimated.push_back(merged);
  }
  return decimated;
}

// Tan-substitution quadrature for the classical ellipsoid added-mass integral
// alpha_i = a*b*c * Integral_0^inf dlambda / ((semi_i^2+lambda) * Delta(lambda)), Delta =
// sqrt((a^2+lambda)(b^2+lambda)(c^2+lambda)). The substitution lambda = tan(theta) maps
// [0,inf) to theta in [0,pi/2) with no singular endpoint under the midpoint rule, avoiding
// the need for a log-spaced grid. Run once per model at buildFromMjModel() time, never in
// the 500Hz hot loop -- not performance-sensitive.
double ellipsoidAlpha(double semi, double a, double b, double c)
{
  constexpr int kN = 4000;
  const double max_axis = std::max({a, b, c});
  const double lambda_max = 1000.0 * max_axis * max_axis;
  const double theta_max = std::atan(lambda_max);
  const double dtheta = theta_max / kN;
  double integral = 0.0;
  for (int i = 0; i < kN; ++i) {
    const double theta = (i + 0.5) * dtheta;
    const double lambda = std::tan(theta);
    const double cos_theta = std::cos(theta);
    const double dlambda_dtheta = 1.0 / (cos_theta * cos_theta);
    const double delta =
      std::sqrt((a * a + lambda) * (b * b + lambda) * (c * c + lambda));
    integral += dlambda_dtheta / ((semi * semi + lambda) * delta) * dtheta;
  }
  return a * b * c * integral;
}

}  // namespace

HydroGeometryModel HydroGeometryModel::buildFromMjModel(
  const mjModel * m, int root_body_id, double rho_water)
{
  HydroGeometryModel geo;
  std::vector<SurfacePatch> raw;

  // Welded-subtree membership + composed root-frame transform (CLAUDE.md Phase 2.12 Section
  // 2): body b belongs to root_body_id's rigid hull subtree iff b == root_body_id, or its
  // parent already belongs to the subtree AND b itself carries zero joints (body_jntnum == 0,
  // i.e. rigidly welded to that parent -- NOT merely "reachable", since a non-welded
  // intermediate body breaks the rigid chain for every one of its descendants too). MuJoCo
  // guarantees body_parentid[b] < b for every b > 0, so a single forward pass over increasing
  // body index is sufficient -- no need to walk the tree recursively or sort it first.
  std::vector<bool> is_rigid(static_cast<size_t>(m->nbody), false);
  std::vector<Eigen::Vector3d> pos_in_root(static_cast<size_t>(m->nbody), Eigen::Vector3d::Zero());
  std::vector<Eigen::Quaterniond> quat_in_root(
    static_cast<size_t>(m->nbody), Eigen::Quaterniond::Identity());
  is_rigid[static_cast<size_t>(root_body_id)] = true;
  for (int b = 0; b < m->nbody; ++b) {
    if (b == root_body_id || is_rigid[static_cast<size_t>(b)]) {
      continue;  // root itself, already resolved above
    }
    const int parent = m->body_parentid[b];
    if (parent < 0 || !is_rigid[static_cast<size_t>(parent)] || m->body_jntnum[b] != 0) {
      continue;  // not reachable, or this body itself has a non-welded joint to its parent
    }
    is_rigid[static_cast<size_t>(b)] = true;
    const Eigen::Vector3d local_pos(
      m->body_pos[3 * b + 0], m->body_pos[3 * b + 1], m->body_pos[3 * b + 2]);
    const Eigen::Quaterniond local_quat(
      m->body_quat[4 * b + 0], m->body_quat[4 * b + 1], m->body_quat[4 * b + 2],
      m->body_quat[4 * b + 3]);
    pos_in_root[static_cast<size_t>(b)] =
      pos_in_root[static_cast<size_t>(parent)] +
      quat_in_root[static_cast<size_t>(parent)] * local_pos;
    quat_in_root[static_cast<size_t>(b)] = quat_in_root[static_cast<size_t>(parent)] * local_quat;
  }

  Eigen::Vector3d aabb_min = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d aabb_max = Eigen::Vector3d::Constant(-1e9);
  bool any_geom = false;

  for (int g = 0; g < m->ngeom; ++g) {
    const int gbody = m->geom_bodyid[g];
    if (!is_rigid[static_cast<size_t>(gbody)]) {
      continue;
    }
    // The 4 small rubber contact-pad spheres are modeled by MuJoCo's own Kelvin-Voigt contact
    // solver (CLAUDE.md Sec.4.2), not by this fluid-surface hydrodynamic model -- tessellating
    // them here would contaminate the hull's patch budget and skew the front plate's normals/
    // area via decimatePatches()'s voxel merging (Phase 2.11 Section 1 fix).
    const char * gname = mj_id2name(m, mjOBJ_GEOM, g);
    if (gname != nullptr && std::strncmp(gname, "pad_", 4) == 0) {
      continue;
    }
    any_geom = true;

    // Compose this geom's own body-local pose with its body's accumulated welded-chain
    // transform, so every patch below ends up expressed in root_body_id's frame regardless of
    // how many welded child bodies it had to pass through (identity composition when gbody ==
    // root_body_id, reproducing Phase 2.11's exact byte-identical output for rov_wall.xml).
    const Eigen::Vector3d geom_local_pos(
      m->geom_pos[3 * g + 0], m->geom_pos[3 * g + 1], m->geom_pos[3 * g + 2]);
    const Eigen::Quaterniond geom_local_quat(
      m->geom_quat[4 * g + 0], m->geom_quat[4 * g + 1], m->geom_quat[4 * g + 2],
      m->geom_quat[4 * g + 3]);
    const Eigen::Vector3d composed_pos_vec =
      pos_in_root[static_cast<size_t>(gbody)] +
      quat_in_root[static_cast<size_t>(gbody)] * geom_local_pos;
    const Eigen::Quaterniond composed_quat_eigen =
      quat_in_root[static_cast<size_t>(gbody)] * geom_local_quat;
    const double composed_pos[3] = {
      composed_pos_vec.x(), composed_pos_vec.y(), composed_pos_vec.z()};
    const double composed_quat[4] = {
      composed_quat_eigen.w(), composed_quat_eigen.x(), composed_quat_eigen.y(),
      composed_quat_eigen.z()};
    const double * geom_pos = composed_pos;
    const double * geom_quat = composed_quat;
    const double * size = &m->geom_size[3 * g];
    const double rbound = m->geom_rbound[g];
    const Eigen::Vector3d center(geom_pos[0], geom_pos[1], geom_pos[2]);

    // Per-geom-type half-extent in the geom's own local frame (NOT geom_rbound's
    // circumscribed-sphere radius, which would isotropically inflate every axis to the
    // diagonal length and erase the hull's real anisotropy -- e.g. a 0.1x0.3x0.2 box's rbound
    // is sqrt(0.1^2+0.3^2+0.2^2)=0.374 on all 3 axes, losing the box's actual shape). Falls
    // back to the rbound sphere only for geom types with no simple closed-form half-extent
    // (mesh, etc).
    Eigen::Vector3d half_extent_local;
    switch (m->geom_type[g]) {
      case mjGEOM_BOX:
      case mjGEOM_ELLIPSOID:
        half_extent_local = Eigen::Vector3d(size[0], size[1], size[2]);
        break;
      case mjGEOM_SPHERE:
        half_extent_local = Eigen::Vector3d::Constant(size[0]);
        break;
      case mjGEOM_CYLINDER:
        half_extent_local = Eigen::Vector3d(size[0], size[0], size[1]);
        break;
      case mjGEOM_CAPSULE:
        half_extent_local = Eigen::Vector3d(size[0], size[0], size[0] + size[1]);
        break;
      default:
        half_extent_local = Eigen::Vector3d::Constant(rbound);
        break;
    }
    // Conservative AABB of a rotated box: |R| * half_extent (elementwise abs of the rotation
    // matrix applied to the local half-extent), reducing to half_extent_local unchanged when
    // geom_quat is identity (true for every geom in the current rov_wall.xml).
    const Eigen::Quaterniond geom_q(geom_quat[0], geom_quat[1], geom_quat[2], geom_quat[3]);
    const Eigen::Vector3d half_extent_body =
      geom_q.toRotationMatrix().cwiseAbs() * half_extent_local;
    aabb_min = aabb_min.cwiseMin(center - half_extent_body);
    aabb_max = aabb_max.cwiseMax(center + half_extent_body);

    switch (m->geom_type[g]) {
      case mjGEOM_BOX: {
        const double half[3] = {size[0], size[1], size[2]};
        tessellateBox(raw, geom_pos, geom_quat, half, 4);
        break;
      }
      case mjGEOM_SPHERE: {
        const double r = size[0];
        tessellateSphereLike(
          raw, geom_pos, geom_quat, Eigen::Vector3d::Zero(), r, r, r, 6, 8, 0.0, M_PI);
        break;
      }
      case mjGEOM_ELLIPSOID: {
        tessellateSphereLike(
          raw, geom_pos, geom_quat, Eigen::Vector3d::Zero(), size[0], size[1], size[2], 6, 8,
          0.0, M_PI);
        break;
      }
      case mjGEOM_CYLINDER: {
        const double r = size[0];
        const double hh = size[1];
        tessellateCylinderLateral(raw, geom_pos, geom_quat, r, hh, 4, 8);
        tessellateDisk(raw, geom_pos, geom_quat, r, hh, 1.0, 4);
        tessellateDisk(raw, geom_pos, geom_quat, r, -hh, -1.0, 4);
        break;
      }
      case mjGEOM_CAPSULE: {
        const double r = size[0];
        const double hh = size[1];
        tessellateCylinderLateral(raw, geom_pos, geom_quat, r, hh, 4, 8);
        // Hemispherical caps: restrict the sphere lat/long grid to [0,pi/2] per cap,
        // centered at each end of the cylindrical core.
        tessellateSphereLike(
          raw, geom_pos, geom_quat, Eigen::Vector3d(0, 0, hh), r, r, r, 3, 8, 0.0, M_PI / 2.0);
        tessellateSphereLike(
          raw, geom_pos, geom_quat, Eigen::Vector3d(0, 0, -hh), r, r, r, 3, 8, M_PI / 2.0, M_PI);
        break;
      }
      case mjGEOM_MESH: {
        tessellateMesh(raw, m, g, geom_pos, geom_quat);
        break;
      }
      default:
        // Other geom types (plane, hfield, etc) do not contribute hydrodynamic surface area
        // for this robot's hull -- silently skipped.
        break;
    }
  }

  if (!any_geom || raw.empty()) {
    return geo;  // degenerate: no geoms on this body, leave everything zeroed
  }

  // CoM-relative patch coordinates (CLAUDE.md Phase 2.13 Fix 2): every patch above was
  // tessellated relative to root_body_id's own BODY frame origin (xpos), composed through
  // geom_pos/body_pos -- which coincides with its CoM (xipos) only when the MJCF declares
  // <inertial pos="0 0 0"/> (true for rov_wall.xml today). FossenHydrodynamics::compute()
  // always evaluates kinematics/lever-arms relative to the CoM (d->xipos), so shift every patch
  // centroid and the AABB by -root_ipos here, once, so a future model with a non-zero <inertial
  // pos> never silently mis-locates its patches relative to the CoM it's actually levered about.
  const Eigen::Vector3d root_ipos(
    m->body_ipos[3 * root_body_id + 0], m->body_ipos[3 * root_body_id + 1],
    m->body_ipos[3 * root_body_id + 2]);
  for (auto & p : raw) {
    p.centroid_body -= root_ipos;
  }
  aabb_min -= root_ipos;
  aabb_max -= root_ipos;

  geo.patches = decimatePatches(raw);
  geo.bbox_half_extents = (aabb_max - aabb_min) / 2.0;

  // Lamb's classical ellipsoid added-mass approximation (Section B3): scaled by the caller-
  // supplied rho_water (hydrodynamic-accuracy audit fix) instead of a hardcoded 1000.0 kg/m^3,
  // so this tensor stays physically consistent with FossenHydrodynamics::compute()'s live
  // params.rho_water-scaled Froude-Krylov term -- both now trace back to the same one canonical
  // water density the caller configured, rather than silently disagreeing if that density is
  // ever overridden away from the historical 1000.0 default.
  const double a = std::max(geo.bbox_half_extents.x(), 1e-6);
  const double b = std::max(geo.bbox_half_extents.y(), 1e-6);
  const double c = std::max(geo.bbox_half_extents.z(), 1e-6);
  const double alpha0 = ellipsoidAlpha(a, a, b, c);
  const double beta0 = ellipsoidAlpha(b, a, b, c);
  const double gamma0 = ellipsoidAlpha(c, a, b, c);
  const double k1 = alpha0 / (2.0 - alpha0);
  const double k2 = beta0 / (2.0 - beta0);
  const double k3 = gamma0 / (2.0 - gamma0);
  const double displaced_mass = rho_water * (4.0 / 3.0) * M_PI * a * b * c;
  const double m_A_x = k1 * displaced_mass;
  const double m_A_y = k2 * displaced_mass;
  const double m_A_z = k3 * displaced_mass;
  geo.added_mass_linear = Eigen::Vector3d(m_A_x, m_A_y, m_A_z).asDiagonal();

  // Closed-form triaxial-ellipsoid rotational added inertia: Fossen (2011), Handbook of Marine
  // Craft Hydrodynamics and Motion Control, Eq. (2.159)-(2.161) (kappa1/2/3 above are exactly
  // alpha0/beta0/gamma0; displaced_mass is exactly rho*V). This replaces the old
  // 0.2*m_A_i*(sum of the other two semi-axes^2) heuristic, which the previous revision of this
  // file documented as an explicit placeholder "pending future tuning" and which does not itself
  // vanish at a=b=c.
  //
  // NOTE on a transcription-sign correction: Eq.(2.161) as it appears in at least one
  // reproduction of the textbook writes the I_A,22 term with (a^2-c^2) paired against
  // (kappa1-kappa3) in the denominator's linear term -- this is NOT internally consistent with
  // the shared cyclic pattern of the I_A,11/I_A,33 terms (both of which pair "(axis_j^2-axis_k^2)"
  // with "(kappa_j-kappa_k)" using the SAME (j,k) order in both the linear term and the implicit
  // numerator order) and was verified numerically to yield a NEGATIVE (unphysical -- added
  // rotational inertia must be >= 0) value for this hull's actual bbox (a=0.1,b=0.3,c=0.2 m).
  // The form used below instead continues the cyclic pattern established by I_A,11
  // ((j,k)=(2,3)) and I_A,33 ((j,k)=(1,2)) to I_A,22 with (j,k)=(3,1), which is both internally
  // consistent and numerically verified to give a strictly positive value for this geometry.
  //
  // Each I_A,ii is a 0/0 indeterminate exactly when its OTHER two semi-axes coincide (the body
  // is then a spheroid and rotation about the remaining, distinct axis induces zero fluid
  // motion by the same ideal-fluid symmetry argument as the sphere case) -- guarded per-axis
  // below rather than relying on floating-point evaluation of the indeterminate form.
  constexpr double kDegenerateRelTol = 1e-6;
  const bool bc_equal = std::abs(b - c) < kDegenerateRelTol * std::max(b, c);
  const bool ac_equal = std::abs(a - c) < kDegenerateRelTol * std::max(a, c);
  const bool ab_equal = std::abs(a - b) < kDegenerateRelTol * std::max(a, b);
  const double I_A11 = bc_equal ? 0.0 :
    (displaced_mass / 5.0) * (b * b - c * c) * (b * b - c * c) * (gamma0 - beta0) /
    (2.0 * (b * b - c * c) + (b * b + c * c) * (beta0 - gamma0));
  const double I_A22 = ac_equal ? 0.0 :
    (displaced_mass / 5.0) * (a * a - c * c) * (a * a - c * c) * (gamma0 - alpha0) /
    (2.0 * (a * a - c * c) + (a * a + c * c) * (alpha0 - gamma0));
  const double I_A33 = ab_equal ? 0.0 :
    (displaced_mass / 5.0) * (a * a - b * b) * (a * a - b * b) * (beta0 - alpha0) /
    (2.0 * (a * a - b * b) + (a * a + b * b) * (alpha0 - beta0));
  geo.added_mass_angular = Eigen::Vector3d(I_A11, I_A22, I_A33).asDiagonal();

  return geo;
}

}  // namespace adhere_mujoco_hardware
