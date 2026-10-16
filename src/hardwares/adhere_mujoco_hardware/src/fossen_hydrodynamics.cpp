#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace adhere_mujoco_hardware
{

Wrench FossenHydrodynamics::compute(
  const RigidBodyState & state, double t, const HydroParams & params, HydroModelMode mode,
  const HydroGeometryModel * geo)
{
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
    const double h_gap = std::max(x_com - params.wall_x_offset, 0.001);
    const double v_tangent_sq = v_rel_body.y() * v_rel_body.y() + v_rel_body.z() * v_rel_body.z();
    const double F_suction_x = -0.5 * params.rho_water * params.C_venturi * params.A_plate *
      v_tangent_sq * std::exp(-(h_gap - params.h_gap_nominal) / params.h0_decay);
    const Eigen::Vector3d F_suction_body(F_suction_x, 0.0, 0.0);
    const Eigen::Vector3d F_suction_world = R * F_suction_body;

    Wrench wrench;
    wrench.force_world =
      F_drag_world * submerged_ratio + F_buoy_world + F_suction_world * submerged_ratio;
    wrench.torque_world = tau_drag_world * submerged_ratio + tau_restore_world;
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

  for (const SurfacePatch & p : geo->patches) {
    const Eigen::Vector3d & r_p = p.centroid_body;
    // Rigid-body kinematics: velocity at this patch relative to the ambient flow, assuming
    // CoM sits at the body-frame origin (rov_wall.xml's <inertial pos="0 0 0">).
    const Eigen::Vector3d v_p_body = v_cog_body + w_body.cross(r_p);
    const double v_n = v_p_body.dot(p.normal_body);
    const Eigen::Vector3d v_t = v_p_body - v_n * p.normal_body;
    const double v_t_norm = v_t.norm();

    // Distributed quadratic pressure drag (normal direction).
    const Eigen::Vector3d F_drag_n =
      -0.5 * params.rho_water * params.Cd_normal * p.area * v_n * std::abs(v_n) * p.normal_body;
    // Skin friction (tangential direction).
    const Eigen::Vector3d F_skin_t =
      -0.5 * params.rho_water * params.Cf_tangent * p.area * v_t_norm * v_t;

    Eigen::Vector3d F_suction_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d F_squeeze_n = Eigen::Vector3d::Zero();
    if (p.normal_body.x() < -0.5) {
      // Wall-facing patch: per-patch near-wall Venturi suction + squeeze-film lubrication,
      // using this patch's own X position instead of one whole-body CoM X (Section B4).
      const double x_world_p = state.com_pos_world.x() + (R * r_p).x();
      const double h_gap_p = std::max(x_world_p - params.wall_x_offset, 0.001);
      const double v_tan_sq_p = v_t.y() * v_t.y() + v_t.z() * v_t.z();
      const double decay = std::exp(-(h_gap_p - params.h_gap_nominal) / params.h0_decay);
      F_suction_n =
        -0.5 * params.rho_water * params.C_venturi * p.area * v_tan_sq_p * decay * p.normal_body;

      // Positive when this patch is closing in on the wall (moving along body -X).
      const double v_n_wall_approach = -v_n;
      F_squeeze_n = -params.C_squeeze * p.area / (h_gap_p * h_gap_p * h_gap_p) *
        v_n_wall_approach * decay * p.normal_body;
    }

    const Eigen::Vector3d F_patch = F_drag_n + F_skin_t + F_suction_n + F_squeeze_n;
    F_total_body += F_patch;
    T_total_body += r_p.cross(F_patch);
  }

  // Unsteady Froude-Krylov (ambient fluid-acceleration reaction) force: fluid accelerating
  // around the body (gusts) exerts a reaction force proportional to the added-mass tensor.
  // This is distinct from the body's own-acceleration added-mass reaction (which mj_step
  // does not know about and which is out of scope here -- it belongs in the controller's
  // effective mass matrix, not as an explicit applied force).
  const Eigen::Vector3d a_fluid_body = R.transpose() * params.currentAccelerationWorld(t);
  F_total_body += geo->added_mass_linear * a_fluid_body;

  // Munk moment: classical rigid-body-in-ideal-fluid moment from added-mass anisotropy
  // coupling relative linear velocity into a destabilizing moment.
  T_total_body += v_cog_body.cross(geo->added_mass_linear * v_cog_body);

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
  constexpr int kGridN = 6;  // 6^3 = 216 <= 256
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
    const int64_t key =
      (static_cast<int64_t>(ix) * kGridN + iy) * kGridN + iz;
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

HydroGeometryModel HydroGeometryModel::buildFromMjModel(const mjModel * m, int root_body_id)
{
  HydroGeometryModel geo;
  std::vector<SurfacePatch> raw;

  // rov_wall.xml confirmed: root_body_id ("rov_base") has no welded child bodies -- all pad
  // and chassis geoms are direct geoms of this one body. TODO: if a future MJCF adds welded
  // child bodies, extend this filter to recurse via m->body_parentid.
  Eigen::Vector3d aabb_min = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d aabb_max = Eigen::Vector3d::Constant(-1e9);
  bool any_geom = false;

  for (int g = 0; g < m->ngeom; ++g) {
    if (m->geom_bodyid[g] != root_body_id) {
      continue;
    }
    any_geom = true;
    const double * geom_pos = &m->geom_pos[3 * g];
    const double * geom_quat = &m->geom_quat[4 * g];
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

  geo.patches = decimatePatches(raw);
  geo.bbox_half_extents = (aabb_max - aabb_min) / 2.0;

  // Lamb's classical ellipsoid added-mass approximation (Section B3): assumed rho_water =
  // 1000.0 kg/m^3, matching the only value ever used by CalmWaterPlugin/
  // StationKeepingGustPlugin -- HydroGeometryModel is built once at configure() time, before
  // HydroParams selection, so it cannot read a per-plugin rho_water; if a future plugin ever
  // overrides rho_water, revisit this assumption.
  constexpr double kRhoWater = 1000.0;
  const double a = std::max(geo.bbox_half_extents.x(), 1e-6);
  const double b = std::max(geo.bbox_half_extents.y(), 1e-6);
  const double c = std::max(geo.bbox_half_extents.z(), 1e-6);
  const double alpha0 = ellipsoidAlpha(a, a, b, c);
  const double beta0 = ellipsoidAlpha(b, a, b, c);
  const double gamma0 = ellipsoidAlpha(c, a, b, c);
  const double k1 = alpha0 / (2.0 - alpha0);
  const double k2 = beta0 / (2.0 - beta0);
  const double k3 = gamma0 / (2.0 - gamma0);
  const double displaced_mass = kRhoWater * (4.0 / 3.0) * M_PI * a * b * c;
  const double m_A_x = k1 * displaced_mass;
  const double m_A_y = k2 * displaced_mass;
  const double m_A_z = k3 * displaced_mass;
  geo.added_mass_linear = Eigen::Vector3d(m_A_x, m_A_y, m_A_z).asDiagonal();
  geo.added_mass_angular = Eigen::Vector3d(
    0.2 * m_A_x * (b * b + c * c), 0.2 * m_A_y * (a * a + c * c),
    0.2 * m_A_z * (a * a + b * b))
    .asDiagonal();

  return geo;
}

}  // namespace adhere_mujoco_hardware
