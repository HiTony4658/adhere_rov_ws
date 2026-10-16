#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using adhere_mujoco_hardware::FossenHydrodynamics;
using adhere_mujoco_hardware::HydroGeometryModel;
using adhere_mujoco_hardware::HydroModelMode;
using adhere_mujoco_hardware::HydroParams;
using adhere_mujoco_hardware::HydroWrenchBreakdown;
using adhere_mujoco_hardware::RigidBodyState;
using adhere_mujoco_hardware::Wrench;

namespace
{
constexpr double kTol = 1e-9;
}

// kParametricLumped must stay byte-identical to the Phase 2 Python-reference-matched
// implementation. These two cases are hand-derived analytically (not reproductions of the
// original random PROGRESS.md vectors, whose inputs were not recorded) so they can be
// independently re-verified by inspection; they exercise the zero-velocity/zero-suction path
// and the drag+suction+buoyancy path respectively.
TEST(FossenHydrodynamicsParity, AtRestSubmerged)
{
  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.2, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.ang_vel_world = Eigen::Vector3d::Zero();
  state.lin_vel_world = Eigen::Vector3d::Zero();

  HydroParams params;  // calm-water defaults
  const Wrench w = FossenHydrodynamics::compute(state, 0.0, params);

  // submerged_ratio = clamp((1.0+0.4-0.5)/0.4, 0, 1) = 1.0 (fully submerged)
  // F_buoy = rho*V*g + (m_aug-m)*g = 1000*0.0202*9.81 + 6*9.81 = 198.162 + 58.86 = 257.022
  EXPECT_NEAR(w.force_world.x(), 0.0, kTol);
  EXPECT_NEAR(w.force_world.y(), 0.0, kTol);
  EXPECT_NEAR(w.force_world.z(), 257.022, kTol);
  EXPECT_NEAR(w.torque_world.x(), 0.0, kTol);
  EXPECT_NEAR(w.torque_world.y(), 0.0, kTol);
  EXPECT_NEAR(w.torque_world.z(), 0.0, kTol);
}

TEST(FossenHydrodynamicsParity, CrossflowDragAndSuction)
{
  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.15, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.ang_vel_world = Eigen::Vector3d::Zero();
  state.lin_vel_world = Eigen::Vector3d(0.0, 0.2, 0.0);

  HydroParams params;  // calm-water defaults
  const Wrench w = FossenHydrodynamics::compute(state, 0.0, params);

  // F_drag_body = -(D_lin*v + D_quad*v*|v|): y = -(20*0.2 + 50*0.2*0.2) = -6
  // h_gap = max(0.15-0.1, 0.001) = 0.05; v_tangent_sq = 0.2^2 = 0.04
  // F_suction_x = -0.5*rho*C_venturi*A_plate*v_tangent_sq*exp(-(h_gap-h_gap_nominal)/h0_decay)
  //             = -0.5*1000*0.45*0.24*0.04*exp(-(0.05-0.025)/0.05) = -2.16*exp(-0.5)
  const double F_suction_x = -2.16 * std::exp(-0.5);
  EXPECT_NEAR(w.force_world.x(), F_suction_x, kTol);
  EXPECT_NEAR(w.force_world.y(), -6.0, kTol);
  EXPECT_NEAR(w.force_world.z(), 257.022, kTol);
  EXPECT_NEAR(w.torque_world.x(), 0.0, kTol);
  EXPECT_NEAR(w.torque_world.y(), 0.0, kTol);
  EXPECT_NEAR(w.torque_world.z(), 0.0, kTol);
}

// kStonefishGeometryDriven falling back to nullptr geo must reproduce kParametricLumped
// exactly -- the defensive fallback documented in compute()'s signature.
TEST(FossenHydrodynamicsParity, StonefishWithNullGeoFallsBackToLumped)
{
  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.2, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.ang_vel_world = Eigen::Vector3d::Zero();
  state.lin_vel_world = Eigen::Vector3d::Zero();

  HydroParams params;
  const Wrench lumped = FossenHydrodynamics::compute(state, 0.0, params);
  const Wrench fallback = FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, nullptr);

  EXPECT_NEAR(lumped.force_world.x(), fallback.force_world.x(), kTol);
  EXPECT_NEAR(lumped.force_world.y(), fallback.force_world.y(), kTol);
  EXPECT_NEAR(lumped.force_world.z(), fallback.force_world.z(), kTol);
}

class HydroGeometryModelFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("simplified_v1_description");
    const std::string mjcf_path = pkg_share + "/mjcf/rov_wall.xml";
    char error[1000] = "";
    model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << "Failed to load " << mjcf_path << ": " << error;
    body_id_ = mj_name2id(model_, mjOBJ_BODY, "rov_base");
    ASSERT_GE(body_id_, 0);
  }
  void TearDown() override
  {
    if (model_) {
      mj_deleteModel(model_);
    }
  }
  mjModel * model_ = nullptr;
  int body_id_ = -1;
};

TEST_F(HydroGeometryModelFixture, StructuralInvariants)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  EXPECT_GT(geo.patches.size(), 0u);
  EXPECT_LE(geo.patches.size(), 256u);

  for (const auto & patch : geo.patches) {
    EXPECT_GT(patch.area, 0.0);
    EXPECT_NEAR(patch.normal_body.norm(), 1.0, 1e-6);
  }

  // rov_wall.xml's chassis_geom half-extents are (0.1, 0.3, 0.2); the 4 small corner pad
  // spheres barely extend the AABB beyond that.
  EXPECT_NEAR(geo.bbox_half_extents.x(), 0.1, 0.02);
  EXPECT_NEAR(geo.bbox_half_extents.y(), 0.3, 0.02);
  EXPECT_NEAR(geo.bbox_half_extents.z(), 0.2, 0.02);

  // Lamb-ellipsoid added-mass tensors: diagonal, positive, same order of magnitude as the
  // existing lumped added mass (augmented_mass - vehicle_mass = 6 kg).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i != j) {
        EXPECT_NEAR(geo.added_mass_linear(i, j), 0.0, 1e-9);
        EXPECT_NEAR(geo.added_mass_angular(i, j), 0.0, 1e-9);
      }
    }
    // Loose sanity band, not a precise physical claim: a Lamb-ellipsoid coefficient along the
    // hull's thinnest axis (X, half-extent 0.1m vs 0.3/0.2m) legitimately exceeds the
    // equivalent-sphere displaced mass (~25 kg) as the shape flattens -- classic oblate-body
    // added-mass behavior, not comparable 1:1 to the lumped model's fixed 6 kg bias.
    EXPECT_GT(geo.added_mass_linear(i, i), 0.5);
    EXPECT_LT(geo.added_mass_linear(i, i), 60.0);
  }
}

TEST_F(HydroGeometryModelFixture, StonefishModeSmokeTest)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.2, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.ang_vel_world = Eigen::Vector3d::Zero();
  state.lin_vel_world = Eigen::Vector3d::Zero();

  HydroParams params;
  const Wrench w = FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);

  ASSERT_TRUE(w.force_world.allFinite());
  ASSERT_TRUE(w.torque_world.allFinite());
  EXPECT_LT(w.force_world.norm(), 10000.0);
  EXPECT_LT(w.torque_world.norm(), 10000.0);
}

// Phase 2.11 Section 1: excluding the 4 small pad_* contact spheres from hull tessellation must
// leave exactly chassis_geom's 96 box patches (6 faces x 16), with the front wall-facing plate
// (x = -0.10m) at its exact 0.24 m^2 area and pure [-1,0,0]^T outward normal -- no lossy
// decimatePatches() merging, since 96 <= kMaxPatches(256).
TEST_F(HydroGeometryModelFixture, FrontPlateAreaAndNormalsExact)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  EXPECT_EQ(geo.patches.size(), 96u);

  double front_area = 0.0;
  for (const auto & p : geo.patches) {
    if (p.normal_body.x() < -0.5) {
      front_area += p.area;
      EXPECT_NEAR(p.normal_body.x(), -1.0, 1e-9);
      EXPECT_NEAR(p.normal_body.y(), 0.0, 1e-9);
      EXPECT_NEAR(p.normal_body.z(), 0.0, 1e-9);
      EXPECT_NEAR(p.centroid_body.x(), -0.10, 1e-6);
    }
  }
  EXPECT_NEAR(front_area, 0.24, 1e-6);
}

// Phase 2.11 Sections 1/2: locks in both the Stonefish-mode wall-suction sign (attractive, -X)
// and the corrected C_squeeze=64.0 magnitude (physical Newtons of resistive damping, not the
// ~1e-6 N/(m/s) unnormalized-era near-zero force).
TEST_F(HydroGeometryModelFixture, StonefishVenturiAndSqueezePhysicalSigns)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;

  RigidBodyState state_crossflow;
  state_crossflow.com_pos_world = Eigen::Vector3d(0.125, 0.0, 0.5);
  state_crossflow.orientation = Eigen::Quaterniond::Identity();
  state_crossflow.lin_vel_world = Eigen::Vector3d(0.0, 0.3, 0.0);
  const Wrench w = FossenHydrodynamics::compute(
    state_crossflow, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
  EXPECT_LT(w.force_world.x(), -1.0);

  RigidBodyState state_rest;
  state_rest.com_pos_world = Eigen::Vector3d(0.125, 0.0, 0.5);
  state_rest.orientation = Eigen::Quaterniond::Identity();
  state_rest.lin_vel_world = Eigen::Vector3d::Zero();
  const Wrench w_rest = FossenHydrodynamics::compute(
    state_rest, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);

  RigidBodyState state_approach = state_rest;
  state_approach.lin_vel_world = Eigen::Vector3d(-0.05, 0.0, 0.0);
  const Wrench w_approach = FossenHydrodynamics::compute(
    state_approach, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
  EXPECT_GT(w_approach.force_world.x(), w_rest.force_world.x() + 0.5);

  RigidBodyState state_sep = state_rest;
  state_sep.lin_vel_world = Eigen::Vector3d(0.05, 0.0, 0.0);
  const Wrench w_sep = FossenHydrodynamics::compute(
    state_sep, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
  EXPECT_LT(w_sep.force_world.x(), w_rest.force_world.x() - 0.5);
}

// Phase 2.12 Section 2/4/3: a minimal in-memory MJCF (written to a temp file for mj_loadXML,
// which has no pure-string-buffer entry point) exercising the three model-agnostic additions
// together: a welded child body ("fairing_child", zero joints -- rigidly attached) whose geom
// must be recursively pulled into "base_link"'s frame by HydroGeometryModel::buildFromMjModel(),
// a <custom><numeric> hydro_vehicle_mass override, and an actuator with a non-default
// ctrlrange (verifying the MJCF itself carries whatever bounds a future model declares, which
// AdhereMujocoHardware::configure() -- not exercised directly by this gtest binary, which has
// no ResourceManager/pluginlib context -- would read via actuator_ctrlrange).
TEST(ModelAgnosticHydro, WeldedChildBodyRecursionAndCustomNumerics)
{
  constexpr const char * kMjcf = R"xml(
<mujoco model="welded_test">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <custom>
    <numeric name="hydro_vehicle_mass" data="35.0"/>
  </custom>
  <worldbody>
    <body name="base_link" pos="0 0 0.5">
      <freejoint name="base_free"/>
      <inertial pos="0 0 0" mass="35.0" diaginertia="1.0 1.0 1.0"/>
      <geom name="base_box" type="box" size="0.1 0.1 0.1"/>
      <site name="thr_test_site" pos="0 0 0" size="0.01"/>
      <body name="fairing_child" pos="0.2 0 0">
        <geom name="fairing_box" type="box" size="0.05 0.05 0.05"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general name="thr_test" site="thr_test_site" gear="1 0 0 0 0 0" ctrlrange="-60 60"
              gaintype="fixed" gainprm="1" biastype="none"/>
  </actuator>
</mujoco>
)xml";

  const std::string path = "/tmp/adhere_mujoco_hardware_test_welded_body.xml";
  {
    std::ofstream out(path);
    out << kMjcf;
  }

  char error[1000] = "";
  mjModel * model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  std::remove(path.c_str());
  ASSERT_NE(model, nullptr) << "Failed to load in-memory welded-body MJCF: " << error;

  const int body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
  ASSERT_GE(body_id, 0);
  const int act_id = mj_name2id(model, mjOBJ_ACTUATOR, "thr_test");
  ASSERT_GE(act_id, 0);
  EXPECT_NEAR(model->actuator_ctrlrange[2 * act_id + 0], -60.0, 1e-9);
  EXPECT_NEAR(model->actuator_ctrlrange[2 * act_id + 1], 60.0, 1e-9);

  // Welded child-body recursion: base_box (6 faces x 4x4 = 96 patches) + fairing_box (96
  // patches) = 192 total, all pulled into base_link's frame and under the 256-patch decimation
  // budget, so this is an exact (not area-weighted-merged) count.
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model, body_id);
  EXPECT_EQ(geo.patches.size(), 192u);

  // fairing_child is only reachable from base_link through the welded-subtree recursion (its
  // geom is a direct child of "fairing_child", not "base_link") -- a patch centroid this far
  // along +X in base_link's own frame is only possible if that recursion actually ran.
  double max_x = -1e9;
  for (const auto & p : geo.patches) {
    max_x = std::max(max_x, p.centroid_body.x());
  }
  EXPECT_GT(max_x, 0.15);

  // <custom><numeric name="hydro_vehicle_mass" data="35.0"/> override.
  HydroParams params;  // default vehicle_mass == 20.0
  HydroParams::applyMjModelOverrides(model, params);
  EXPECT_NEAR(params.vehicle_mass, 35.0, 1e-9);
  // Names absent from this MJCF must leave their defaults untouched.
  EXPECT_NEAR(params.augmented_mass, 26.0, 1e-9);

  mj_deleteModel(model);
}

// ===========================================================================================
// CLAUDE.md Phase 2.13: 5-Tier Hydrodynamic Accuracy V&V Suite
// ===========================================================================================

// Tier 1 (Lamb Sphere Analytical Limit): for a sphere (a=b=c=R), buildFromMjModel()'s
// ellipsoidAlpha-based linear added mass must equal the classical Lamb result
// M_A = 0.5 * rho * (4/3 pi R^3) * I_3 (k1=k2=k3=0.5) to <0.1% relative error.
TEST(HydroVV_Tier1, LambSphereAddedMassAnalyticalExact)
{
  constexpr const char * kMjcf = R"xml(
<mujoco model="lamb_sphere_test">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="sphere_body" pos="0 0 0.5">
      <freejoint name="sphere_free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>
      <geom name="sphere_geom" type="sphere" size="0.2"/>
    </body>
  </worldbody>
</mujoco>
)xml";
  const std::string path = "/tmp/adhere_mujoco_hardware_test_lamb_sphere.xml";
  {
    std::ofstream out(path);
    out << kMjcf;
  }
  char error[1000] = "";
  mjModel * model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  std::remove(path.c_str());
  ASSERT_NE(model, nullptr) << "Failed to load in-memory Lamb-sphere MJCF: " << error;

  const int body_id = mj_name2id(model, mjOBJ_BODY, "sphere_body");
  ASSERT_GE(body_id, 0);
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model, body_id);

  // M_A = 0.5 * rho * (4/3) * pi * R^3, R = 0.2 m, rho = 1000 kg/m^3 (HydroGeometryModel's own
  // hardcoded kRhoWater, see buildFromMjModel()'s comment).
  constexpr double kR = 0.2;
  constexpr double kRho = 1000.0;
  const double displaced_mass = kRho * (4.0 / 3.0) * M_PI * kR * kR * kR;
  const double expected_m_a = 0.5 * displaced_mass;  // == 16.75516... kg
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(geo.added_mass_linear(i, i), expected_m_a, 2e-2);
  }

  // A sphere rotating in an unbounded ideal fluid induces exactly zero fluid motion (Lamb,
  // Hydrodynamics Art. 114), so its added rotational inertia is exactly zero about every axis
  // -- special-cased in buildFromMjModel() since the general heuristic formula used for
  // non-spherical hulls does not itself vanish at a=b=c.
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(geo.added_mass_angular(i, i), 0.0, 1e-9);
  }

  mj_deleteModel(model);
}

// Tier 2 (Degenerate Patch-to-Lumped Exact Equivalence): under pure translation (omega=0,
// identity orientation), kStonefishGeometryDriven's per-patch Venturi suction must sum to
// exactly kParametricLumped's single-point Venturi force, and a pure axial crossflow velocity
// must produce exactly the analytical frontal-drag + skin-friction integral.
TEST_F(HydroGeometryModelFixture, HydroVV_Tier2_PatchToLumpedExactEquivalence)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;  // calm-water defaults

  // Part A: Venturi suction, zero rotation. h_gap = 0.15 - 0.10 = 0.05 m > h_gap_nominal, so
  // neither model's decay term is clamped and the two formulations must agree exactly.
  RigidBodyState state_venturi;
  state_venturi.com_pos_world = Eigen::Vector3d(0.15, 0.0, 0.5);
  state_venturi.orientation = Eigen::Quaterniond::Identity();
  state_venturi.ang_vel_world = Eigen::Vector3d::Zero();
  state_venturi.lin_vel_world = Eigen::Vector3d(0.0, 0.2, 0.0);

  HydroWrenchBreakdown breakdown_lumped;
  FossenHydrodynamics::compute(
    state_venturi, 0.0, params, HydroModelMode::kParametricLumped, nullptr, &breakdown_lumped);
  HydroWrenchBreakdown breakdown_stonefish;
  FossenHydrodynamics::compute(
    state_venturi, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo,
    &breakdown_stonefish);

  EXPECT_NEAR(breakdown_lumped.f_venturi_body.x(), breakdown_stonefish.f_venturi_body.x(), 1e-12);
  EXPECT_NEAR(breakdown_lumped.f_venturi_body.y(), breakdown_stonefish.f_venturi_body.y(), 1e-12);
  EXPECT_NEAR(breakdown_lumped.f_venturi_body.z(), breakdown_stonefish.f_venturi_body.z(), 1e-12);
  // Sanity: this must be the known nonzero suction value, not two sides of a degenerate 0==0.
  EXPECT_LT(breakdown_lumped.f_venturi_body.x(), -1.0);

  // Part B: pure axial relative velocity v_y = 0.4 m/s, no wall proximity effects (patches with
  // normal != +-Y see pure tangential flow, contributing skin friction only; the single +Y face
  // sees pure normal flow, contributing windward pressure drag only -- the -Y face is leeward
  // and contributes nothing, matching kStonefishGeometryDriven's windward-only convention).
  RigidBodyState state_axial;
  state_axial.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);  // far from the wall: no suction
  state_axial.orientation = Eigen::Quaterniond::Identity();
  state_axial.ang_vel_world = Eigen::Vector3d::Zero();
  const double v_y = 0.4;
  state_axial.lin_vel_world = Eigen::Vector3d(0.0, v_y, 0.0);

  double area_windward_y = 0.0;   // the single +Y face
  double area_tangential = 0.0;   // every face except +-Y (zero normal component for pure v_y)
  for (const auto & p : geo.patches) {
    if (p.normal_body.y() > 0.5) {
      area_windward_y += p.area;
    } else if (p.normal_body.y() > -0.5) {
      area_tangential += p.area;  // normal is +-X or +-Z: pure tangential flow
    }
  }
  ASSERT_GT(area_windward_y, 0.0);
  ASSERT_GT(area_tangential, 0.0);

  const double F_y_exact = -(
    params.D_lin.y() * v_y +
    0.5 * params.rho_water * (params.Cd_normal * area_windward_y +
                               params.Cf_tangent * area_tangential) *
      v_y * v_y);
  const Wrench w_axial = FossenHydrodynamics::compute(
    state_axial, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
  EXPECT_NEAR(w_axial.force_world.y(), F_y_exact, 1e-11);
}

// Tier 3 (Analytical Surface Integral Benchmark under Pitch Tilt & Rotation).
TEST_F(HydroGeometryModelFixture, HydroVV_Tier3_AnalyticalPitchSqueezeAndVenturiTorque)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  // --- Part A: pure pitch rate omega_y at the nominal gap (h_gap == h_gap_nominal, so decay ==
  // 1 and (h_nom/h_gap)^3 == 1 exactly for every front patch, all of which share the same flat
  // x = -0.10m centroid -- CLAUDE.md FrontPlateAreaAndNormalsExact). Cd_normal/Cf_tangent/
  // D_ang_lin/D_ang_quad are zeroed to isolate the squeeze-film torque alone: windward pressure
  // drag and skin friction are real (nonzero) contributors to tau_y at this state (see the
  // derivation in the task write-up) and would otherwise contaminate the comparison against the
  // squeeze-only closed form below.
  HydroParams params;
  params.Cd_normal = 0.0;
  params.Cf_tangent = 0.0;
  params.D_ang_lin = Eigen::Vector3d::Zero();
  params.D_ang_quad = Eigen::Vector3d::Zero();

  RigidBodyState state_pitch_rate;
  state_pitch_rate.com_pos_world = Eigen::Vector3d(0.125, 0.0, 0.5);  // h_gap = 0.125-0.10 = h_nom
  state_pitch_rate.orientation = Eigen::Quaterniond::Identity();
  const double omega_y = 0.5;
  state_pitch_rate.ang_vel_world = Eigen::Vector3d(0.0, omega_y, 0.0);
  state_pitch_rate.lin_vel_world = Eigen::Vector3d::Zero();

  HydroWrenchBreakdown breakdown;
  FossenHydrodynamics::compute(
    state_pitch_rate, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown);

  double sum_area_z2 = 0.0;
  double area_plate = 0.0;
  for (const auto & p : geo.patches) {
    if (p.normal_body.x() < -0.5) {
      sum_area_z2 += p.area * p.centroid_body.z() * p.centroid_body.z();
      area_plate += p.area;
    }
  }
  ASSERT_GT(area_plate, 0.0);
  const double tau_y_discrete_exact = -params.C_squeeze * sum_area_z2 * omega_y;
  EXPECT_NEAR(breakdown.t_drag_body.y(), tau_y_discrete_exact, 1e-9);

  // Continuum closed form: Integral_{-Ly}^{Ly} Integral_{-Lz}^{Lz} z^2 dy dz
  //   = (2Ly) * (2Lz^3/3) = A_plate * Lz^2 / 3, so
  //   tau_y,squeeze,exact ~= -C_squeeze * A_plate * Lz^2 / 3 * omega_y.
  // (Lz is the plate's own half-height, recovered from the tessellated AABB -- not hardcoded --
  // so this check stays valid if the hull geometry ever changes.)
  const double L_z = geo.bbox_half_extents.z();
  const double tau_y_continuum = -params.C_squeeze * area_plate * L_z * L_z / 3.0 * omega_y;
  EXPECT_NEAR(tau_y_discrete_exact, tau_y_continuum, 0.1 * std::abs(tau_y_continuum));

  // --- Part B: static pitch tilt theta in a pure Y crossflow (omega=0): the closer half of the
  // front plate experiences less exponential suction attenuation (smaller h_gap => decay closer
  // to 1) than the farther half, so the discrete per-patch model sees a net pitching moment that
  // the single-point Lumped model cannot represent at all. The exact *sign* of that moment
  // depends on which half ends up closer for a given rotation-convention/pitch-sign combination
  // (verified analytically, not asserted a priori here to avoid baking in an unverified
  // convention); what is asserted is the qualitative physical claim CLAUDE.md actually cares
  // about: a nonzero geometry-driven pitch moment vs. an exactly-zero Lumped one.
  HydroParams params_b;
  params_b.cob_offset_body = Eigen::Vector3d::Zero();  // isolate: remove hydrostatic restoring
                                                         // moment so Lumped's torque is exactly
                                                         // zero, not just "small"
  const double theta = 0.05;
  RigidBodyState state_tilt;
  state_tilt.com_pos_world = Eigen::Vector3d(0.125, 0.0, 0.5);
  state_tilt.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
  state_tilt.ang_vel_world = Eigen::Vector3d::Zero();
  state_tilt.lin_vel_world = Eigen::Vector3d(0.0, 0.5, 0.0);

  const Eigen::Matrix3d R = state_tilt.orientation.toRotationMatrix();
  double h_gap_pos_z = -1.0, h_gap_neg_z = -1.0;
  for (const auto & p : geo.patches) {
    if (p.normal_body.x() < -0.5) {
      const double x_world = state_tilt.com_pos_world.x() + (R * p.centroid_body).x();
      if (p.centroid_body.z() > 0.1 && h_gap_pos_z < 0.0) {
        h_gap_pos_z = x_world;
      }
      if (p.centroid_body.z() < -0.1 && h_gap_neg_z < 0.0) {
        h_gap_neg_z = x_world;
      }
    }
  }
  ASSERT_GE(h_gap_pos_z, 0.0);
  ASSERT_GE(h_gap_neg_z, 0.0);
  ASSERT_NE(h_gap_pos_z, h_gap_neg_z) << "pitch tilt must break the z-symmetry of the front gap";

  const Wrench w_tilt = FossenHydrodynamics::compute(
    state_tilt, 0.0, params_b, HydroModelMode::kStonefishGeometryDriven, &geo);
  const Wrench w_tilt_lumped = FossenHydrodynamics::compute(state_tilt, 0.0, params_b);

  EXPECT_GT(std::abs(w_tilt.torque_world.y()), 1e-6);
  EXPECT_NEAR(w_tilt_lumped.torque_world.y(), 0.0, 1e-9);
  // Sign check derived from first principles (r x F with F concentrated on the closer/stronger
  // side): whichever z-half is closer to the wall contributes the larger-magnitude suction
  // force, and that side's own sign dominates tau_y with a flipped sign.
  if (h_gap_pos_z < h_gap_neg_z) {
    EXPECT_LT(w_tilt.torque_world.y(), -1e-6);
  } else {
    EXPECT_GT(w_tilt.torque_world.y(), 1e-6);
  }
}

// Tier 4 (Thermodynamic Passivity / Energy Dissipation Law): across 200 deterministic
// pseudo-random 6-DOF states in calm water, the dissipative hydrodynamic power must be strictly
// negative, and the rotational Munk self-coupling term must be exactly workless.
TEST_F(HydroGeometryModelFixture, HydroVV_Tier4_ThermodynamicPassivityAndWorklessMunk)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  const HydroParams params;  // calm-water defaults: zero current, zero gusts

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist_x(0.105, 0.5);
  std::uniform_real_distribution<double> dist_v(-1.0, 1.0);
  std::uniform_real_distribution<double> dist_w(-2.0, 2.0);

  for (int sample = 0; sample < 200; ++sample) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(dist_x(rng), 0.0, 0.5);
    state.orientation = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d v(dist_v(rng), dist_v(rng), dist_v(rng));
    const Eigen::Vector3d w(dist_w(rng), dist_w(rng), dist_w(rng));
    state.lin_vel_world = v;
    state.ang_vel_world = w;

    HydroWrenchBreakdown breakdown;
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown);

    const double p_diss =
      v.dot(breakdown.f_drag_body + breakdown.f_squeeze_body) + w.dot(breakdown.t_drag_body);
    EXPECT_LT(p_diss, 0.0) << "sample " << sample;

    // Rotational Munk self-coupling is workless by construction (a . (a x b) == 0 for any a,b):
    // verified directly against the same geo.added_mass_angular FossenHydrodynamics::compute()
    // itself reads, independent of the (non-workless) translational v x (M_lin v) term that is
    // also folded into the combined t_munk_body breakdown field.
    const Eigen::Vector3d t_munk_rot = -w.cross(geo.added_mass_angular * w);
    EXPECT_NEAR(w.dot(t_munk_rot), 0.0, 1e-12) << "sample " << sample;
  }
}

// Tier 5 (Closed-Loop MuJoCo Dynamic Trajectory V&V): actually step mj_step with the Fossen
// wrench applied every tick (mirroring AdhereMujocoHardware::applyHydrodynamics()), away from
// the wall so no contact forces are involved.
TEST_F(HydroGeometryModelFixture, HydroVV_Tier5_ClosedLoopTerminalVelocityAndRollStability)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  // Neutral buoyancy (vehicle_mass == augmented_mass == the MJCF's actual 26kg, displaced_volume
  // chosen so rho*V*g exactly cancels mj_step's own built-in gravity on that mass) isolates the
  // crossflow/roll dynamics under test from the unrelated, already-documented net +1.96N rise
  // drift ("数值校验" section) that would otherwise carry the ROV out of the submerged band
  // during a long trajectory.
  HydroParams params;
  params.vehicle_mass = 26.0;
  params.augmented_mass = 26.0;
  params.displaced_volume = params.vehicle_mass / params.rho_water;

  auto stepWithHydro = [&](mjData * d, const HydroParams & p) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(
      d->xipos[3 * body_id_ + 0], d->xipos[3 * body_id_ + 1], d->xipos[3 * body_id_ + 2]);
    state.orientation = Eigen::Quaterniond(
      d->xquat[4 * body_id_ + 0], d->xquat[4 * body_id_ + 1], d->xquat[4 * body_id_ + 2],
      d->xquat[4 * body_id_ + 3]);
    state.ang_vel_world = Eigen::Vector3d(
      d->cvel[6 * body_id_ + 0], d->cvel[6 * body_id_ + 1], d->cvel[6 * body_id_ + 2]);
    state.lin_vel_world = Eigen::Vector3d(
      d->cvel[6 * body_id_ + 3], d->cvel[6 * body_id_ + 4], d->cvel[6 * body_id_ + 5]);
    const Wrench w = FossenHydrodynamics::compute(
      state, d->time, p, HydroModelMode::kStonefishGeometryDriven, &geo);
    d->xfrc_applied[6 * body_id_ + 0] = w.force_world.x();
    d->xfrc_applied[6 * body_id_ + 1] = w.force_world.y();
    d->xfrc_applied[6 * body_id_ + 2] = w.force_world.z();
    d->xfrc_applied[6 * body_id_ + 3] = w.torque_world.x();
    d->xfrc_applied[6 * body_id_ + 4] = w.torque_world.y();
    d->xfrc_applied[6 * body_id_ + 5] = w.torque_world.z();
    mj_step(model_, d);
  };

  // --- Part A: steady 0.3 m/s crossflow terminal-velocity convergence, far from the wall
  // (x=1.0m) so no pad/wall contact forces are in play. ---
  {
    HydroParams p = params;
    p.current_steady_world = Eigen::Vector3d(0.0, 0.3, 0.0);

    mjData * d = mj_makeData(model_);
    ASSERT_NE(d, nullptr);
    mj_resetData(model_, d);
    d->qpos[0] = 1.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.5;
    d->qpos[3] = 1.0;  // identity quaternion [w,x,y,z]
    d->qpos[4] = 0.0;
    d->qpos[5] = 0.0;
    d->qpos[6] = 0.0;
    mj_forward(model_, d);

    std::vector<double> v_y_samples;
    constexpr int kTotalSteps = 6000;   // 12s @ dt=0.002
    constexpr int kSampleEvery = 200;   // every 0.4s
    for (int step = 0; step < kTotalSteps; ++step) {
      stepWithHydro(d, p);
      if (step % kSampleEvery == 0) {
        v_y_samples.push_back(d->qvel[1]);
      }
    }
    mj_deleteData(d);

    ASSERT_GT(v_y_samples.size(), 2u);
    for (size_t i = 1; i < v_y_samples.size(); ++i) {
      EXPECT_GE(v_y_samples[i], v_y_samples[i - 1] - 1e-6)
        << "v_y must converge monotonically (sample " << i << ")";
    }
    EXPECT_NEAR(v_y_samples.back(), 0.3, 0.01);
  }

  // --- Part B: 10 deg initial roll (about body X, the wall-normal axis) must decay stably to
  // <0.5 deg under the hydrostatic CoB-offset restoring moment + angular drag, never diverging.
  {
    mjData * d = mj_makeData(model_);
    ASSERT_NE(d, nullptr);
    mj_resetData(model_, d);
    const double roll0 = 10.0 * M_PI / 180.0;
    const Eigen::Quaterniond q0(Eigen::AngleAxisd(roll0, Eigen::Vector3d::UnitX()));
    d->qpos[0] = 1.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.5;
    d->qpos[3] = q0.w();
    d->qpos[4] = q0.x();
    d->qpos[5] = q0.y();
    d->qpos[6] = q0.z();
    mj_forward(model_, d);

    auto rollDeg = [&]() {
      const double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
      const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
      return roll * 180.0 / M_PI;
    };

    constexpr int kTotalSteps = 4000;  // 8s @ dt=0.002
    for (int step = 0; step < kTotalSteps; ++step) {
      stepWithHydro(d, params);
      ASSERT_LT(std::abs(rollDeg()), 15.0) << "roll diverged at step " << step;
    }
    EXPECT_LT(std::abs(rollDeg()), 0.5);
    mj_deleteData(d);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
