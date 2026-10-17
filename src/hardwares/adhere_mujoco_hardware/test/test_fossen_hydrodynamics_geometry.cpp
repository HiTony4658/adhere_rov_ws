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
using adhere_mujoco_hardware::RadiationMemoryState;
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

  // M_A = 0.5 * rho * (4/3) * pi * R^3, R = 0.2 m, rho = 1000 kg/m^3 (buildFromMjModel()'s own
  // rho_water parameter default, see RhoWaterParameterizationScalesAddedMassLinearly below).
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

// Tier 1 companion (Spheroid Symmetry-Axis Limit): for a spheroid with two equal semi-axes
// (b=c != a), Fossen (2011) Eq.(2.159)-(2.161)'s rotational added inertia about the distinct
// symmetry axis (x, paired with the equal b=c pair) must vanish by the same ideal-fluid
// symmetry argument as the sphere case, while the inertia about the two transverse axes (y,
// z -- paired respectively with (a,c) and (a,b), neither of which are degenerate here) must be
// strictly positive and finite (added rotational inertia can never be negative).
TEST(HydroVV_Tier1, SpheroidSymmetryAxisRotationalInertiaVanishes)
{
  constexpr const char * kMjcf = R"xml(
<mujoco model="spheroid_test">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="spheroid_body" pos="0 0 0.5">
      <freejoint name="spheroid_free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>
      <geom name="spheroid_geom" type="ellipsoid" size="0.3 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)xml";
  const std::string path = "/tmp/adhere_mujoco_hardware_test_spheroid.xml";
  {
    std::ofstream out(path);
    out << kMjcf;
  }
  char error[1000] = "";
  mjModel * model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  std::remove(path.c_str());
  ASSERT_NE(model, nullptr) << "Failed to load in-memory spheroid MJCF: " << error;

  const int body_id = mj_name2id(model, mjOBJ_BODY, "spheroid_body");
  ASSERT_GE(body_id, 0);
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model, body_id);

  // a=0.3 (x), b=c=0.1 (y,z): rotation about x (paired with the degenerate b=c pair) vanishes;
  // rotation about y/z (each paired with one non-degenerate axis, a) is strictly positive.
  EXPECT_NEAR(geo.added_mass_angular(0, 0), 0.0, 1e-9);
  EXPECT_GT(geo.added_mass_angular(1, 1), 0.0);
  EXPECT_GT(geo.added_mass_angular(2, 2), 0.0);
  ASSERT_TRUE(std::isfinite(geo.added_mass_angular(1, 1)));
  ASSERT_TRUE(std::isfinite(geo.added_mass_angular(2, 2)));
  // b=c exactly, so by construction the two transverse terms must agree.
  EXPECT_NEAR(geo.added_mass_angular(1, 1), geo.added_mass_angular(2, 2), 1e-9);

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

  // Hydrodynamic-accuracy audit Finding D: Cf_tangent is now Reynolds-dependent (Schlichting
  // turbulent flat-plate correlation, scaled MULTIPLICATIVELY relative to a fixed reference Re
  // so that Cf_tangent == 0.0 still fully disables skin friction and Re == kReReferenceCf
  // reproduces params.Cf_tangent exactly -- see reynoldsScaledCf()'s doc comment in
  // fossen_hydrodynamics.cpp) for every patch, curved or sharp-edged; Cd_normal's "drag crisis"
  // Re-dependence applies ONLY to is_curved patches, and rov_wall.xml's chassis is a pure box
  // (is_curved == false everywhere), so Cd_eff on the windward +Y face stays exactly
  // params.Cd_normal here -- only the Cf_eff term below needs updating to mirror the
  // implementation's Re-corrected formula (both sides of this "exact equivalence" now compute
  // the same Reynolds-dependent quantity, so the equivalence claim is unchanged in kind).
  constexpr double kReReferenceCf = 2.0e5;  // must match fossen_hydrodynamics.cpp's own constant
  auto schlichtingShape = [](double Re) {
    const double denom = std::log10(std::max(Re, 1.0e4)) - 2.0;
    return 0.075 / (denom * denom);
  };
  const double L_plate = 2.0 * geo.bbox_half_extents.maxCoeff();
  const double Re_t = v_y * L_plate / params.nu_water;
  const double Cf_exact =
    params.Cf_tangent * schlichtingShape(Re_t) / schlichtingShape(kReReferenceCf);

  const double F_y_exact = -(
    params.D_lin.y() * v_y +
    0.5 * params.rho_water * (params.Cd_normal * area_windward_y +
                               Cf_exact * area_tangential) *
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

// ===========================================================================================
// Hydrodynamic-accuracy audit: rho_water parameterization + wall-proximity added mass
// ===========================================================================================

// Audit fix 5.2: buildFromMjModel()'s rho_water parameter must scale both added-mass tensors
// exactly linearly (M_A = k_i * rho_water * (4/3 pi a b c), k_i independent of rho_water), and
// must leave every geometry-only quantity (patches, bbox) completely unaffected.
TEST_F(HydroGeometryModelFixture, RhoWaterParameterizationScalesAddedMassLinearly)
{
  const HydroGeometryModel geo_default = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  const HydroGeometryModel geo_double_rho =
    HydroGeometryModel::buildFromMjModel(model_, body_id_, 2000.0);

  for (int i = 0; i < 3; ++i) {
    ASSERT_GT(geo_default.added_mass_linear(i, i), 0.0);
    EXPECT_NEAR(
      geo_double_rho.added_mass_linear(i, i), 2.0 * geo_default.added_mass_linear(i, i),
      1e-9 * geo_default.added_mass_linear(i, i));
    if (geo_default.added_mass_angular(i, i) > 1e-9) {
      EXPECT_NEAR(
        geo_double_rho.added_mass_angular(i, i), 2.0 * geo_default.added_mass_angular(i, i),
        1e-6 * geo_default.added_mass_angular(i, i));
    }
  }
  EXPECT_EQ(geo_default.patches.size(), geo_double_rho.patches.size());
  EXPECT_NEAR(geo_default.bbox_half_extents.x(), geo_double_rho.bbox_half_extents.x(), 1e-12);
}

// Wall-proximity added-mass enhancement: under a pure +World-X ambient-fluid acceleration (a
// gust at its own t=0 window start), F_fk_am_body's X-component must grow strictly as the hull
// approaches the World X=0 wall, must stay bounded (never diverge) arbitrarily close to it, and
// must reduce to the unbounded free-stream tensor's own prediction far from the wall.
TEST_F(HydroGeometryModelFixture, WallProximityAddedMassAmplifiesNearWallAndIsBounded)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);

  HydroParams params;
  // GustEvent::acceleration(t_start) = peak_x * pi/(t_end-t_start): a pure +World-X impulsive
  // acceleration peak at t=0, the window's own start.
  params.gust_events = {adhere_mujoco_hardware::GustEvent{0.0, 1.0, 0.0, 1.0}};

  RigidBodyState state_far;
  state_far.com_pos_world = Eigen::Vector3d(1.0, 0.0, 0.5);  // h_gap ~0.9m: negligible wall effect
  state_far.orientation = Eigen::Quaterniond::Identity();

  RigidBodyState state_near;
  state_near.com_pos_world = Eigen::Vector3d(0.15, 0.0, 0.5);  // h_gap = 0.05m: strong wall effect
  state_near.orientation = Eigen::Quaterniond::Identity();

  RigidBodyState state_contact;
  state_contact.com_pos_world = Eigen::Vector3d(0.1005, 0.0, 0.5);  // h_gap clamped to the floor
  state_contact.orientation = Eigen::Quaterniond::Identity();

  HydroWrenchBreakdown b_far, b_near, b_contact;
  FossenHydrodynamics::compute(
    state_far, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &b_far);
  FossenHydrodynamics::compute(
    state_near, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &b_near);
  FossenHydrodynamics::compute(
    state_contact, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &b_contact);

  ASSERT_GT(std::abs(b_far.f_fk_am_body.x()), 1e-6) << "test setup must produce a nonzero signal";
  // Monotonic amplification as the hull approaches the wall.
  EXPECT_GT(std::abs(b_near.f_fk_am_body.x()), std::abs(b_far.f_fk_am_body.x()) * 1.05);
  EXPECT_GT(std::abs(b_contact.f_fk_am_body.x()), std::abs(b_near.f_fk_am_body.x()));

  // Bounded: the leading-order image series itself diverges as h -> 0, so the implementation
  // clamps at a finite ceiling (kMaxWallAmplification, an implementation-internal constant not
  // re-derived exactly here -- loosely bounded at 10x the unbounded free-stream prediction).
  ASSERT_TRUE(std::isfinite(b_contact.f_fk_am_body.x()));
  EXPECT_LT(std::abs(b_contact.f_fk_am_body.x()), std::abs(b_far.f_fk_am_body.x()) * 10.0);

  // Far from the wall, the correction must be negligible (within 1% of the unbounded
  // free-stream tensor's own prediction, computed directly from geo.added_mass_linear).
  const double a_fluid_x =
    adhere_mujoco_hardware::GustEvent{0.0, 1.0, 0.0, 1.0}.acceleration(0.0).x();
  const double expected_far =
    (params.rho_water * params.displaced_volume + geo.added_mass_linear(0, 0)) * a_fluid_x;
  EXPECT_NEAR(b_far.f_fk_am_body.x(), expected_far, std::abs(expected_far) * 0.01);
}

// ===========================================================================================
// Hydrodynamic-accuracy audit Finding D: Reynolds-number-dependent Cd_normal/Cf_tangent
// ===========================================================================================

// Cf_tangent must follow the Schlichting turbulent flat-plate correlation (scaled relative to
// the fixed reference Re anchor -- see reynoldsScaledCf()'s doc comment in
// fossen_hydrodynamics.cpp): the ratio of two skin-friction-only forces at two different speeds
// must match the correlation's own predicted ratio, not simply "monotonic".
TEST_F(HydroGeometryModelFixture, CfTangentFollowsSchlichtingCorrelation)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;
  params.Cd_normal = 0.0;  // isolate skin friction: zero out normal pressure drag
  params.D_lin = Eigen::Vector3d::Zero();
  params.D_quad = Eigen::Vector3d::Zero();

  constexpr double kReReferenceCf = 2.0e5;
  auto schlichtingShape = [](double Re) {
    const double denom = std::log10(std::max(Re, 1.0e4)) - 2.0;
    return 0.075 / (denom * denom);
  };
  const double L_plate = 2.0 * geo.bbox_half_extents.maxCoeff();

  auto skinFrictionForceY = [&](double v_y) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);  // far from the wall
    state.orientation = Eigen::Quaterniond::Identity();
    state.lin_vel_world = Eigen::Vector3d(0.0, v_y, 0.0);
    const Wrench w = FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
    return w.force_world.y();
  };

  const double v_lo = 0.1;
  const double v_hi = 0.8;
  const double F_lo = skinFrictionForceY(v_lo);
  const double F_hi = skinFrictionForceY(v_hi);
  ASSERT_LT(F_lo, 0.0);
  ASSERT_LT(F_hi, 0.0);

  const double Cf_lo_ratio =
    schlichtingShape(v_lo * L_plate / params.nu_water) / schlichtingShape(kReReferenceCf);
  const double Cf_hi_ratio =
    schlichtingShape(v_hi * L_plate / params.nu_water) / schlichtingShape(kReReferenceCf);
  // F_skin ~ Cf(Re) * v^2 (tangential-only patches, pure v_y -> v_t_norm == |v_y| everywhere
  // except the single windward +-Y face already excluded by Cd_normal=0 not mattering here).
  const double expected_ratio = (Cf_hi_ratio * v_hi * v_hi) / (Cf_lo_ratio * v_lo * v_lo);
  const double actual_ratio = F_hi / F_lo;
  EXPECT_NEAR(actual_ratio, expected_ratio, expected_ratio * 0.01);
}

// Sharp-edged (box) hulls must NOT exhibit a Reynolds-dependent drag crisis: rov_wall.xml's
// chassis is a pure box (SurfacePatch::is_curved == false everywhere), so Cd_eff on the windward
// face must be exactly params.Cd_normal regardless of how far Re is swept across the drag-crisis
// band -- this is the physically-correct behavior (fixed separation point), not an omission.
TEST_F(HydroGeometryModelFixture, CdNormalConstantForSharpEdgedBoxHull)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  for (const auto & p : geo.patches) {
    EXPECT_FALSE(p.is_curved) << "rov_wall.xml's chassis_geom is a box; no patch should be "
                                 "tagged is_curved";
  }

  HydroParams params;
  params.Cf_tangent = 0.0;  // isolate normal pressure drag
  params.D_lin = Eigen::Vector3d::Zero();
  params.D_quad = Eigen::Vector3d::Zero();

  auto normalDragForceY = [&](double v_y) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);
    state.orientation = Eigen::Quaterniond::Identity();
    state.lin_vel_world = Eigen::Vector3d(0.0, v_y, 0.0);
    const Wrench w = FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
    return w.force_world.y();
  };

  // v_lo is well below Re_crit_low, v_hi is well above Re_crit_high -- a Re-dependent Cd would
  // show a clearly different Cd_eff/v^2 ratio between the two; a box hull must not.
  const double F_lo = normalDragForceY(0.05);
  const double F_hi = normalDragForceY(2.0);
  ASSERT_LT(F_lo, 0.0);
  ASSERT_LT(F_hi, 0.0);
  const double Cd_eff_lo_proxy = F_lo / (0.05 * 0.05);
  const double Cd_eff_hi_proxy = F_hi / (2.0 * 2.0);
  EXPECT_NEAR(Cd_eff_hi_proxy, Cd_eff_lo_proxy, std::abs(Cd_eff_lo_proxy) * 1e-9);
}

// A smooth curved geom (sphere) MUST exhibit the drag-crisis transition: Cd_eff blends from
// Cd_normal to Cd_normal_supercritical as Re sweeps through [Re_crit_low, Re_crit_high],
// monotonically, clamped flat outside that band.
TEST(HydroVV_Finding_D, CdNormalDragCrisisAppliesToCurvedGeomOnly)
{
  constexpr const char * kMjcf = R"xml(
<mujoco model="cd_crisis_sphere_test">
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
  const std::string path = "/tmp/adhere_mujoco_hardware_test_cd_crisis_sphere.xml";
  {
    std::ofstream out(path);
    out << kMjcf;
  }
  char error[1000] = "";
  mjModel * model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  std::remove(path.c_str());
  ASSERT_NE(model, nullptr) << "Failed to load in-memory sphere MJCF: " << error;
  const int body_id = mj_name2id(model, mjOBJ_BODY, "sphere_body");
  ASSERT_GE(body_id, 0);

  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model, body_id);
  ASSERT_GT(geo.cd_diameter, 0.0);
  for (const auto & p : geo.patches) {
    ASSERT_TRUE(p.is_curved) << "every patch of a pure-sphere body must be tagged is_curved";
  }

  HydroParams params;
  params.Cf_tangent = 0.0;  // isolate normal pressure drag
  params.D_lin = Eigen::Vector3d::Zero();
  params.D_quad = Eigen::Vector3d::Zero();

  auto frontalDragForceY = [&](double v_y) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(1.0, 0.0, 0.5);
    state.orientation = Eigen::Quaterniond::Identity();
    state.lin_vel_world = Eigen::Vector3d(0.0, v_y, 0.0);
    const Wrench w = FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);
    return w.force_world.y();
  };

  // Re = v * diameter / nu_water. Pick v's so Re lands well below, inside, and well above
  // [Re_crit_low, Re_crit_high] (defaults 1.5e5/3.5e5) for this sphere's cd_diameter (0.4m).
  const double v_sub = 0.2;    // Re ~ 8e4, below the band
  const double v_mid = 0.6;    // Re ~ 2.4e5, inside the band
  const double v_super = 1.2;  // Re ~ 4.8e5, above the band

  // Each sample's force/v^2 ("drag admittance") bundles together -0.5*rho_water*Cd_eff with the
  // per-patch windward-hemisphere surface-integral-of-cos^2(theta) area factor -- a fixed
  // geometric constant, NOT a free-stream frontal-projected-area constant, since compute()
  // integrates each patch's own local v_n = v_y * normal.y(), not a single lumped frontal area.
  // That geometric factor is identical across all three samples (same hull, same patches), so it
  // cancels EXACTLY in the ratio below -- letting this test assert the precise Cd_normal /
  // Cd_normal_supercritical ratio without needing to independently re-derive that geometric
  // integration factor.
  const double admittance_sub = frontalDragForceY(v_sub) / (v_sub * v_sub);
  const double admittance_mid = frontalDragForceY(v_mid) / (v_mid * v_mid);
  const double admittance_super = frontalDragForceY(v_super) / (v_super * v_super);
  ASSERT_LT(admittance_sub, 0.0);
  ASSERT_LT(admittance_mid, 0.0);
  ASSERT_LT(admittance_super, 0.0);

  // Monotonic magnitude decrease sub -> mid -> super (the drag-crisis transition).
  EXPECT_GT(std::abs(admittance_sub), std::abs(admittance_mid));
  EXPECT_GT(std::abs(admittance_mid), std::abs(admittance_super));

  // Bounded ratio check: each patch sees its OWN local v_n = v_y * normal.y() (not the
  // free-stream v_y uniformly), so different patches of the SAME hemisphere sit at different
  // local Re simultaneously -- sweeping the global speed does not move the whole hemisphere
  // into a single Cd plateau at once (a patch near the equator has small v_n and may stay
  // subcritical even while the apex patch is deep supercritical). The integrated admittance
  // ratio is therefore a *smoothed blend*, not a clean two-point jump -- but it is still
  // mathematically bounded: every single patch's Cd_eff is a convex combination of Cd_normal and
  // Cd_normal_supercritical (dragCrisisCd()'s frac in [0,1]), so the GLOBAL admittance ratio must
  // lie strictly between 1.0 (if every patch were still subcritical at both speeds) and the fully
  // idealized point-Cd ratio Cd_normal/Cd_normal_supercritical (if every patch were fully
  // supercritical at v_super) -- and strictly greater than 1.0 here specifically because v_super's
  // apex patch Re (4.8e5) is well past v_sub's apex patch Re (8e4), guaranteeing at least some
  // patches transitioned.
  const double ratio_actual = admittance_sub / admittance_super;
  const double ratio_ideal_ceiling = params.Cd_normal / params.Cd_normal_supercritical;
  EXPECT_GT(ratio_actual, 1.0);
  EXPECT_LT(ratio_actual, ratio_ideal_ceiling);

  mj_deleteModel(model);
}

// ===========================================================================================
// Hydrodynamic-accuracy audit Finding B: wall-proximity rotational added inertia + the
// (explicitly unvalidated placeholder) translation-rotation coupling term
// ===========================================================================================

// Rotational wall-correction mapping: pitch/yaw (which induce wall-normal local patch motion,
// per the omega x r_p kinematic argument in fossen_hydrodynamics.cpp) must be amplified by the
// STRONGER amp_normal factor, while roll (purely wall-tangent motion) gets the weaker
// amp_tangent -- so a combined roll+pitch Munk moment (which depends on the DIFFERENCE between
// the two axes' added inertia, T_munk_z = omega_x*omega_y*(I_yy-I_xx)) must grow in magnitude as
// the hull approaches the wall, since I_yy (pitch) grows faster than I_xx (roll).
TEST_F(HydroGeometryModelFixture, WallProximityRotationalAddedInertiaAmplifiesNearWall)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;
  params.k_coupling_wall = 0.0;  // isolate the rotational self-coupling (Munk) term from the
                                 // separate translation-rotation coupling term below

  auto munkZFor = [&](double h_gap) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(params.wall_x_offset + h_gap, 0.0, 0.5);
    state.orientation = Eigen::Quaterniond::Identity();
    state.ang_vel_world = Eigen::Vector3d(1.0, 1.0, 0.0);  // combined roll + pitch
    state.lin_vel_world = Eigen::Vector3d::Zero();
    HydroWrenchBreakdown breakdown;
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown);
    return breakdown.t_munk_body.z();
  };

  const double munk_far = munkZFor(1.0);
  const double munk_near = munkZFor(0.05);
  ASSERT_GT(std::abs(munk_far), 1e-9) << "test setup must produce a nonzero baseline signal";
  EXPECT_GT(std::abs(munk_near), std::abs(munk_far) * 1.05);
  ASSERT_TRUE(std::isfinite(munk_near));
}

// Translation-rotation coupling (UNVALIDATED placeholder, see HydroParams::k_coupling_wall):
// must vanish far from the wall, grow approaching it, and be bounded (clamped) near it -- and
// must satisfy the reciprocal relation built into its construction (the same scalar coefficient
// relates F_x<->omega_y and T_y<->v_x).
TEST_F(HydroGeometryModelFixture, TranslationRotationCouplingVanishesGrowsAndBoundsNearWall)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;  // default k_coupling_wall = 0.1

  auto couplingFxForPurePitch = [&](double h_gap) {
    RigidBodyState state;
    state.com_pos_world = Eigen::Vector3d(params.wall_x_offset + h_gap, 0.0, 0.5);
    state.orientation = Eigen::Quaterniond::Identity();
    state.ang_vel_world = Eigen::Vector3d(0.0, 1.0, 0.0);  // pure pitch
    state.lin_vel_world = Eigen::Vector3d::Zero();
    HydroWrenchBreakdown breakdown;
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown);
    return breakdown.f_coupling_body.x();
  };

  const double f_far = couplingFxForPurePitch(1.0);    // (a/h)^3 ~ 0.001, negligible
  const double f_mid = couplingFxForPurePitch(0.15);   // (a/h)^3 ~ 0.3, unclamped and growing
  const double f_near = couplingFxForPurePitch(0.02);  // (a/h)^3 ~ 125, deep in the clamped band
  const double f_deeper = couplingFxForPurePitch(0.005);  // even deeper in the clamped band

  ASSERT_GT(std::abs(f_mid), 1e-9) << "test setup must produce a nonzero baseline signal";
  // Vanishes far from the wall relative to the mid-range sample.
  EXPECT_LT(std::abs(f_far), std::abs(f_mid) * 0.05);
  // Grows approaching the wall.
  EXPECT_GT(std::abs(f_near), std::abs(f_mid));
  // Bounded/clamped: two samples both deep in the saturated (a/h)^3 regime must agree (if
  // unclamped, (a/h)^3 would keep growing by orders of magnitude between h=0.02 and h=0.005).
  EXPECT_NEAR(f_deeper, f_near, std::abs(f_near) * 1e-9);
  ASSERT_TRUE(std::isfinite(f_deeper));

  // Reciprocity: driving pure v_x = 1.0 m/s (instead of pure omega_y = 1.0 rad/s) at the SAME
  // wall distance must produce the reciprocal torque T_coupling_body.y() with identical
  // magnitude, since both are built from the one shared coefficient C_coupling_y.
  RigidBodyState state_v;
  state_v.com_pos_world = Eigen::Vector3d(params.wall_x_offset + 0.02, 0.0, 0.5);
  state_v.orientation = Eigen::Quaterniond::Identity();
  state_v.lin_vel_world = Eigen::Vector3d(1.0, 0.0, 0.0);
  state_v.ang_vel_world = Eigen::Vector3d::Zero();
  HydroWrenchBreakdown breakdown_v;
  FossenHydrodynamics::compute(
    state_v, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown_v);
  EXPECT_NEAR(breakdown_v.t_coupling_body.y(), f_near, std::abs(f_near) * 1e-9);
}

// Regression lock: f_coupling_body/t_coupling_body must exist, be nonzero near the wall, and
// NOT be folded into f_drag_body/t_drag_body -- i.e. Tier4's existing dissipative-power check
// (which only sums f_drag_body/f_squeeze_body/t_drag_body, deliberately excluding t_munk_body)
// must still see exactly the same passivity law hold even with this new reactive term active,
// across the same 200-sample random sweep Tier4 itself uses.
TEST_F(HydroGeometryModelFixture, CouplingTermExcludedFromDissipativePowerLaw)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  const HydroParams params;  // default k_coupling_wall = 0.1, nonzero

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist_x(0.105, 0.5);
  std::uniform_real_distribution<double> dist_v(-1.0, 1.0);
  std::uniform_real_distribution<double> dist_w(-2.0, 2.0);

  bool saw_nonzero_coupling = false;
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

    if (breakdown.f_coupling_body.norm() > 1e-9 || breakdown.t_coupling_body.norm() > 1e-9) {
      saw_nonzero_coupling = true;
    }
    const double p_diss =
      v.dot(breakdown.f_drag_body + breakdown.f_squeeze_body) + w.dot(breakdown.t_drag_body);
    EXPECT_LT(p_diss, 0.0) << "sample " << sample;
  }
  EXPECT_TRUE(saw_nonzero_coupling)
    << "test setup must exercise at least one near-wall sample with nonzero coupling";
}

// ===========================================================================================
// Hydrodynamic-accuracy audit Finding A: frequency-dependent radiation damping (washout filter)
// ===========================================================================================

// Zero-regression guard: with the default K_radiation_memory == Vector3d::Zero(), passing a
// live memory/dt pair must reproduce byte-identical output to the pre-Finding-A call signature
// (and to memory == nullptr), since F_memory_body = -K.cwiseProduct(...) is identically zero
// regardless of what lagged_vel_body/dt do.
TEST_F(HydroGeometryModelFixture, RadiationMemoryZeroGainReproducesExistingBehaviorExactly)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  const HydroParams params;  // default K_radiation_memory == Zero()

  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.lin_vel_world = Eigen::Vector3d(0.1, 0.4, -0.05);
  state.ang_vel_world = Eigen::Vector3d(0.2, -0.3, 0.1);

  const Wrench w_no_memory = FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo);

  RadiationMemoryState memory;
  memory.lagged_vel_body = Eigen::Vector3d(5.0, -5.0, 5.0);  // deliberately non-settled garbage
  memory.initialized = true;
  const Wrench w_with_memory = FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, nullptr, &memory, 0.002);

  EXPECT_NEAR(w_no_memory.force_world.x(), w_with_memory.force_world.x(), 1e-12);
  EXPECT_NEAR(w_no_memory.force_world.y(), w_with_memory.force_world.y(), 1e-12);
  EXPECT_NEAR(w_no_memory.force_world.z(), w_with_memory.force_world.z(), 1e-12);
}

// At true steady state (constant v_rel_body held for many ticks), lagged_vel_body converges to
// v_rel_body and f_radiation_memory_body must decay to ~0 -- no double-counting with D_lin/
// D_quad's already-tuned quasi-steady damping (directly protects Tier5 Part A's terminal-
// velocity convergence test).
TEST_F(HydroGeometryModelFixture, RadiationMemoryWashoutVanishesAtSteadyState)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;
  params.K_radiation_memory = Eigen::Vector3d(50.0, 50.0, 50.0);  // nonzero test gain
  params.tau_radiation_memory = 0.3;

  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.lin_vel_world = Eigen::Vector3d(0.0, 0.3, 0.0);  // constant, held fixed below
  state.ang_vel_world = Eigen::Vector3d::Zero();

  RadiationMemoryState memory;
  // Deliberately seed lagged_vel_body far from v_rel_body so this test actually exercises
  // convergence dynamics (not already-at-the-fixed-point trivia): initialized=true here skips
  // compute()'s first-call auto-seed, forcing the washout filter to converge from this
  // mismatch over the loop below.
  memory.lagged_vel_body = Eigen::Vector3d(0.0, -0.3, 0.0);
  memory.initialized = true;
  constexpr double kDt = 0.002;
  HydroWrenchBreakdown breakdown;
  FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory, kDt);
  ASSERT_GT(std::abs(breakdown.f_radiation_memory_body.y()), 1.0)
    << "test setup must start with a substantial initial mismatch";

  for (int i = 0; i < 5000; ++i) {  // 10s @ 2ms, >> tau_radiation_memory (0.3s)
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory,
      kDt);
  }
  EXPECT_NEAR(breakdown.f_radiation_memory_body.y(), 0.0, 1e-6);
}

// A step change in velocity must produce a large immediate f_radiation_memory_body response
// that decays back toward zero over a timescale set by tau_radiation_memory.
TEST_F(HydroGeometryModelFixture, RadiationMemoryRespondsToStepChangeInVelocity)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;
  params.K_radiation_memory = Eigen::Vector3d(50.0, 50.0, 50.0);
  params.tau_radiation_memory = 0.3;

  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.ang_vel_world = Eigen::Vector3d::Zero();

  RadiationMemoryState memory;
  constexpr double kDt = 0.002;
  HydroWrenchBreakdown breakdown;

  // Settle at v_y = 0 first.
  state.lin_vel_world = Eigen::Vector3d::Zero();
  for (int i = 0; i < 1000; ++i) {
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory,
      kDt);
  }
  EXPECT_NEAR(breakdown.f_radiation_memory_body.y(), 0.0, 1e-6);

  // Step v_y from 0 to 0.5 m/s: lagged_vel_body is still ~0 (it only just converged to the OLD
  // v_y=0), so the very first post-step tick sees close to the maximum possible (v - lagged)
  // gap -- the washout term's response is LARGEST right after a step, not immediately after
  // settling (the opposite of the pre-step steady state checked above).
  state.lin_vel_world = Eigen::Vector3d(0.0, 0.5, 0.0);
  FossenHydrodynamics::compute(
    state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory, kDt);
  const double f_immediately_after_step = breakdown.f_radiation_memory_body.y();
  EXPECT_LT(f_immediately_after_step, -1.0)
    << "should be a large transient opposing +Y motion right after the velocity step";

  // Decays back toward zero after many time constants (lagged_vel_body catches up to the new
  // constant v_y, restoring the washout filter's zero-at-steady-state property).
  for (int i = 0; i < 5000; ++i) {
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory,
      kDt);
  }
  const double f_long_after_step = breakdown.f_radiation_memory_body.y();
  EXPECT_NEAR(f_long_after_step, 0.0, 1e-6);
  EXPECT_LT(std::abs(f_long_after_step), std::abs(f_immediately_after_step) * 1e-3);
}

// Finite/bounded under adversarial inputs (large velocities, tiny tau).
TEST_F(HydroGeometryModelFixture, RadiationMemoryBoundedAndFinite)
{
  const HydroGeometryModel geo = HydroGeometryModel::buildFromMjModel(model_, body_id_);
  HydroParams params;
  params.K_radiation_memory = Eigen::Vector3d(1e6, 1e6, 1e6);
  params.tau_radiation_memory = 1e-6;  // adversarially tiny

  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(0.5, 0.0, 0.5);
  state.orientation = Eigen::Quaterniond::Identity();
  state.lin_vel_world = Eigen::Vector3d(1e3, -1e3, 1e3);
  state.ang_vel_world = Eigen::Vector3d::Zero();

  RadiationMemoryState memory;
  HydroWrenchBreakdown breakdown;
  for (int i = 0; i < 100; ++i) {
    FossenHydrodynamics::compute(
      state, 0.0, params, HydroModelMode::kStonefishGeometryDriven, &geo, &breakdown, &memory,
      0.002);
    ASSERT_TRUE(breakdown.f_radiation_memory_body.allFinite()) << "iteration " << i;
    ASSERT_TRUE(memory.lagged_vel_body.allFinite()) << "iteration " << i;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
