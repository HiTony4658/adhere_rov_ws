#include <gtest/gtest.h>

#include <cmath>

#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using adhere_mujoco_hardware::FossenHydrodynamics;
using adhere_mujoco_hardware::HydroGeometryModel;
using adhere_mujoco_hardware::HydroModelMode;
using adhere_mujoco_hardware::HydroParams;
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
