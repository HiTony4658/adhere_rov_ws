#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/dynamics/ModelParams.h"

#include <gtest/gtest.h>

using adhere_controller::buildContactMap;
using adhere_controller::buildKernConstraintMatrix;
using adhere_controller::buildPadNormalMap;
using adhere_controller::buildPadNormalMapPseudoInverse;
using adhere_controller::buildThrusterAllocationMatrix;
using adhere_controller::buildThrusterAllocationPseudoInverse;
using adhere_controller::Mat6;
using adhere_controller::ModelParams;
using adhere_controller::padHalfSpans;
using adhere_controller::Vec4;
using adhere_controller::Vec6;
using adhere_controller::Vec8;

namespace
{
// ModelParams' compiled-in defaults are rov_wall.xml's real geometry (CLAUDE.md Phase 3.3),
// used directly here rather than duplicating the literals.
const ModelParams kDefaultModelParams;
const auto & kDefaultPos = kDefaultModelParams.thruster_pos;
const auto & kDefaultDir = kDefaultModelParams.thruster_dir;
}  // namespace

// CLAUDE.md Phase 3.2 Stage 3.2: rank(B) == 6, so B*B^dagger must reproduce I_6 to within 1e-4
// despite the lambda_reg=1e-6 damping term.
TEST(ThrusterAllocator, BTimesPseudoInverseIsIdentity)
{
  const auto b = buildThrusterAllocationMatrix(kDefaultPos, kDefaultDir);
  const auto b_pinv = buildThrusterAllocationPseudoInverse(b);

  const Mat6 product = b * b_pinv;
  const Mat6 identity = Mat6::Identity();
  EXPECT_LT((product - identity).cwiseAbs().maxCoeff(), 1e-4);
}

// CLAUDE.md Phase 3.2 Stage 3.2: a pure sway (Fy) or pure heave (Fz) wrench command must
// allocate to zero parasitic Tx/Ty/Tz moment -- rov_wall.xml's zero-lever-arm geometry (all 8
// thruster sites at body X=0, same X as the CoM) guarantees this exactly, unlike the
// illustrative X=-0.10m lever-arm placeholder in CLAUDE.md Sec.4.3.
TEST(ThrusterAllocator, PureSwayProducesZeroParasiticMoment)
{
  const auto b = buildThrusterAllocationMatrix(kDefaultPos, kDefaultDir);
  const auto b_pinv = buildThrusterAllocationPseudoInverse(b);

  Vec6 wrench_sway;
  wrench_sway << 0.0, 25.0, 0.0, 0.0, 0.0, 0.0;
  const Vec8 u = b_pinv * wrench_sway;
  const Vec6 produced = b * u;

  EXPECT_NEAR(produced(1), 25.0, 1e-3);
  EXPECT_NEAR(produced(0), 0.0, 1e-3);
  EXPECT_NEAR(produced(3), 0.0, 1e-3);
  EXPECT_NEAR(produced(4), 0.0, 1e-3);
  EXPECT_NEAR(produced(5), 0.0, 1e-3);
}

TEST(ThrusterAllocator, PureHeaveProducesZeroParasiticMoment)
{
  const auto b = buildThrusterAllocationMatrix(kDefaultPos, kDefaultDir);
  const auto b_pinv = buildThrusterAllocationPseudoInverse(b);

  Vec6 wrench_heave;
  wrench_heave << 0.0, 0.0, 30.0, 0.0, 0.0, 0.0;
  const Vec8 u = b_pinv * wrench_heave;
  const Vec6 produced = b * u;

  EXPECT_NEAR(produced(2), 30.0, 1e-3);
  EXPECT_NEAR(produced(0), 0.0, 1e-3);
  EXPECT_NEAR(produced(3), 0.0, 1e-3);
  EXPECT_NEAR(produced(4), 0.0, 1e-3);
  EXPECT_NEAR(produced(5), 0.0, 1e-3);
}

// --- Phase 3.6 Stage 2.2/3.1: pre-MPPI A_contact/C_kern verification -------------------------
TEST(ThrusterAllocator, ContactMapReproducesEqualPressingLoadExactly)
{
  const auto b = buildThrusterAllocationMatrix(kDefaultPos, kDefaultDir);
  const auto g = buildPadNormalMap(kDefaultModelParams.pad_pos);
  const auto g_pinv = buildPadNormalMapPseudoInverse(g);
  const auto a_contact = buildContactMap(g_pinv, b);

  // CLAUDE.md Sec.3's example: equal pressing thrusts u[0:4]=25N (the 4 normal thrusters
  // co-located with the 4 pads) must yield A_contact*u = [25, 25, 25, 25]^T N exactly.
  Vec8 u = Vec8::Zero();
  u(0) = 25.0;
  u(1) = 25.0;
  u(2) = 25.0;
  u(3) = 25.0;

  const Vec4 f_n = a_contact * u;
  EXPECT_NEAR(f_n(0), 25.0, 1e-3);
  EXPECT_NEAR(f_n(1), 25.0, 1e-3);
  EXPECT_NEAR(f_n(2), 25.0, 1e-3);
  EXPECT_NEAR(f_n(3), 25.0, 1e-3);
}

TEST(ThrusterAllocator, KernConstraintSatisfiedForSymmetricPressingLoad)
{
  const auto b = buildThrusterAllocationMatrix(kDefaultPos, kDefaultDir);
  const auto spans = padHalfSpans(kDefaultModelParams.pad_pos);
  const auto c_kern = buildKernConstraintMatrix(
    b, spans.first, spans.second, kDefaultModelParams.kern_eps_margin);

  // A perfectly symmetric pressing load (CoP at the geometric center) must sit strictly inside
  // the admissible kern for every one of the 4 edge constraints.
  Vec8 u = Vec8::Zero();
  u(0) = 25.0;
  u(1) = 25.0;
  u(2) = 25.0;
  u(3) = 25.0;

  const Eigen::Matrix<double, 4, 1> result = c_kern * u;
  for (int i = 0; i < 4; ++i) {
    EXPECT_LT(result(i), 0.0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
