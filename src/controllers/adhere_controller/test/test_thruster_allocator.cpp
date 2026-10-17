#include "adhere_controller/common/ModelParams.h"
#include "adhere_controller/common/ThrusterAllocator.h"

#include <gtest/gtest.h>

using adhere_controller::buildThrusterAllocationMatrix;
using adhere_controller::buildThrusterAllocationPseudoInverse;
using adhere_controller::Mat6;
using adhere_controller::ModelParams;
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
