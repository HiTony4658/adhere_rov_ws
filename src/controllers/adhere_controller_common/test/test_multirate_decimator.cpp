#include "adhere_controller_common/common/MultiRateDecimator.h"

#include <gtest/gtest.h>

using adhere_controller_common::MultiRateDecimator;

// CLAUDE.md Phase 3.1.1 Stage 3.2: over 1.0s of simulation (500 ticks @ 2.0ms), the inner loop
// must fire exactly 250 times (250Hz, dt=4.0ms) and the outer loop exactly 25 times (25Hz,
// dt=40.0ms), with zero non-uniform alternation.
TEST(MultiRateDecimator, DefaultRatiosProduce250InnerAnd25OuterOver500Ticks)
{
  MultiRateDecimator decimator(/*cm_update_rate=*/500, /*inner_decimation=*/2, /*outer_decimation=*/20);

  int inner_count = 0;
  int outer_count = 0;
  for (int i = 0; i < 500; ++i) {
    const auto result = decimator.tick();
    if (result.inner_due) {
      ++inner_count;
    }
    if (result.outer_due) {
      ++outer_count;
    }
  }

  EXPECT_EQ(inner_count, 250);
  EXPECT_EQ(outer_count, 25);
  EXPECT_DOUBLE_EQ(decimator.innerPeriodSeconds(), 0.004);
  EXPECT_DOUBLE_EQ(decimator.outerPeriodSeconds(), 0.040);
}

TEST(MultiRateDecimator, EveryOuterTickIsAlsoAnInnerTick)
{
  MultiRateDecimator decimator(500, 2, 20);
  for (int i = 0; i < 500; ++i) {
    const auto result = decimator.tick();
    if (result.outer_due) {
      EXPECT_TRUE(result.inner_due) << "outer-due tick " << i << " was not also inner-due";
    }
  }
}

TEST(MultiRateDecimator, FirstTickAfterConstructionOrResetIsAlwaysDue)
{
  MultiRateDecimator decimator(500, 2, 20);
  auto first = decimator.tick();
  EXPECT_TRUE(first.inner_due);
  EXPECT_TRUE(first.outer_due);

  // Burn a few ticks so tick_count_ is no longer tick-0-aligned, then reset.
  decimator.tick();
  decimator.tick();
  decimator.reset();
  auto after_reset = decimator.tick();
  EXPECT_TRUE(after_reset.inner_due);
  EXPECT_TRUE(after_reset.outer_due);
}

TEST(MultiRateDecimator, AlternateOuterDecimation25HzConfiguration)
{
  // CLAUDE.md Sec.3: outer_decimation can also be configured to 25 (-> 20Hz outer loop).
  MultiRateDecimator decimator(500, 2, 25);

  int inner_count = 0;
  int outer_count = 0;
  for (int i = 0; i < 500; ++i) {
    const auto result = decimator.tick();
    if (result.inner_due) {
      ++inner_count;
    }
    if (result.outer_due) {
      ++outer_count;
    }
  }

  EXPECT_EQ(inner_count, 250);
  EXPECT_EQ(outer_count, 20);
  EXPECT_DOUBLE_EQ(decimator.outerPeriodSeconds(), 0.050);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
