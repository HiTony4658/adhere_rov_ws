#include "adhere_controller/common/MultiRateDecimator.h"

#include <gtest/gtest.h>

using adhere_controller::MultiRateDecimator;

// CLAUDE.md Phase 3.1.1 Stage 3.2 / Phase 5.0 Stage 3: over 1.0s of simulation (500 ticks @
// 2.0ms), the inner channel must fire exactly 250 times (250Hz, dt=4.0ms) and the outer channel
// exactly 25 times (25Hz, dt=40.0ms), with zero non-uniform alternation.
TEST(MultiRateDecimator, DefaultRatiosProduce250InnerAnd25OuterOver500Ticks)
{
  MultiRateDecimator decimator(/*master_rate_hz=*/500);
  const int inner = decimator.addChannel("inner", 2);
  const int outer = decimator.addChannel("outer", 20);

  int inner_count = 0;
  int outer_count = 0;
  for (int i = 0; i < 500; ++i) {
    const auto due = decimator.tick();
    if (due[inner]) {
      ++inner_count;
    }
    if (due[outer]) {
      ++outer_count;
    }
  }

  EXPECT_EQ(inner_count, 250);
  EXPECT_EQ(outer_count, 25);
  EXPECT_DOUBLE_EQ(decimator.periodSeconds(inner), 0.004);
  EXPECT_DOUBLE_EQ(decimator.periodSeconds(outer), 0.040);
}

TEST(MultiRateDecimator, EveryOuterTickIsAlsoAnInnerTick)
{
  MultiRateDecimator decimator(500);
  const int inner = decimator.addChannel("inner", 2);
  const int outer = decimator.addChannel("outer", 20);
  for (int i = 0; i < 500; ++i) {
    const auto due = decimator.tick();
    if (due[outer]) {
      EXPECT_TRUE(due[inner]) << "outer-due tick " << i << " was not also inner-due";
    }
  }
}

TEST(MultiRateDecimator, FirstTickAfterConstructionOrResetIsAlwaysDue)
{
  MultiRateDecimator decimator(500);
  const int inner = decimator.addChannel("inner", 2);
  const int outer = decimator.addChannel("outer", 20);

  auto first = decimator.tick();
  EXPECT_TRUE(first[inner]);
  EXPECT_TRUE(first[outer]);

  // Burn a few ticks so tick_count_ is no longer tick-0-aligned, then reset.
  decimator.tick();
  decimator.tick();
  decimator.reset();
  auto after_reset = decimator.tick();
  EXPECT_TRUE(after_reset[inner]);
  EXPECT_TRUE(after_reset[outer]);
}

TEST(MultiRateDecimator, AlternateOuterDecimation25HzConfiguration)
{
  // CLAUDE.md Sec.3: outer_decimation can also be configured to 25 (-> 20Hz outer loop).
  MultiRateDecimator decimator(500);
  const int inner = decimator.addChannel("inner", 2);
  const int outer = decimator.addChannel("outer", 25);

  int inner_count = 0;
  int outer_count = 0;
  for (int i = 0; i < 500; ++i) {
    const auto due = decimator.tick();
    if (due[inner]) {
      ++inner_count;
    }
    if (due[outer]) {
      ++outer_count;
    }
  }

  EXPECT_EQ(inner_count, 250);
  EXPECT_EQ(outer_count, 20);
  EXPECT_DOUBLE_EQ(decimator.periodSeconds(outer), 0.050);
}

// Phase 5.0 Stage 3: the registry supports an arbitrary number of independently-rated channels
// (not just inner/outer) -- exercised here with the real diag (10Hz) + estimator (every tick)
// channels AdhereController registers.
TEST(MultiRateDecimator, ArbitraryNamedChannelsFireAtTheirOwnRate)
{
  MultiRateDecimator decimator(500);
  const int inner = decimator.addChannel("inner", 2);
  const int outer = decimator.addChannel("outer", 20);
  const int diag = decimator.addChannel("diag", 50);        // 500/10 -> 10Hz
  const int estimator = decimator.addChannel("estimator", 1);  // every tick

  int inner_count = 0;
  int outer_count = 0;
  int diag_count = 0;
  int estimator_count = 0;
  for (int i = 0; i < 500; ++i) {
    const auto due = decimator.tick();
    inner_count += due[inner] ? 1 : 0;
    outer_count += due[outer] ? 1 : 0;
    diag_count += due[diag] ? 1 : 0;
    estimator_count += due[estimator] ? 1 : 0;
  }

  EXPECT_EQ(inner_count, 250);
  EXPECT_EQ(outer_count, 25);
  EXPECT_EQ(diag_count, 10);
  EXPECT_EQ(estimator_count, 500);
  EXPECT_EQ(decimator.channelCount(), 4u);
  EXPECT_EQ(decimator.name(diag), "diag");
  EXPECT_DOUBLE_EQ(decimator.periodSeconds(diag), 0.1);
  EXPECT_DOUBLE_EQ(decimator.periodSeconds(estimator), 0.002);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
