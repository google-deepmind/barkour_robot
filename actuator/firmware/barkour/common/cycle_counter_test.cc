// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ==============================================================================

#include "actuator/firmware/barkour/common/cycle_counter.h"

#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "stm32h7xx_hal.h"  // NOLINT

namespace barkour {
namespace {

TEST(CycleCounterTest, InitialValuesCorrectAfterReset) {
  auto& counter = CycleCounter::Get();
  counter.Disable();
  counter.Reset();
  EXPECT_EQ(counter.IsEnabled(), false);
  EXPECT_EQ(counter.GetCount(), 0u);
}

TEST(CycleCounterTest, DoesNotCountWhenDisabled) {
  auto& counter = CycleCounter::Get();
  counter.Disable();

  // Counter is disabled, and should not count up.
  uint32_t count = counter.GetCount();
  EXPECT_EQ(counter.GetCount(), count);
}

TEST(CycleCounterTest, CountsWhenEnabled) {
  auto& counter = CycleCounter::Get();
  counter.Disable();
  counter.Reset();

  // Enable the counter.
  counter.Enable();
  EXPECT_EQ(counter.IsEnabled(), true);

  // Take two readings and check that the count has increased.
  uint32_t count1 = counter.GetCount();
  uint32_t count2 = counter.GetCount();
  EXPECT_GT(count2, count1);
}

TEST(CycleCounterTest, CounterResetWorks) {
  auto& counter = CycleCounter::Get();
  counter.Disable();
  counter.Reset();

  counter.Enable();
  EXPECT_GT(counter.GetCount(), 0u);
  counter.Disable();
  counter.Reset();
  EXPECT_EQ(counter.GetCount(), 0u);
}

TEST(CycleCounterTest, BlockForMicrosecondsWorks) {
  constexpr int kMicroSecondsToWait = 100;
  auto& counter = CycleCounter::Get();
  counter.Enable();

  uint32_t count1 = counter.GetCount();
  counter.BlockForMicroseconds(kMicroSecondsToWait);
  uint32_t count2 = counter.GetCount();

  uint32_t cycles_elapsed = count2 - count1;
  uint32_t expected_min_cycles_elapsed =
      kMicroSecondsToWait * (HAL_RCC_GetHCLKFreq() / 1000000);
  uint32_t expected_max_cycles_elapsed =
      (kMicroSecondsToWait + 1) * (HAL_RCC_GetHCLKFreq() / 1000000);
  EXPECT_GE(cycles_elapsed, expected_min_cycles_elapsed);
  EXPECT_LT(cycles_elapsed, expected_max_cycles_elapsed);
}

}  // namespace
}  // namespace barkour
