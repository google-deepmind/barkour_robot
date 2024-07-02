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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/ma732_encoder.h"

#include <cmath>
#include <cstdint>

#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr uint8_t kMa732EncoderResolutionBits = 16;

TEST(Ma732EncoderTest, CountsPerTurnIsCorrect) {
  Ma732Encoder& encoder = Ma732Encoder::Get();
  uint32_t counts_per_turn = encoder.GetCountsPerTurn();
  EXPECT_EQ(counts_per_turn, 1UL << kMa732EncoderResolutionBits);
}

TEST(Ma732EncoderTest, EncoderStateIsSane) {
  Ma732Encoder& encoder = Ma732Encoder::Get();

  EXPECT_TRUE(encoder.Update().ok());

  pw::Result<RotaryEncoderState> maybe_encoder_state =
      encoder.GetEncoderState();
  ASSERT_TRUE(maybe_encoder_state.ok());

  RotaryEncoderState& encoder_state = maybe_encoder_state.value();
  EXPECT_LE(encoder_state.raw_encoder_counts,
            1UL << kMa732EncoderResolutionBits);
}

TEST(Ma732EncoderTest, EncoderStatisticsAreBasicallySane) {
  Ma732Encoder& encoder = Ma732Encoder::Get();

  encoder.ResetStatistics();
  Ma732Stats statistics = encoder.GetStatistics();
  ASSERT_EQ(statistics.num_readings, 0);
  ASSERT_EQ(statistics.parity_error_count, 0);

  EXPECT_TRUE(encoder.Update().ok());

  pw::Result<RotaryEncoderState> maybe_encoder_state =
      encoder.GetEncoderState();
  ASSERT_TRUE(maybe_encoder_state.ok());

  statistics = encoder.GetStatistics();
  ASSERT_EQ(statistics.num_readings, 1);

  // While bad readings are certainly possible under normal operation, it
  // should not happen in this unit tests.
  ASSERT_EQ(statistics.parity_error_count, 0);

  encoder.ResetStatistics();
  statistics = encoder.GetStatistics();
  ASSERT_EQ(statistics.num_readings, 0);
  ASSERT_EQ(statistics.parity_error_count, 0);
}

}  // namespace
}  // namespace barkour
