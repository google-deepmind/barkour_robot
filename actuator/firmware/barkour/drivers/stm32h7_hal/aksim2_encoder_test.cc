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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/aksim2_encoder.h"

#include <cmath>

#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr uint8_t kEncoderResolutionBits = 19;

// Calling GetLatestReading before the encoder has been read should result
// in pw::Status::NotFound().
TEST(AksIm2EncoderTest, GetLatestReadingFailsNotFound) {
  AksIm2Encoder& encoder = AksIm2Encoder::Get();
  auto reading_result = encoder.GetLatestReading();
  EXPECT_EQ(reading_result.status(), pw::Status::NotFound());
}

TEST(AksIm2EncoderTest, PollingRead) {
  AksIm2Encoder& encoder = AksIm2Encoder::Get();
  auto reading_result = encoder.GetReading();
  EXPECT_EQ(reading_result.status(), pw::OkStatus());
}

TEST(AksIm2EncoderTest, EncoderStatisticsWorks) {
  AksIm2Encoder& encoder = AksIm2Encoder::Get();
  auto statistics = encoder.GetStatistics();
  auto num_readings = statistics.readings_counter;

  auto reading_result = encoder.GetReading();
  statistics = encoder.GetStatistics();
  EXPECT_EQ(reading_result.status(), pw::OkStatus());
  EXPECT_EQ(statistics.readings_counter, num_readings + 1);

  encoder.ResetStatistics();
  statistics = encoder.GetStatistics();
  EXPECT_EQ(statistics.readings_counter, 0u);
  EXPECT_EQ(statistics.error_counter, 0u);
  EXPECT_EQ(statistics.warning_counter, 0u);
  EXPECT_EQ(statistics.crc_error_counter, 0u);
}

TEST(AksIm2EncoderTest, EncoderPositionIsSane) {
  AksIm2Encoder& encoder = AksIm2Encoder::Get();
  auto reading_result = encoder.GetReading();
  EXPECT_EQ(reading_result.status(), pw::OkStatus());
  auto reading = reading_result.value();
  EXPECT_GE(reading.encoder_position, 0u);
  EXPECT_LE(reading.encoder_position, std::pow(2, kEncoderResolutionBits));
}

}  // namespace
}  // namespace barkour
