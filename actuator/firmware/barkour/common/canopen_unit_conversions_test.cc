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

#include "actuator/firmware/barkour/common/canopen_unit_conversions.h"

#include <cmath>
#include <limits>

#include "actuator/firmware/barkour/common/math_constants.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(CanopenUnitConversionTest, CurrentToCia402Torque) {
  pw::Result<int16_t> maybe_value = CurrentToCia402Torque(100.5f, 590, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 29647);

  maybe_value = CurrentToCia402Torque(-100.5f, 590, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -29647);

  maybe_value = CurrentToCia402Torque(-100.5f, 590, 10000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -5929);

  maybe_value = CurrentToCia402Torque(0, 590, 10000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = CurrentToCia402Torque(100.0f, 0, 10000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Torque(100.0f, 123, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Torque(266.0f, 123, 1000);
  EXPECT_TRUE(maybe_value.ok());
  maybe_value = CurrentToCia402Torque(-266.0f, 123, 1000);
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = CurrentToCia402Torque(267.0f, 123, 1000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Torque(-267.0f, 123, 1000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());
}

TEST(CanopenUnitConversionTest, CurrentToCia402Current) {
  pw::Result<int16_t> maybe_value = CurrentToCia402Current(50.5, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 25250);

  maybe_value = CurrentToCia402Current(1.23f, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 1230);

  maybe_value = CurrentToCia402Current(-2.46f, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -1230);

  maybe_value = CurrentToCia402Current(-50.5f, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -25250);

  maybe_value = CurrentToCia402Current(-101, 4000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -25250);

  maybe_value = CurrentToCia402Current(2e16f, 2000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Current(65.5f, 2000);
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = CurrentToCia402Current(65.6f, 2000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Current(0, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = CurrentToCia402Current(-10.3423f, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -5171);

  maybe_value = CurrentToCia402Current(0, 1234);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);
}

TEST(CanopenUnitConversionTest, Cia402TorqueToCia402CurrentAndBack) {
  pw::Result<int16_t> maybe_value = CurrentToCia402Torque(1.234f, 1000, 1000);
  ASSERT_TRUE(maybe_value.ok());
  pw::Result<float> maybe_result =
      Cia402TorqueToCurrent(maybe_value.value(), 1000, 1000);
  EXPECT_EQ(maybe_result.value(), 1.234f);

  maybe_value = CurrentToCia402Torque(-10.234f, 5000, 2123);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402TorqueToCurrent(maybe_value.value(), 5000, 2123);
  EXPECT_LT(std::fabs(maybe_result.value() + 10.234), 0.01f);

  maybe_value = CurrentToCia402Torque(0, 5000, 2123);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402TorqueToCurrent(maybe_value.value(), 5000, 2123);
  EXPECT_LT(std::fabs(maybe_result.value()), 0.001);
}

TEST(CanopenUnitConversionTest, CurrentToCia402CurrentAndBack) {
  pw::Result<int16_t> maybe_value = CurrentToCia402Current(1.234f, 1000);
  ASSERT_TRUE(maybe_value.ok());
  pw::Result<float> maybe_result =
      Cia402CurrentToCurrent(maybe_value.value(), 1000);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.001f);

  maybe_value = CurrentToCia402Current(1.234f, 12345);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402CurrentToCurrent(maybe_value.value(), 12345);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.1f);

  maybe_value = CurrentToCia402Current(0, 12345);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402CurrentToCurrent(maybe_value.value(), 12345);
  EXPECT_LT(std::fabs(maybe_result.value()), 0.001f);
}

TEST(CanopenUnitConversionTest, Cia402CurrentToCurrent) {
  pw::Result<float> maybe_value = Cia402CurrentToCurrent(0, 32768);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = Cia402CurrentToCurrent(0, std::numeric_limits<int16_t>::max());
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = Cia402CurrentToCurrent(0, std::numeric_limits<int16_t>::min());
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = Cia402CurrentToCurrent(std::numeric_limits<int16_t>::max(),
                                       std::numeric_limits<uint32_t>::max());
  ASSERT_TRUE(maybe_value.ok());
  // Not a lot of precision left, but still about 5 significant digits.
  EXPECT_LT(std::fabs(maybe_value.value() - 140737488.355328f), 1e5f);

  maybe_value = Cia402CurrentToCurrent(std::numeric_limits<int16_t>::min(),
                                       std::numeric_limits<uint32_t>::max());
  ASSERT_TRUE(maybe_value.ok());
  // Not a lot of precision left, but still about 5 significant digits.
  EXPECT_LT(std::fabs(maybe_value.value() + 140737488.355328f), 1e5f);

  maybe_value = Cia402CurrentToCurrent(1, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0.001f);

  maybe_value = Cia402CurrentToCurrent(-1, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -0.001f);

  maybe_value = Cia402CurrentToCurrent(-12345, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 12.345f), 0.001);

  maybe_value = Cia402CurrentToCurrent(-12345, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 12.345f * 2), 0.001);
}

TEST(CanopenUnitConversionTest, Cia402TorqueToCurrent) {
  pw::Result<float> maybe_value = Cia402TorqueToCurrent(29647, 590, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() - 100.5f), 1e-2f);

  maybe_value = Cia402TorqueToCurrent(-29647, 590, 2000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 100.5f), 1e-2f);

  maybe_value = Cia402TorqueToCurrent(-5929, 590, 10000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 100.5f), 1e-2f);

  maybe_value = Cia402TorqueToCurrent(0, 590, 10000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = Cia402TorqueToCurrent(1.0f, 0, 10000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = Cia402TorqueToCurrent(1.0f, 10, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value =
      Cia402TorqueToCurrent(std::numeric_limits<int16_t>::max(), 123, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() - 266.406), 1e-2f);

  maybe_value =
      Cia402TorqueToCurrent(std::numeric_limits<int16_t>::min(), 123, 1000);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 266.406), 1e-2f);
}

TEST(CanopenUnitConversionTest, Cia402PositionToPosition) {
  pw::Result<float> maybe_value = Cia402PositionToPosition(14363, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() - 22.034f), 1e-2f);

  maybe_value = Cia402PositionToPosition(-14363, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 22.034f), 1e-2f);

  maybe_value = Cia402PositionToPosition(-1849, 16384);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 0.7091f), 1e-2f);

  maybe_value = Cia402PositionToPosition(0, 8192);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = Cia402PositionToPosition(100, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());
}

TEST(CanopenUnitConversionTest, PositionToCia402Position) {
  pw::Result<int32_t> maybe_value = PositionToCia402Position(50.5, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 32921);

  maybe_value = PositionToCia402Position(0, 8192);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = PositionToCia402Position(-10.3423f, 16384);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -26969);

  maybe_value = PositionToCia402Position(kTwoPi, 16);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 16);

  maybe_value = PositionToCia402Position(2e16f, 2000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  // int32 overflow is between 2.1e9 and 2.2e9.
  maybe_value = PositionToCia402Position(2.1f * kTwoPi, 1000000000);
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = PositionToCia402Position(2.2f * kTwoPi, 1000000000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = PositionToCia402Position(0, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());
}

TEST(CanopenUnitConversionTest, PositionToCia402PositionAndBack) {
  pw::Result<int16_t> maybe_value = PositionToCia402Position(1.234f, 4096);
  ASSERT_TRUE(maybe_value.ok());
  pw::Result<float> maybe_result =
      Cia402PositionToPosition(maybe_value.value(), 4096);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.001f);

  maybe_value = PositionToCia402Position(1.234f, 65536);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402PositionToPosition(maybe_value.value(), 65536);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.1f);

  maybe_value = PositionToCia402Position(0, 65536);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402PositionToPosition(maybe_value.value(), 65536);
  EXPECT_LT(std::fabs(maybe_result.value()), 0.001f);
}

// Repeat same tests for velocity
TEST(CanopenUnitConversionTest, Cia402VelocityToVelocity) {
  pw::Result<float> maybe_value = Cia402VelocityToVelocity(14363, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() - 22.034f), 1e-2f);

  maybe_value = Cia402VelocityToVelocity(-14363, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 22.034f), 1e-2f);

  maybe_value = Cia402VelocityToVelocity(-1849, 16384);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_LT(std::fabs(maybe_value.value() + 0.7091f), 1e-2f);

  maybe_value = Cia402VelocityToVelocity(0, 8192);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = Cia402VelocityToVelocity(100, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());
}

TEST(CanopenUnitConversionTest, VelocityToCia402Velocity) {
  pw::Result<int32_t> maybe_value = VelocityToCia402Velocity(50.5, 4096);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 32921);

  maybe_value = VelocityToCia402Velocity(0, 8192);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 0);

  maybe_value = VelocityToCia402Velocity(-10.3423f, 16384);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), -26969);

  maybe_value = VelocityToCia402Velocity(kTwoPi, 16);
  ASSERT_TRUE(maybe_value.ok());
  EXPECT_EQ(maybe_value.value(), 16);

  maybe_value = VelocityToCia402Velocity(2e16f, 2000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  // int32 overflow is between 2.1e9 and 2.2e9.
  maybe_value = VelocityToCia402Velocity(2.1f * kTwoPi, 1000000000);
  EXPECT_TRUE(maybe_value.ok());

  maybe_value = VelocityToCia402Velocity(2.2f * kTwoPi, 1000000000);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());

  maybe_value = VelocityToCia402Velocity(0, 0);
  EXPECT_EQ(maybe_value.status(), pw::Status::InvalidArgument());
}

TEST(CanopenUnitConversionTest, VelocityToCia402VelocityAndBack) {
  pw::Result<int16_t> maybe_value = VelocityToCia402Velocity(1.234f, 4096);
  ASSERT_TRUE(maybe_value.ok());
  pw::Result<float> maybe_result =
      Cia402VelocityToVelocity(maybe_value.value(), 4096);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.001f);

  maybe_value = VelocityToCia402Velocity(1.234f, 65536);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402VelocityToVelocity(maybe_value.value(), 65536);
  EXPECT_LT(std::fabs(maybe_result.value() - 1.234f), 0.1f);

  maybe_value = VelocityToCia402Velocity(0, 65536);
  ASSERT_TRUE(maybe_value.ok());
  maybe_result = Cia402VelocityToVelocity(maybe_value.value(), 65536);
  EXPECT_LT(std::fabs(maybe_result.value()), 0.001f);
}

}  // namespace
}  // namespace barkour
