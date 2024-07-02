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

#include "actuator/firmware/barkour/devices/imu/lsm9ds1/lsm9ds1_imu.h"

#include <cmath>

#include "pw_log/log.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {
namespace {

// The datasheet for the LSM9DS1 does not make any guarantees regarding the
// accuracy of the accelerometer in terms of sensitivities and offsets.
// A typical range is given in the datasheet, but in practice some sensors have
// been discovered to fall outside of this range, thus the range of acceptable
// vector magnitudes associated with this test case is quite large.

// Minimum and maximum expected magnitudes for acceleration, in
// meters / seconds^2.
constexpr float kAccelerometerMinimumExpectedMagnitude = 0.50f;
constexpr float kAccelerometerMaximumExpectedMagnitude = 15.0f;

// Minimum and maximum expected magnitudes for the gyro, in radians/second.
constexpr float kGyroscopeMinimumExpectedMagnitude = 0.50f;

// Time delay between test readings, for tests that require multiple readings.
constexpr float kTestReadingDelayMs = 10.0f;

TEST(Lsm9Ds1ImuTest, CanInitialize) {
  Lsm9Ds1Imu imu = Lsm9Ds1Imu();

  EXPECT_EQ(imu.Initialize(), pw::OkStatus());
}

TEST(Lsm9Ds1ImuTest, CanGetImuState) {
  Lsm9Ds1Imu imu = Lsm9Ds1Imu();
  EXPECT_EQ(imu.Initialize(), pw::OkStatus());
  pw::Result<ImuState> maybe_reading = imu.GetImuState();
  EXPECT_TRUE(maybe_reading.ok());
}

TEST(Lsm9Ds1ImuTest, AccelerometerMagnitudeIsInRange) {
  Lsm9Ds1Imu imu = Lsm9Ds1Imu();
  EXPECT_EQ(imu.Initialize(), pw::OkStatus());

  // Take 10 readings, for good measure.
  for (int i = 0; i < 10; i++) {
    pw::Result<ImuState> maybe_reading = imu.GetImuState();
    EXPECT_TRUE(maybe_reading.ok());

    float accel_magnitude =
        sqrtf(powf(maybe_reading.value().accelerometer[0], 2) +
              powf(maybe_reading.value().accelerometer[1], 2) +
              powf(maybe_reading.value().accelerometer[2], 2));
    EXPECT_GT(accel_magnitude, 5.0f);
    EXPECT_LT(accel_magnitude, 15.0f);
    HAL_Delay(kTestReadingDelayMs);
  }
}

TEST(Lsm9Ds1ImuTest, GyroscopeMagnitudeIsInRange) {
  Lsm9Ds1Imu imu = Lsm9Ds1Imu();
  EXPECT_EQ(imu.Initialize(), pw::OkStatus());

  // Take 10 readings, for good measure.
  for (int i = 0; i < 10; i++) {
    pw::Result<ImuState> maybe_reading = imu.GetImuState();
    EXPECT_TRUE(maybe_reading.ok());
    float gyro_magnitude = sqrtf(powf(maybe_reading.value().gyroscope[0], 2) +
                                 powf(maybe_reading.value().gyroscope[1], 2) +
                                 powf(maybe_reading.value().gyroscope[2], 2));
    EXPECT_LT(gyro_magnitude, 1.0f);
    HAL_Delay(kTestReadingDelayMs);
  }
}

}  // namespace
}  // namespace barkour
