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

#include "actuator/firmware/barkour/common/read_sensors.h"

#include <cmath>
#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "actuator/firmware/barkour/devices/imu/testing/fake_imu.h"
#include "actuator/firmware/barkour/drivers/testing/fake_adc.h"
#include "actuator/firmware/barkour/drivers/testing/fake_foc.h"
#include "actuator/firmware/barkour/drivers/testing/fake_gate_driver.h"
#include "actuator/firmware/barkour/drivers/testing/fake_rotary_encoder.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(SensorReaderTest, InternalErrorReturnedIfNoMultiturnEncoderCounts) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  EXPECT_EQ(sensor_reader.GetLatestReadingsFromAllSensors().status(),
            pw::Status::Internal());
}

TEST(SensorReaderTest, CurrentsSampledIfGateDriverPoweredOn) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  // Set some multiturn encoder counts to avoid error path where they are
  // missing.
  fake_encoder.multiturn_encoder_counts_ = 100;

  // Set some nonzero injected sample.
  fake_adc.SetInjectedSamples(200, 200, 200);
  fake_foc.SetState(100, 200, 300);

  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();
  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();

  ASSERT_TRUE(maybe_sensor_readings.ok());

  EXPECT_NE(maybe_sensor_readings->phase_a_current, 0);
  EXPECT_NE(maybe_sensor_readings->phase_b_current, 0);
  EXPECT_NE(maybe_sensor_readings->phase_c_current, 0);
}

TEST(SensorReaderTest, ErrorFromAdcPassedThrough) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  // Set some multiturn encoder counts to avoid error path where they are
  // missing.
  fake_encoder.multiturn_encoder_counts_ = 100;

  fake_adc.return_status_ = pw::Status::Unavailable();

  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();

  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();
  EXPECT_EQ(maybe_sensor_readings.status(), pw::Status::Unavailable());
}

TEST(SensorReaderTest, ErrorFromImuPassedThrough) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  // Set some multiturn encoder counts to avoid error path where they are
  // missing.
  fake_encoder.multiturn_encoder_counts_ = 100;

  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();

  fake_imu.set_status(pw::Status::FailedPrecondition());

  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();

  EXPECT_EQ(maybe_sensor_readings.status(), pw::Status::FailedPrecondition());
}

TEST(SensorReaderTest, ImuPassedThroughOk) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  // Set some multiturn encoder counts to avoid error path where they are
  // missing.
  fake_encoder.multiturn_encoder_counts_ = 100;

  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();
  ImuState imu_state = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};

  fake_imu.set_state(imu_state);  // set some readings

  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();

  EXPECT_EQ(maybe_sensor_readings.status(), pw::OkStatus());

  SensorReadings readings = maybe_sensor_readings.value();

  EXPECT_EQ(readings.imu_state.accelerometer[0], 1.0f);
  EXPECT_EQ(readings.imu_state.accelerometer[1], 2.0f);
  EXPECT_EQ(readings.imu_state.accelerometer[2], 3.0f);

  EXPECT_EQ(readings.imu_state.gyroscope[0], 4.0f);
  EXPECT_EQ(readings.imu_state.gyroscope[1], 5.0f);
  EXPECT_EQ(readings.imu_state.gyroscope[2], 6.0f);
}

TEST(SensorReaderTest, InitialEncoderDataLossErrorPassedThrough) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu, 5);

  // Set some multiturn encoder counts to avoid error path where they are
  // missing.
  fake_encoder.multiturn_encoder_counts_ = 100;

  fake_encoder.return_status_ = pw::Status::DataLoss();

  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();

  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();
  EXPECT_EQ(maybe_sensor_readings.status(), pw::Status::DataLoss());
}

TEST(SensorReaderTest, EncoderDataLossCounterWorksAsExpected) {
  FakeRotaryEncoder fake_encoder(1024);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;
  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  int max_consecutive_encoder_data_loss_errors = 5;
  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu,
                             max_consecutive_encoder_data_loss_errors);

  fake_encoder.multiturn_encoder_counts_ = 100;
  fake_gate_driver.current_state_ = GateDriverState::PowerOnGateEnabled();

  // Get a good initial set of readings.
  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();
  ASSERT_TRUE(maybe_sensor_readings.ok());

  // Now set the encoder status to a data loss error.
  fake_encoder.return_status_ = pw::Status::DataLoss();

  // Up to `max_consecutive_data_loss_errors` sensor readings succeed.
  for (int i = 0; i < max_consecutive_encoder_data_loss_errors; ++i) {
    maybe_sensor_readings = sensor_reader.GetLatestReadingsFromAllSensors();
    ASSERT_TRUE(maybe_sensor_readings.ok());
    EXPECT_EQ(maybe_sensor_readings->encoder_position, 100);
  }

  // The next sensor read should fail.
  maybe_sensor_readings = sensor_reader.GetLatestReadingsFromAllSensors();
  EXPECT_EQ(maybe_sensor_readings.status(), pw::Status::DataLoss());

  // Run the whole sequence again, to check the counter is reset after a good
  // reading.
  fake_encoder.multiturn_encoder_counts_ = 200;
  fake_encoder.return_status_ = pw::OkStatus();

  ASSERT_TRUE(sensor_reader.GetLatestReadingsFromAllSensors().ok());

  fake_encoder.return_status_ = pw::Status::DataLoss();

  // Up to `max_consecutive_data_loss_errors` sensor readings succeed.
  for (int i = 0; i < max_consecutive_encoder_data_loss_errors; ++i) {
    maybe_sensor_readings = sensor_reader.GetLatestReadingsFromAllSensors();
    ASSERT_TRUE(maybe_sensor_readings.ok());
    EXPECT_EQ(maybe_sensor_readings->encoder_position, 200);
  }

  maybe_sensor_readings = sensor_reader.GetLatestReadingsFromAllSensors();
  EXPECT_EQ(maybe_sensor_readings.status(), pw::Status::DataLoss());
}

TEST(SensorReaderTest, FocCalculatedValues) {
  uint32_t num_pole_pairs = 21;
  uint32_t enc_offset = 512;
  uint32_t encoder_counts_per_turn = 1024;

  FakeRotaryEncoder fake_encoder(encoder_counts_per_turn);
  FakeAdc fake_adc;
  FakeGateDriver fake_gate_driver;

  fake_adc.SetInjectedSamples(512, 100, 300);
  fake_adc.SetInjectedChannels(PhaseSampleSelection::kAB);

  FakeFoc fake_foc;
  // Initialize IMU before being passed to SensorReader.
  FakeImu fake_imu;
  fake_imu.Initialize();

  fake_encoder.SetRawEncoderCounts(200);
  fake_encoder.multiturn_encoder_counts_ = 200;

  (void)fake_encoder.Update();

  float expect_angle = kTwoPi * (float)num_pole_pairs * (float)enc_offset /
                       (float)encoder_counts_per_turn;
  expect_angle = fmodf(expect_angle, kTwoPi);

  fake_foc.SetState(expect_angle);
  fake_foc.SetState(100, 200);

  int max_consecutive_encoder_data_loss_errors = 5;
  SensorReader sensor_reader(fake_adc, fake_encoder, fake_foc, fake_imu,
                             max_consecutive_encoder_data_loss_errors);

  // Get a good initial set of readings.
  pw::Result<SensorReadings> maybe_sensor_readings =
      sensor_reader.GetLatestReadingsFromAllSensors();

  ASSERT_TRUE(maybe_sensor_readings.ok());

  SensorReadings sensor_readings = maybe_sensor_readings.value();

  EXPECT_EQ(sensor_readings.electrical_angle, expect_angle);

  EXPECT_EQ(sensor_readings.direct_current, 200);
  EXPECT_EQ(sensor_readings.quadrature_current, 100);
}

}  // namespace
}  // namespace barkour
