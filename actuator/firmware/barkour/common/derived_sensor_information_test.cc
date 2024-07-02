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

#include "actuator/firmware/barkour/common/derived_sensor_information.h"

#include <cmath>

#include "actuator/firmware/barkour/common/interfaces/signal_filter_interface.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr float kNumericalTolerance = 1.0e-3;

class NoopFilter : public SignalFilterInterface {
 public:
  void ResetFilter(float initial_value) override { value_ = initial_value; }
  void UpdateFilter(float new_value) override { value_ = new_value; }
  float GetFilteredValue() const override { return value_; }

 private:
  float value_ = 0;
};

TEST(DerivedSensorInformationUpdaterTest, TestOutputsAsExpected) {
  // Build dependencies
  float step_time = 0.01;
  uint32_t encoder_counts_per_turn = 4096;
  int32_t encoder_count_at_zero_shaft_angle = 8942;

  NoopFilter velocity_filter;

  pw::Result<DerivedSensorInformationUpdater> maybe_updater =
      DerivedSensorInformationUpdater::Build(velocity_filter,
                                             encoder_counts_per_turn,
                                             encoder_count_at_zero_shaft_angle,
                                             step_time);

  ASSERT_TRUE(maybe_updater.ok());

  DerivedSensorInformationUpdater& updater = maybe_updater.value();

  updater.Reset();

  SensorReadings first_update_sensor_readings = {
      .raw_encoder_count = 2116,
      .encoder_position = 2116,
      .electrical_angle = 0.0,
      .quadrature_current = 0,
      .direct_current = 0,
      .phase_a_current = 1.5,
      .phase_b_current = -2.6,
      .phase_c_current = 1.1,
      .bus_voltage = 34.3,
      .thermistor_1 = 12.3,
      .thermistor_2 = 21.1,
      .thermistor_3 = 10.0,
      .thermistor_4 = 8.2,
      .sto_24v_safe_adc_voltage = 0.0,
      .imu_state = {{1, 2, 3}, {4, 5, 6}}};

  pw::Result<SensorDerivedQuantities> maybe_derived_sensor_readings =
      updater.GetDerivedSensorInformation(first_update_sensor_readings);

  ASSERT_TRUE(maybe_derived_sensor_readings.ok());

  // Check instantaneous sensor readings (i.e. not velocity).
  EXPECT_EQ(maybe_derived_sensor_readings->shaft_position,
            first_update_sensor_readings.encoder_position -
                encoder_count_at_zero_shaft_angle);

  int32_t enc_offset = first_update_sensor_readings.encoder_position;

  if (enc_offset < 0) {
    enc_offset += encoder_counts_per_turn;
  }

  if (enc_offset > (int32_t)encoder_counts_per_turn) {
    enc_offset -= encoder_counts_per_turn;
  }

  // First update since reset, velocity should be zero since the time between
  // reset and first update may not be consistent.
  EXPECT_EQ(maybe_derived_sensor_readings->shaft_velocity, 0);

  SensorReadings second_update_sensor_readings = first_update_sensor_readings;
  second_update_sensor_readings.encoder_position = 2146;

  maybe_derived_sensor_readings =
      updater.GetDerivedSensorInformation(second_update_sensor_readings);

  ASSERT_TRUE(maybe_derived_sensor_readings.ok());

  float expected_velocity = (second_update_sensor_readings.encoder_position -
                             first_update_sensor_readings.encoder_position) /
                            step_time;
  EXPECT_LT(std::fabs(maybe_derived_sensor_readings->shaft_velocity -
                      expected_velocity),
            kNumericalTolerance);
}

}  // namespace
}  // namespace barkour
