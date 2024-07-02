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
#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/canopen_unit_conversions.h"
#include "actuator/firmware/barkour/common/interfaces/signal_filter_interface.h"
#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

pw::Result<DerivedSensorInformationUpdater>
DerivedSensorInformationUpdater::Build(
    SignalFilterInterface& velocity_filter, uint32_t encoder_counts_per_turn,
    int32_t encoder_count_at_zero_shaft_angle, float cycle_time) {
  if (cycle_time <= 0 || encoder_counts_per_turn == 0) {
    return pw::Status::InvalidArgument();
  }

  return DerivedSensorInformationUpdater(
      velocity_filter, encoder_counts_per_turn,
      encoder_count_at_zero_shaft_angle, cycle_time);
}

DerivedSensorInformationUpdater::DerivedSensorInformationUpdater(
    SignalFilterInterface& velocity_filter, uint32_t encoder_counts_per_turn,
    int32_t encoder_count_at_zero_shaft_angle, float cycle_time)
    : velocity_filter_(velocity_filter),
      encoder_counts_per_turn_(encoder_counts_per_turn),
      encoder_count_at_zero_shaft_angle_(encoder_count_at_zero_shaft_angle),
      cycle_time_(cycle_time) {}

void DerivedSensorInformationUpdater::Reset() {
  prev_encoder_position_ = std::nullopt;
  velocity_filter_.ResetFilter(0);
}

pw::Result<SensorDerivedQuantities>
DerivedSensorInformationUpdater::GetDerivedSensorInformation(
    const SensorReadings& sensor_readings) {
  SensorDerivedQuantities output;

  output.shaft_position =
      sensor_readings.encoder_position - encoder_count_at_zero_shaft_angle_;

  // In the first call since reset, there won't be a previous encoder position,
  // so we use the current one to estimate the velocity (i.e. it will be zero).
  float prev_encoder_position = sensor_readings.encoder_position;

  if (prev_encoder_position_.has_value()) {
    prev_encoder_position = prev_encoder_position_.value();
  }

  // Update prev_encoder_position_ before any further operations which might
  // fail.
  prev_encoder_position_ = sensor_readings.encoder_position;

  // Do velocity filtering in float space.
  PW_TRY_ASSIGN(float shaft_position_diff,
                Cia402PositionToPosition(
                    sensor_readings.encoder_position - prev_encoder_position,
                    encoder_counts_per_turn_));

  velocity_filter_.UpdateFilter(shaft_position_diff / cycle_time_);

  PW_TRY_ASSIGN(output.shaft_velocity,
                VelocityToCia402Velocity(velocity_filter_.GetFilteredValue(),
                                         encoder_counts_per_turn_));

  return output;
}

// Angle 0-2*pi.
float EncoderCountToElectricalAngle(int32_t count, uint32_t counts_per_turn,
                                    int32_t count_at_electrical_zero,
                                    uint16_t num_pole_pairs) {
  // Apply zero offset.
  int32_t count_zero = count - count_at_electrical_zero;

  // Wrap to be positive.
  if (count_zero < 0) {
    count_zero += counts_per_turn;
  }
  if (count_zero > static_cast<int32_t>(counts_per_turn)) {
    count_zero -= counts_per_turn;
  }

  // Electrical angle (0-21*2*pi).
  float angle = static_cast<float>(num_pole_pairs) * kTwoPi *
                static_cast<float>(count_zero) / counts_per_turn;

  // Electrical angle (0-2*pi).
  angle = fmodf(angle, kTwoPi);

  return angle;
}

}  // namespace barkour
