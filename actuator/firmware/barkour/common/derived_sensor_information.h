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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DERIVED_SENSOR_INFORMATION_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DERIVED_SENSOR_INFORMATION_H_

#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/signal_filter_interface.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"

namespace barkour {

// Utility class for calculating useful information and quantities derived from
// sensor readings.
//
// This calculates:
// - Velocity estimates, by finite differencing then filtering the encoder
//   readings.
// - Shaft position, by applying a fixed offset to the encoder readings.
// - Electrical angle, using the encoder reading, a calibrated encoder offset
//   and the number of pole pairs.
// - Direct and Quadrature current estimates, by applying the Clarke and Park
//   transforms to the measured phase currents.
class DerivedSensorInformationUpdater {
 public:
  // Factory function.
  //
  // Returns an invalid argument error if the cycle time is <= 0, or encoder
  // counts per turn is equal to zero.
  static pw::Result<DerivedSensorInformationUpdater> Build(
      SignalFilterInterface& velocity_filter, uint32_t encoder_counts_per_turn,
      int32_t encoder_count_at_zero_shaft_angle, float cycle_time);

  // Resets the internal state of the updater, in particular the stored previous
  // position and filter state for the filtered finite-difference velocity
  // calculations.
  void Reset();

  // Calculates an updated set of information derived from sensor readings.
  //
  // Note that this will update the internal state of the velocity estimator, so
  // should only be called once per cycle.
  //
  // This will fail if the unit conversions of the position or velocity readings
  // fail, e.g. due to overflow from an overly large position difference.
  pw::Result<SensorDerivedQuantities> GetDerivedSensorInformation(
      const SensorReadings& sensor_readings);

 private:
  explicit DerivedSensorInformationUpdater(
      SignalFilterInterface& velocity_filter, uint32_t encoder_counts_per_turn,
      int32_t encoder_count_at_zero_shaft_angle, float cycle_time);

  SignalFilterInterface& velocity_filter_;

  uint32_t encoder_counts_per_turn_;
  int32_t encoder_count_at_zero_shaft_angle_;
  float cycle_time_;
  std::optional<int32_t> prev_encoder_position_;
};

// Returns the electrical angle of the shaft, in the stator frame [Radians].
//
// Args:
//   count: Current encoder counts reading.
//   counts_per_turn: Number of encoder counts in a full rotation of the motor.
//   count_at_electrical_zero: Encoder reading when the electrical angle of the
//    motor is zero.
//   num_pole_pairs: The number of motor pole pairs.
float EncoderCountToElectricalAngle(int32_t count, uint32_t counts_per_turn,
                                    int32_t count_at_electrical_zero,
                                    uint16_t num_pole_pairs);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DERIVED_SENSOR_INFORMATION_H_
