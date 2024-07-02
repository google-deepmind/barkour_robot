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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SENSOR_READING_TYPES_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SENSOR_READING_TYPES_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"

namespace barkour {

// Contains  sensor readings
// including unit conversions.
struct SensorReadings {
  uint32_t raw_encoder_count;      // Encoder Ticks (wraps at 1 revolution)
  int32_t encoder_position;        // Encoder Ticks
  float electrical_angle;          // Radians
  float quadrature_current;        // Amps
  float direct_current;            // Amps
  float phase_a_current;           // Amps
  float phase_b_current;           // Amps
  float phase_c_current;           // Amps
  float bus_voltage;               // Volts
  float thermistor_1;              // Degrees C
  float thermistor_2;              // Degrees C
  float thermistor_3;              // Degrees C
  float thermistor_4;              // Degrees C
  float sto_24v_safe_adc_voltage;  // Volts
  ImuState imu_state;              // imu acceleration and gyro data
};

// Contains useful quantities derived from the raw sensor readings, which
// require further calculations to compute.
struct SensorDerivedQuantities {
  int32_t shaft_position;  // Encoder ticks
  int32_t shaft_velocity;  // Encoder ticks / second
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SENSOR_READING_TYPES_H_
