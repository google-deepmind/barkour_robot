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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_READ_SENSORS_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_READ_SENSORS_H_

#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"

namespace barkour {

// Helper class to get the latest reading from all physical sensors.
//
// The encoder instance passed is required to give multi-turn encoder counts,
// otherwise an internal error will be returned from
// GetLatestReadingsFromAllSensors.
class SensorReader {
 public:
  // Constructor.
  //
  // Args:
  // - adc: ADC instance to use to get analog sensor readings.
  // - encoder: Encoder instance to use to get shaft position readings.
  // - max_consecutive_encoder_data_loss_errors: Maximum number of consecutive
  //   data loss errors from the encoder allowed before an error is returned
  //   from GetLatestReadingsFromAllSensors.
  //
  explicit SensorReader(AdcInterface& adc, RotaryEncoder& encoder,
                        FocInterface& foc, ImuInterface& imu,
                        uint8_t max_consecutive_encoder_data_loss_errors);

  // Returns the latest sensor readings, or an error if these are not available
  // for any reason.
  //
  // If a data loss error is returned from the encoder, the behaviour will
  // depend on whether a good encoder reading has been made since the class
  // instance was constructed.
  // - If a good reading hasn't been made yet, a data loss error will be
  //   returned.
  // - If a good reading has been made, either the last good reading or a data
  //   loss error will be returned, depending on the number of consecutive
  //   errors reported from the encoder.
  pw::Result<SensorReadings> GetLatestReadingsFromAllSensors();

 private:
  AdcInterface& adc_;
  RotaryEncoder& encoder_;
  FocInterface& foc_;
  ImuInterface& imu_;

  uint8_t max_consecutive_encoder_data_loss_errors_;

  uint8_t encoder_data_loss_error_counter_;

  // Contains a value if and only if a successful encoder reading has been made.
  std::optional<int32_t> last_good_encoder_reading_;
  std::optional<uint32_t> last_good_raw_encoder_reading_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_READ_SENSORS_H_
