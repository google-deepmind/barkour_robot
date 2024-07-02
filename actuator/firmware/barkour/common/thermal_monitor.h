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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_THERMAL_MONITOR_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_THERMAL_MONITOR_H_

#include <array>
#include <cstdint>

#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// Monitors the temperatures of the thermistors, and reports whether a phase has
// overheated or not.
//
// This maintains in internal counter for each thermistor. At each call to
// `Update`, if the temperature of each thermistor is above the specified
// temperature threshold then the associated counter will be incremented (unless
// it has hit the saturation value), and if not then the counter will be
// decremented (unless it is already zero). The motor is deemed to have
// overheated if any of the counters is at least the specified counter
// threshold.
template <uint8_t NumThermistors>
class ThermalMonitor {
 public:
  // Factory function. Validates the parameters, returning an invalid argument
  // error if either `counter_threshold` or `counter_saturation` is == 0, or if
  // `counter_saturation < counter_threshold`. Returns a configured
  // ThermalMonitor instance otherwise.
  static pw::Result<ThermalMonitor> Build(float temperature_threshold,
                                          uint16_t counter_threshold,
                                          uint16_t counter_saturation) {
    if (counter_threshold == 0 || counter_saturation == 0 ||
        counter_saturation < counter_threshold) {
      PW_LOG_ERROR(
          "Invalid counter parameters received for ThermalMonitor - threshold: "
          "%d, saturation: %d.",
          counter_threshold,
          counter_saturation);
      return pw::Status::InvalidArgument();
    }

    return ThermalMonitor(
        temperature_threshold, counter_threshold, counter_saturation);
  }

  // Updates the counters.
  void Update(std::array<float, NumThermistors> temperatures) {
    for (uint8_t i = 0; i < NumThermistors; ++i) {
      if (temperatures[i] >= temperature_threshold_) {
        counters_[i] = counters_[i] < counter_saturation_ ? counters_[i] + 1
                                                          : counters_[i];
      } else {
        counters_[i] = counters_[i] > 0 ? counters_[i] - 1 : 0;
      }
    }
  }

  // Reports whether the motor has overheated, or not.
  bool Overheated() const {
    for (uint16_t counter : counters_) {
      if (counter >= counter_threshold_)
        return true;
    }
    return false;
  }

 private:
  explicit ThermalMonitor(float temperature_threshold,
                          uint16_t counter_threshold,
                          uint16_t counter_saturation)
      : temperature_threshold_(temperature_threshold),
        counter_threshold_(counter_threshold),
        counter_saturation_(counter_saturation),
        counters_({}) {}

  float temperature_threshold_;
  uint16_t counter_threshold_;
  uint16_t counter_saturation_;
  std::array<uint16_t, NumThermistors> counters_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_THERMAL_MONITOR_H_
