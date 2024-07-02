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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ADC_CONVERSIONS_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ADC_CONVERSIONS_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "pw_result/result.h"

namespace barkour {

// Converts an ADC signal reading to the associated floating point value.
//
// All values will be returned in SI units.
// e.g.:
//   kPhaseACurrent will be returned in Amperes.
//   kPhaseAVoltage will be returned in Volts.
//   kMotorTherm1 will be returned in degrees Celsius.
//
// Returns: The value, in SI units, pw::Status::Unimplemented() otherwise.
pw::Result<float> AdcReadingToFloat(AdcAnalogSignal signal, int32_t reading);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ADC_CONVERSIONS_H_
