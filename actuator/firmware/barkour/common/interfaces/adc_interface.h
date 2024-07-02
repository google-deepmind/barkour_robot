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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ADC_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ADC_INTERFACE_H_

#include <cstddef>
#include <cstdint>
#include <limits>

#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_result/result.h"

namespace barkour {

enum class AdcAnalogSignal : uint8_t {
  kPhaseACurrent = 0,
  kPhaseBCurrent = 1,
  kPhaseCCurrent = 2,
  kMotorCurrent = 3,
  kPhaseAVoltage = 4,
  kPhaseBVoltage = 5,
  kPhaseCVoltage = 6,
  k24VSafeVoltage = 7,
  kBusVoltage = 8,
  kMotorTherm1 = 9,
  kMotorTherm2 = 10,
  kMotorTherm3 = 11,
  kEncoderTherm = 12,
  kDriveTherm = 13,
  kH7TempSensor = 14,
  kDrainSourceVoltageA = 15,
  kDrainSourceVoltageB = 16,
  kDrainSourceVoltageC = 17,
};
constexpr size_t kNumAdcAnalogSignals = 19;

// Provides access to ADC samples.
class AdcInterface {
 public:
  virtual ~AdcInterface() = default;

  // Retrieves the latest sample for the given signal.
  virtual pw::Result<int32_t> GetSample(const AdcAnalogSignal& signal) = 0;

  // Retrieves the latest injected mode sample for the given signal.
  virtual pw::Result<int32_t> GetInjectedSample(
      const AdcAnalogSignal& signal) = 0;

  // Setup which 2 channels to sample phase current with the injected sampling
  // function using:
  //  ADC1 and ADC2 for one channel each
  //
  // phases = kAB
  //   ADC 1 samples Phase A // ADC1 CH0
  //   ADC 2 samples Phase B // ADC2 CH1
  //
  // phases = kAC
  //   ADC 1 samples Phase C // ADC1 CH2
  //   ADC 2 samples Phase A // ADC2 CH0
  //
  // phases = kBC
  //   ADC 1 samples Phase C // ADC1 CH2
  //   ADC 2 samples Phase B // ADC2 CH1
  virtual void SetInjectedChannels(PhaseSampleSelection phases) = 0;

  // Return code indicating which phases are sampled by injected readings.
  virtual PhaseSampleSelection GetSelectedPhases() = 0;

  // Waits for injected conversion, tracking the count of injected
  // conversions which have occurred since the last call to this function.
  //
  // Args:
  // - block_duration: The number of ticks to block for, before returning.
  //
  // Returns the number times of injected samples has been updated since the
  //   last call.
  virtual uint32_t WaitForInjectedConversion(uint32_t block_duration) = 0;

  // Waits indefinitely for injected conversion, tracking the count of injected
  // conversions which have occurred since the last call to this function.
  //
  // Returns the number times of injected samples has been updated since the
  //   last call.
  uint32_t WaitForInjectedConversion() {
    return WaitForInjectedConversion(std::numeric_limits<uint32_t>::max());
  }
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ADC_INTERFACE_H_
