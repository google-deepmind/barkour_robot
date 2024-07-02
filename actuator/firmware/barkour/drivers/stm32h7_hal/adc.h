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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_ADC_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_ADC_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "FreeRTOS.h" // NOLINT
#include "semphr.h" // NOLINT
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

// Provides access to ADC samples.
// Under the hood, ADC1, ADC2, and ADC3 are each configured in DMA mode.
// All values are continuously sampled and made available via the GetSample
// method.
// In addition to continuously sampling signals in DMA mode, ADC1/ADC2 are also
// configured to sample motor currents synchronously with PWM. These sampled
// signals are stored in the motor_adc_samples_injected member variable.
// If tight synchronization with injected mode sampling is desired, users
// can call RegisterForInjectedNotification followed by
// WaitForInjectedConversion.
class Adc : public AdcInterface {
 public:
  // Singleton access to ADC.
  static Adc& Get();

  // Retrieves the latest sample for the given signal.
  // Note: GetSample() will return pw::Status::Unknown() if GetSample has been
  // called before the first conversion has been completed.
  pw::Result<int32_t> GetSample(const AdcAnalogSignal& signal) override;

  // Retrieves the latest injected mode sample for the given signal.
  // Presently only kPhaseACurrent, kPhaseBCurrent, and kPhaseCCurrent are
  // sampled in the injected mode.
  //
  // Returns pw::Status::Unavailable() if an invalid signal has been provided.
  pw::Result<int32_t> GetInjectedSample(const AdcAnalogSignal& signal) override;

  // Waits for injected conversion, tracking the count of injected
  // conversions which have occurred since the last call to this function.
  //
  // Each time the an injected mode conversion is completed, the internal
  // conversion counter (a semaphore) will be incremented by one.
  //
  // When this function is called:
  //   - If the counter is nonzero
  //     - The counter will be cleared.
  //   - If the counter is zero:
  //     - This function will block until the counter has incremented.
  //     - The counter will be cleared.
  //
  // Args:
  // - block_duration: The number of ticks to block for, before returning.
  //
  // Returns the number times of injected samples has been updated since the
  //   last call.
  uint32_t WaitForInjectedConversion(
      TickType_t block_duration = portMAX_DELAY) override;

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
  void SetInjectedChannels(PhaseSampleSelection phases) override;

  PhaseSampleSelection GetSelectedPhases() override { return phases_; }

  // Stores the most recent injected mode samples for kPhaseACurrent,
  // kPhaseBCurrent, and kPhaseCCurrent, respectively. If tight synchronization
  // with injected mode sample updates is desired, use
  // RegisterForInjectedNotification/WaitForInjectedConversion. Do not access
  // this directly, it is public by necessity, use the GetInjectedSample()
  // instead.
  std::array<int32_t, 3> motor_adc_samples_injected;

  // Do not touch, public by necessity.
  SemaphoreHandle_t GetInjectedSampleSemaphore() const;

  // Start doing offset calibrations
  // Note: Motor PWM outputs must be set to 0V (0.5 duty) during calibration
  void StartOffsetCalibration();

  // Returns true if offset calibration has been completed.
  inline bool IsCalibrated() const {
    return calibration_state_ == CalibrationState::kAdcOcStateDone;
  }

  // Returns true if offset calibration is active.
  inline bool IsCalibrating() const {
    return calibration_state_ != CalibrationState::kAdcOcStatePending &&
           calibration_state_ != CalibrationState::kAdcOcStateDone;
  }

  // Return the adc1 and adc2 offset based on currently active phases
  void GetCalOffsets(int32_t& offset1, int32_t& offset2);

  // Run the calibration state machine.
  void DoCalibration(uint32_t adc1, uint32_t adc2);

 private:
  StaticSemaphore_t injected_sample_semaphore_buffer_;
  SemaphoreHandle_t injected_sample_semaphore_;

  PhaseSampleSelection phases_;

  // Offsets for injected channel measurements.
  int32_t offset_adc1_0_;
  int32_t offset_adc1_1_;
  int32_t offset_adc2_0_;
  int32_t offset_adc2_1_;

  uint32_t calibration_sample_count_;

  enum class CalibrationState {
    kAdcOcStateInit = 0,
    kAdcOcStatePending,
    kAdcOcStateAb,
    kAdcOcStateAc,
    kAdcOcStateBc,
    kAdcOcStateDone
  };

  CalibrationState calibration_state_;

  Adc();

  // Configures the ADCs and starts sampling.
  void SetupHardware();

  // Contains the memory location of the signal of interest.
  std::array<uint16_t*, kNumAdcAnalogSignals> signal_to_buffer_address_{
      nullptr};
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_ADC_H_
