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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GPIO_DEBUG_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GPIO_DEBUG_INTERFACE_H_

// This interface and the concrete instance provide a means of using a timer PWM
// outputs to generate debugging analog signals as well as control GPIO pins for
// debugging functions.
//
// To generate the Analog signals an extern RC Filter is required on the output
// pin of the STM, a 10K Ohm resistor and 0.1uF cap is suggested.

#include <cstdint>

#include "pw_status/status.h"

namespace barkour {

// Helper macros
#define DBG_DAC1(value)                          \
  barkour::GpioDebugInterface::Get().SetPWMDACOutput( \
      barkour::PWMDACChannel::kPwmDacChannel1, value)
#define DBG_DAC2(value)                          \
  barkour::GpioDebugInterface::Get().SetPWMDACOutput( \
      barkour::PWMDACChannel::kPwmDacChannel2, value)
#define DBG_GPIO(state)                           \
  barkour::GpioDebugInterface::Get().SetDigitalOutput( \
      barkour::GPIOChannel::kGpioChannel1, state)

// Define available PWM DAC Channels.
enum class PWMDACChannel : uint8_t {
  kPwmDacChannel1 = 0,
  kPwmDacChannel2 = 1,
};

// Define available GPIO Channels.
enum class GPIOChannel : uint8_t {
  kGpioChannel1 = 0,
};

// Number of supported DAC and GPIOs.
inline constexpr uint8_t kNumPWMDACChannels = 2;
inline constexpr uint8_t kNumGPIOChannels = 1;

// Provides access to PWM DAC and GPIO debugging outputs.
class GpioDebugInterface {
 public:
  // Returns singleton instance of concrete GPIO Debug object.
  static GpioDebugInterface& Get();  // singleton getter.

  virtual ~GpioDebugInterface() = default;

  // Set PWM DAC Channel output to value = -1 to 1.
  // Values out side limits will be clamped to +-1.
  virtual pw::Status SetPWMDACOutput(PWMDACChannel channel, float value) = 0;

  // Set GPIO Channel output to value - state =true - pin set high, false pin
  // set low.
  virtual pw::Status SetDigitalOutput(GPIOChannel channel, bool state) = 0;

  // Init timer/GPIO hardware IOs.
  virtual pw::Status InitHardware() = 0;

  // Return true if GPIO debugging is enabled.
  virtual bool IsGpioDebugEnabled() = 0;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GPIO_DEBUG_INTERFACE_H_
