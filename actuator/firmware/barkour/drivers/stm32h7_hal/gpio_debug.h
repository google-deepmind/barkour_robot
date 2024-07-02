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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GPIO_DEBUG_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GPIO_DEBUG_H_

/**
 ******************************************************************************
 * @file    gpio_debug.h
 * @brief   This file contains all the function prototypes for
 *          the gpio_debug.cc file
 *
 *   This class creates access to two PWM outputs on the TRACE2/TRACE3 debug
 *   IO pins.
 *   These can be used to create debug PWM DAC analog signals that can be
 *   monitored on an oscilloscope.
 *
 *   TRACE1 is used as a GPIO digital output
 *
 *   To generate the Analog signals from the PWM outputs an extern RC Filter is
 *   required on the output pin of the STM, a 10K ohm resistor and 0.1uF cap is
 *   suggested.
 *
 *   TIM15 channel 1 and 2 are used as the PWM signal sources
 *   GPIOE pin 4 is used for the GPIO output
 *
 ******************************************************************************
 *
 * Based on code generated from STM Cube IDE development Tool
 *
 ******************************************************************************
 */

#include <cstddef>
#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/gpio_debug_interface.h"
#include "pw_status/status.h"

namespace barkour {

//  This flag must be set true to enable PWM DAC and GPIO output.
//
//  NOTE: Only enable this if TRACExx pins are not grounded
//  via debug cable to Mataric PCB.
//
inline constexpr bool kEnableGpioDebug = false;

class GpioDebug : public GpioDebugInterface {
 public:
  // Get singleton instance.
  static GpioDebug& Get();

  // Set a channel output.
  pw::Status SetPWMDACOutput(const PWMDACChannel channel, float value) override;
  pw::Status SetDigitalOutput(const GPIOChannel channel, bool state) override;

  // Must be called at startup to initialise the TIM15 hardware peripheral.
  pw::Status InitHardware(void) override;

  // Return true if gpio debugging is enabled.
  bool IsGpioDebugEnabled() override { return kEnableGpioDebug; };

 private:
  void* htim;  // Handle to timer to use.
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GPIO_DEBUG_H_
