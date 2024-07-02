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

#ifndef BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_STM32_TIM7_NOTIFIER_H_
#define BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_STM32_TIM7_NOTIFIER_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/timer_notifier_interface.h"
#include "stm32h7xx_hal.h" // NOLINT
#include "pw_sync/thread_notification.h"

namespace barkour {

// A TimerNotifierInterface based upon the STM32 TIM7 basic timer peripheral.
class Stm32Tim7Notifier : public TimerNotifierInterface {
 public:
  static Stm32Tim7Notifier& Get();

  // Registers a ThreadNotification which will be released whenever the timer
  // has elapsed.
  pw::Status RegisterThreadNotification(
      pw::sync::ThreadNotification& notification) override;

  // Returns the timer tick frequency, in Hertz.
  float GetTickFrequency() override;

  // Sets the timer period.
  //
  // Notes:
  //  - TIM7 is a 16-bit timer, so the period should not exceed 65535 ticks.
  //  - Preload is enabled, meaning that the tick value will be prior to the
  //     next timer cycle.
  //
  // Args:
  //  - ticks: The number of ticks.
  //
  // Returns:
  //  InvalidArgument(): if the tick value exceeds the number of counter bits.
  pw::Status SetPeriod(uint32_t ticks) override;

  // Returns the timer period, in ticks.
  uint32_t GetPeriod() override;

  pw::Status Start() override;

  pw::Status Stop() override;

  // This function should only be called internally, by the timer interrupt
  // service routine.
  void ReleaseNotification();

 private:
  pw::sync::ThreadNotification* notification_;
  TIM_HandleTypeDef& timer_handle_;

  // Constructs and instance of Stm32TimNotifier.
  Stm32Tim7Notifier();
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_STM32_TIM7_NOTIFIER_H_
