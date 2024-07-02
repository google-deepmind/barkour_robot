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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_TIMER_NOTIFIER_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_TIMER_NOTIFIER_INTERFACE_H_

#include <cstdint>

#include "pw_status/status.h"
#include "pw_sync/thread_notification.h"

namespace barkour {

// Provides a timer-backed thread notification interface.
class TimerNotifierInterface {
 public:
  virtual ~TimerNotifierInterface() = default;

  // Registers a ThreadNotification which will be released whenever the timer
  // has elapsed.
  virtual pw::Status RegisterThreadNotification(
      pw::sync::ThreadNotification& notification) = 0;

  // Returns the timer tick frequency, in Hertz.
  virtual float GetTickFrequency() = 0;

  // Sets the timer period.
  //
  // Args:
  //  - ticks: The number of ticks.
  //
  // Returns:
  //  - InvalidArgument() if the tick value exceeds the number of counter bits.
  virtual pw::Status SetPeriod(uint32_t ticks) = 0;

  // Returns the timer period, in ticks.
  virtual uint32_t GetPeriod() = 0;

  // Starts the timer.
  virtual pw::Status Start() = 0;

  // Stops the timer.
  virtual pw::Status Stop() = 0;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_TIMER_NOTIFIER_INTERFACE_H_
