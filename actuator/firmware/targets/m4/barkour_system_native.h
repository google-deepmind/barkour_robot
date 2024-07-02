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

#ifndef BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_BARKOUR_SYSTEM_NATIVE_H_
#define BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_BARKOUR_SYSTEM_NATIVE_H_

#include <cstddef>

#include "actuator/firmware/targets/m4/stm32_tim7_notifier.h"
#include "pw_thread_freertos/options.h"
#include "pw_result/result.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

namespace system_backend {

// Thread stack sizes.
constexpr size_t kControllerTaskStackSizeBytes = 1024;

class M4System {
 public:
  M4System();

  pw::Status Initialize();

  // Provides a pointer to the ControllerTimerNotifier used for control loop
  // notification.
  //
  // In order to receiver controller timer notifications, the caller must do the
  // following:
  //   - Register a thread notification with the controller TimerNotifier.
  //   - Start the TimerNotifier.
  //
  // Returns:
  //  - Unavailable() if this compoment is not available on the target.
  //  - FailedPrecondition() if barkour::system::Init() has not successfully
  //  completed.
  pw::Result<TimerNotifierInterface*> GetControllerTimerNotifier();

  // Provides options for the controller thread.
  const pw::thread::Options& GetControllerThreadOptions();

 private:
  bool initialized_;

  // Peripheral-backed objects.
  Stm32Tim7Notifier* tim7_notifier_;

  // Thread contexts.
  pw::thread::freertos::StaticContextWithStack<kControllerTaskStackSizeBytes>
      controller_thread_context_;

  // Thread options.
  pw::thread::freertos::Options controller_thread_options_;
};

using NativeSystemType = M4System;

}  // namespace system_backend

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_BARKOUR_SYSTEM_NATIVE_H_
