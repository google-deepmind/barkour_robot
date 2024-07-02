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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BARKOUR_SYSTEM_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BARKOUR_SYSTEM_H_

// This modules provides target-agnostic interfaces to hardware-backed
// components used by the Barkour Motor Controller.

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_thread/thread.h"
#include "actuator/firmware/barkour/common/interfaces/timer_notifier_interface.h"
#include "actuator/firmware/targets/m4/barkour_system_native.h"

namespace barkour {

// The frequency at which controller timer elapses.
constexpr float kControllerTimerNotifierUpdateFrequencyHz = 1000.0f;

class System {
 public:
  // Singleton access to System.
  static System& Get();

  // Initializes hardware peripherals used by the Barkour application.
  //
  // This function must be called prior calling any of the peripheral accessor
  // functions.
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
  //  - Unavailable() if this component is not available on the target.
  //  - FailedPrecondition() if barkour::system::Init() has not successfully
  //  completed.
  pw::Result<TimerNotifierInterface*> GetControllerTimerNotifier();

  // Provides options for the controller thread.
  const pw::thread::Options& GetControllerThreadOptions();

 private:
  System();

  // The native context is defined by the backend implementation.
  system_backend::NativeSystemType native_system_;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BARKOUR_SYSTEM_H_
