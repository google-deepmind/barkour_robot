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

#include "actuator/firmware/barkour/common/barkour_system.h"

#include "actuator/firmware/targets/m4/stm32_tim7_notifier.h"
#include "pw_status/try.h"
#include "pw_thread_freertos/options.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

namespace {}  // anonymous namespace.

System::System() {}

// Singleton access to System.
System& System::Get() {
  static System system;
  return system;
}

pw::Status System::Initialize() { return native_system_.Initialize(); }

pw::Result<TimerNotifierInterface*> System::GetControllerTimerNotifier() {
  return native_system_.GetControllerTimerNotifier();
}

const pw::thread::Options& System::GetControllerThreadOptions() {
  return native_system_.GetControllerThreadOptions();
}

}  // namespace barkour.
