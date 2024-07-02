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

#include "actuator/firmware/targets/m4/barkour_system_native.h"

#include <cstdint>

#include "actuator/firmware/barkour/common/barkour_system.h"
#include "actuator/firmware/barkour/common/interfaces/gpio_debug_interface.h"
#include "pw_log/log.h"
#include "pw_status/try.h"

namespace barkour {

namespace system_backend {

namespace {

// Thread priorities.
constexpr uint8_t kControllerThreadPriority = tskIDLE_PRIORITY + 3;

}  // anonymous namespace.

M4System::M4System() : initialized_(false) {
  controller_thread_options_.set_name("ControllerThread")
      .set_static_context(controller_thread_context_)
      .set_priority(static_cast<UBaseType_t>(kControllerThreadPriority));
}

pw::Status M4System::Initialize() {
  tim7_notifier_ = &Stm32Tim7Notifier::Get();
  PW_TRY(
      tim7_notifier_->SetPeriod(tim7_notifier_->GetTickFrequency() /
      barkour::kControllerTimerNotifierUpdateFrequencyHz));

  // Setup timer 15 as 2 PWM DACs for debug analog outputs on TRACE 2/Trace 3
  // IOs and a digital GPIO output on GPIOE 4.
  pw::Status result = GpioDebugInterface::Get().InitHardware();

  if (!result.ok()) {
    // Not fatal if error - for debugging only.
    PW_LOG_ERROR("PWMDAC Failed to initialize hardware");
  }

  initialized_ = true;
  return pw::OkStatus();
}

pw::Result<TimerNotifierInterface*> M4System::GetControllerTimerNotifier() {
  if (!initialized_) {
    return pw::Status::FailedPrecondition();
  }

  return tim7_notifier_;
}

const pw::thread::Options& M4System::GetControllerThreadOptions() {
  return controller_thread_options_;
}

}  // namespace system_backend.

}  // namespace barkour.
