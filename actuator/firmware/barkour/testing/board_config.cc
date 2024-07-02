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

// Provides fake board-level functions for the Barkour.

#include "actuator/firmware/barkour/common/board_config.h"

#include <string_view>

#include "pw_status/status.h"
#include "pw_sync/mutex.h"

namespace {

constexpr std::string_view kCpuName("TEST_CPU");

pw::sync::Mutex motor_power_enabled_mtx;
bool motor_power_enabled;

}  // namespace

std::string_view barkour::BoardConfig::CpuName() { return kCpuName; }

pw::Status barkour::BoardConfig::EnableMotorPower() {
  std::lock_guard lock(motor_power_enabled_mtx);
  motor_power_enabled = true;
  return pw_Status::PW_STATUS_OK;
}

pw::Status barkour::BoardConfig::DisableMotorPower() {
  std::lock_guard lock(motor_power_enabled_mtx);
  motor_power_enabled = false;
  return pw_Status::PW_STATUS_OK;
}

// Return `motor_power_enabled` for both STO1 and STO2 status.
bool barkour::BoardConfig::Sto1Status() {
  std::lock_guard lock(motor_power_enabled_mtx);
  return motor_power_enabled;
}

bool barkour::BoardConfig::Sto2Status() {
  std::lock_guard lock(motor_power_enabled_mtx);
  return motor_power_enabled;
}

bool barkour::BoardConfig::MotorPowerEnabled() {
  std::lock_guard lock(motor_power_enabled_mtx);
  return motor_power_enabled;
}

// LEDs and buttons don't exist in tests.
void barkour::BoardConfig::EnableBlinky2Led() {}

void barkour::BoardConfig::DisableBlinky2Led() {}

void barkour::BoardConfig::ToggleBlinky2Led() {}

bool barkour::BoardConfig::Blinky2LedEnabled() { return false; }

bool barkour::BoardConfig::DebugButtonEnabled() { return false; }

bool barkour::BoardConfig::Blinky1ButtonEnabled() { return false; }
