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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOARD_CONFIG_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOARD_CONFIG_H_

#include <cstdint>
#include <string_view>

#include "pw_status/status.h"

namespace barkour {

// Provides easy access to hardware specific information.
class BoardConfig {
 public:
  // Singleton access to BoardConfig.
  static BoardConfig& Get() {
    static BoardConfig board_config;
    return board_config;
  }

  // Returns the name of the chip the code is running on (e.g. M4 or M7).
  std::string_view CpuName();

  // Tries to enable power to the gate driver (24Vsafe) by controlling the
  // STO lines. Note: This is not a guarantee the 24Vsafe line will be enabled
  // as there are multiple HW and SW interlocks (STO1, STO2, FSOE_STO1/2).
  // Returns:
  // - PW_STATUS_OK. Note: reserved for future use (e.g. we might check if
  // the voltage on the 24V line is stable).
  pw::Status EnableMotorPower();

  // Disables power to the gate driver (24Vsafe) and this the motor by
  // controlling the STO lines.
  // Returns:
  // - PW_STATUS_OK. Note: reserved for future use.
  pw::Status DisableMotorPower();

  // These methods return the status of the of the FB_STO1/2 lines, which
  // control the 24Vsafe line.
  bool Sto1Status();
  bool Sto2Status();

  // Returns Sto1Status() && Sto2Status();
  bool MotorPowerEnabled();

  // Sets the mode of the Mataric Blinky2 button, false=input(button),
  // true=output(led)
  void Blinky2SetIOMode(bool output);

  // Turns on the Blinky2 LED.
  void EnableBlinky2Led();
  // Turns off the Blinky2 LED.
  void DisableBlinky2Led();
  // Toggles the Blinky2 LED.
  void ToggleBlinky2Led();
  // Reads the current state of the Blinky2 LED.
  bool Blinky2LedEnabled();

  // Reads the state of the Mataric Blinky2 button
  bool Blinky2ButtonEnabled();

  // Reads the state of the Mataric Debug button.
  bool DebugButtonEnabled();

  // Sets the mode of the Mataric Blinky1 button, false=input(button),
  // true=output(led)
  void Blinky1SetIOMode(bool output);

  // Turns on the Blinky1 LED.
  void EnableBlinky1Led();

  // Turns off the Blinky1 LED.
  void DisableBlinky1Led();

  // Toggles the Blinky1 LED.
  void ToggleBlinky1Led();

  // Reads the current state of the Blinky1 LED.
  bool Blinky1LedEnabled();

  // Reads the state of the Mataric Blinky1 button, (if set to input mode).
  bool Blinky1ButtonEnabled();

 private:
  BoardConfig() = default;
  ~BoardConfig() = default;
  void SetPinMode(void* gpio_port, uint16_t pin_num, bool output);
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOARD_CONFIG_H_
