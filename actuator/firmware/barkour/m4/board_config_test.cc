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

#include "actuator/firmware/barkour/common/board_config.h"

#include <cstdint>
#include <string_view>

#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "stm32h755xx.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {
namespace {

static void SetupGPIO() {
  GPIO_InitTypeDef gpio_init_struct;

  /// GPIO Ports Clock Enable.
  __HAL_RCC_GPIOG_CLK_ENABLE();

  // Setup FSOE STO1 pin.
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_2;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

  // Setup FSOE STO2 pin.
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_3;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

  // Setup FB STO 1 pin.
  gpio_init_struct.Pin = GPIO_PIN_4;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

  // Setup FB STO 2 pin.
  gpio_init_struct.Pin = GPIO_PIN_5;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);
}

TEST(BoardConfigTest, Blinky1ButtonIsDisabledByDefault) {
  BoardConfig& board = BoardConfig::Get();
  ASSERT_FALSE(board.Blinky1ButtonEnabled());
}

// Simply tests that nothing crashes when we try to toggle the LED.
TEST(BoardConfigTest, Blinky2LedsCanBeToggled) {
  BoardConfig& board = BoardConfig::Get();
  board.EnableBlinky2Led();
  // Give it some time to change state.
  for (uint16_t j = 0; j < 1000; ++j) {}
  ASSERT_TRUE(board.Blinky2LedEnabled());
  board.DisableBlinky2Led();
  for (uint16_t j = 0; j < 10000; ++j) {}
  ASSERT_FALSE(board.Blinky2LedEnabled());
  board.ToggleBlinky2Led();
  for (uint16_t j = 0; j < 10000; ++j) {}
  ASSERT_TRUE(board.Blinky2LedEnabled());
  // Turn the LED off when done.
  board.DisableBlinky2Led();
}

// Verifies that the debug button is not pressed.
TEST(BoardConfigTest, DebugButtonIsNotPressed) {
  BoardConfig& board = BoardConfig::Get();
  ASSERT_FALSE(board.DebugButtonEnabled());
}

// Verifies that the CpuName is valid..
TEST(BoardConfigTest, CpuNameIsvalid) {
  BoardConfig& board = BoardConfig::Get();
  ASSERT_NE(board.CpuName().find("M"), std::string_view::npos);
}

TEST(BoardConfigTest, MotorPowerIsDisabledByDefault) {
  BoardConfig& board = BoardConfig::Get();
  ASSERT_FALSE(board.MotorPowerEnabled());
}

TEST(BoardConfigTest, MotorPowerRemainsDisabledWhenDisablingPower) {
  BoardConfig& board = BoardConfig::Get();
  ASSERT_FALSE(board.MotorPowerEnabled());
  EXPECT_EQ(pw::OkStatus(), board.DisableMotorPower());
  ASSERT_FALSE(board.MotorPowerEnabled());
  for (uint16_t j = 0; j < 1000; ++j) {}
  ASSERT_FALSE(board.MotorPowerEnabled());
}

TEST(BoardConfigTest, CanDisableMotorPowerMultipleTimes) {
  BoardConfig& board = BoardConfig::Get();
  for (uint16_t i = 0; i < 100; i++) {
    ASSERT_FALSE(board.MotorPowerEnabled());
    EXPECT_EQ(pw::OkStatus(), board.DisableMotorPower());
  }
}

// This test assumes that the MCU has control over the 24Vsafe line.
// In other words, hardware interlocks must be in armed mode.
TEST(BoardConfigTest, CanEnableMotorPower) {
  // Using EXPECT in this test to make sure the test disabled the 24V line.

  SetupGPIO();

  BoardConfig& board = BoardConfig::Get();
  ASSERT_FALSE(board.MotorPowerEnabled());
  EXPECT_EQ(pw::OkStatus(), board.EnableMotorPower());
  for (uint16_t j = 0; j < 10000; ++j) {}
  EXPECT_TRUE(board.MotorPowerEnabled());
  // Turns motor power off after test.
  EXPECT_EQ(pw::OkStatus(), board.DisableMotorPower());
  for (uint16_t j = 0; j < 100000; ++j) {}
  ASSERT_FALSE(board.MotorPowerEnabled());
}

}  // namespace
}  // namespace barkour
