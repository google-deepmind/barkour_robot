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

// Provides board-level functions for the barkour.

#include "actuator/firmware/barkour/common/board_config.h"

#include <string_view>

#include "FreeRTOS.h" // NOLINT
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT
#include "pw_status/status.h"

constexpr std::string_view kCpuName("M4");

std::string_view barkour::BoardConfig::CpuName() { return kCpuName; }

pw::Status barkour::BoardConfig::EnableMotorPower() {
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
  taskEXIT_CRITICAL();
  return pw_Status::PW_STATUS_OK;
}

pw::Status barkour::BoardConfig::DisableMotorPower() {
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
  return pw_Status::PW_STATUS_OK;
}

bool barkour::BoardConfig::Sto1Status() {
  return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4) == GPIO_PIN_RESET;
}

bool barkour::BoardConfig::Sto2Status() {
  return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) == GPIO_PIN_RESET;
}

bool barkour::BoardConfig::MotorPowerEnabled() {
  return Sto1Status() && Sto2Status();
}

void barkour::BoardConfig::Blinky2SetIOMode(bool output) {
  SetPinMode(GPIOH, GPIO_PIN_11, output);
}

void barkour::BoardConfig::EnableBlinky2Led() {
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
}

void barkour::BoardConfig::DisableBlinky2Led() {
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
}

void barkour::BoardConfig::ToggleBlinky2Led() {
  HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
}

bool barkour::BoardConfig::Blinky2LedEnabled() {
  return (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) == GPIO_PIN_RESET);
}

bool barkour::BoardConfig::Blinky2ButtonEnabled() {
  return (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) == GPIO_PIN_RESET);
}

bool barkour::BoardConfig::DebugButtonEnabled() {
  return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET);
}

void barkour::BoardConfig::Blinky1SetIOMode(bool output) {
  SetPinMode(GPIOD, GPIO_PIN_11, output);
}

void barkour::BoardConfig::EnableBlinky1Led() {
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
}

void barkour::BoardConfig::DisableBlinky1Led() {
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
}

void barkour::BoardConfig::ToggleBlinky1Led() {
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
}

bool barkour::BoardConfig::Blinky1LedEnabled() {
  return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_RESET);
}

bool barkour::BoardConfig::Blinky1ButtonEnabled() {
  return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_RESET);
}

void barkour::BoardConfig::SetPinMode(void* gpio_port, uint16_t pin_num,
                                 bool output) {
  GPIO_InitTypeDef gpio_init_struct;

  gpio_init_struct.Pin = pin_num;

  if (output) {
    // Setup pin to output.
    HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, pin_num, GPIO_PIN_RESET);

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  } else {
    // set to input
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  }

  HAL_GPIO_Init((GPIO_TypeDef*)gpio_port, &gpio_init_struct);
}
