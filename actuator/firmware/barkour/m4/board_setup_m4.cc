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

#include "actuator/firmware/barkour/m4/board_setup_m4.h"

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gpio_debug.h"
#include "stm32h755xx.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT
#include "pw_log/log.h"

namespace barkour {
void SetupGPIO() {
  GPIO_InitTypeDef gpio_init_struct;

  /// GPIO Ports Clock Enable.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  // Blinky1 LED/Button.
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init_struct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOD, &gpio_init_struct);

  // Setup EN pin for DRV8353.
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_7;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

  // Setup CS pin for DRV8353.
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
  gpio_init_struct.Pin = GPIO_PIN_8;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

  // Setup nFault pin for DRV8353.
  gpio_init_struct.Pin = GPIO_PIN_6;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gpio_init_struct);

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

  // Blinky2 LED/Button.
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init_struct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOH, &gpio_init_struct);

  // Debug LED/Button.
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init_struct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpio_init_struct);

  // Drive PWM low during initialization.
  // Phase B low-side (Gate_IN_LB).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_8;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_init_struct);

  // Phase C high-side (Gate_IN_HC).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_9;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_init_struct);

  // Phase C low-side (Gate_IN_LC).
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_10;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_init_struct);

  // Phase A high-side (Gate_IN_HA).
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_6;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_init_struct);

  // Phase A, low-side (Gate_IN_LA).
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_7;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_init_struct);

  // Phase B, high-side (Gate_IN_HB).
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_8;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio_init_struct);

  // Test pin TP57.
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
  gpio_init_struct.Pin = GPIO_PIN_12;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_PULLDOWN;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &gpio_init_struct);
}

void Reboot() { HAL_NVIC_SystemReset(); }

}  // namespace barkour
