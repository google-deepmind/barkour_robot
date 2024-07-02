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

/**
 ******************************************************************************
 * @file    gpio_debug.cc
 * @brief   This file provides code for the configuration
 *          of the GPIO DEBUG instances.
 ******************************************************************************
 *
 * Based on code generated from STM Cube IDE development Tool
 *
 ******************************************************************************
 */

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gpio_debug.h"

#include "pw_log/log.h"
#include "pw_status/status.h"
#include "stm32h7xx_hal.h" // NOLINT

void MX_TIM15_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

TIM_HandleTypeDef htim15;

/* TIM15 init function */
void MX_TIM15_Init(void) {
  if (!barkour::kEnableGpioDebug) {
    return;
  }
  TIM_ClockConfigTypeDef sClockSourceConfig = {0, 0, 0, 0};
  TIM_MasterConfigTypeDef sMasterConfig = {0, 0, 0};
  TIM_OC_InitTypeDef sConfigOC = {0, 0, 0, 0, 0, 0, 0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 base for GPIO debug");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 clock for GPIO debug");
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 for GPIO debug");
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) !=
      HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 master config for GPIO debug");
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 PWM channel 1 for GPIO debug");
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    PW_LOG_ERROR("Failed to init Timer15 PWM channel 1 for GPIO debug");
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) {
    PW_LOG_ERROR(
        "Failed to init Timer15 Break/DeadTime channel 1 for GPIO debug");
  }

  HAL_TIM_MspPostInit(&htim15);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM15) {
    /* USER CODE BEGIN TIM15_MspInit 0 */

    /* USER CODE END TIM15_MspInit 0 */
    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
    /* USER CODE BEGIN TIM15_MspInit 1 */

    /* USER CODE END TIM15_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0, 0, 0, 0, 0};
  if (timHandle->Instance == TIM15) {
    /* USER CODE BEGIN TIM15_MspPostInit 0 */

    /* USER CODE END TIM15_MspPostInit 0 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM15 GPIO Configuration
    PE5     ------> TIM15_CH1
    PE6     ------> TIM15_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM15_MspPostInit 1 */

    /* USER CODE END TIM15_MspPostInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {
  if (tim_baseHandle->Instance == TIM15) {
    /* USER CODE BEGIN TIM15_MspDeInit 0 */

    /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();
    /* USER CODE BEGIN TIM15_MspDeInit 1 */

    /* USER CODE END TIM15_MspDeInit 1 */
  }
}

namespace barkour {

// set an pwm dac output channel (1 or 2)  duty cycle
// value should be from -1.0 to +1.0
pw::Status GpioDebug::SetPWMDACOutput(const PWMDACChannel channel,
                                      float value) {
  if (!barkour::kEnableGpioDebug) {
    return pw::Status();
  }

  if (!(channel == PWMDACChannel::kPwmDacChannel1 ||
        channel == PWMDACChannel::kPwmDacChannel2)) {
    PW_LOG_ERROR("PWM_DAC Invalid Channel");
    return pw::Status::InvalidArgument();
  }

  if (value > 1.0f) {
    PW_LOG_WARN(
        "PWM_DAC[%lu] Value out of range (%5.2f >1.0 - Will be set to 1.0)",
        (uint32_t)channel,
        value);
    value = 1.0f;
  }
  if (value < -1.0f) {
    PW_LOG_WARN(
        "PWM_DAC[%lu] Value out of range (%5.2f < -1.0 - Will be set to -1.0)",
        (uint32_t)channel,
        value);
    value = -1.0f;
  }

  __HAL_TIM_SET_COMPARE(
      (TIM_HandleTypeDef*)htim,
      channel == PWMDACChannel::kPwmDacChannel1 ? TIM_CHANNEL_1 : TIM_CHANNEL_2,
      (uint16_t)((500.0 * value) + 500));

  return pw::Status();
}

// Set a GPIO output channel hi or low.
// state=true - output set high, false=low.
pw::Status GpioDebug::SetDigitalOutput(const GPIOChannel channel, bool state) {
  if (!barkour::kEnableGpioDebug) {
    return pw::Status();
  }

  if (channel == GPIOChannel::kGpioChannel1) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  } else {
    return pw::Status::InvalidArgument();
  }

  return pw::Status();
}

// Init the timer hardware peripheral, sets it to PWM output and starts the PWM.
pw::Status GpioDebug::InitHardware(void) {
#ifdef DEBUG
  PW_LOG_WARN("DEBUG BUILD!!!");
#endif

  if (!barkour::kEnableGpioDebug) {
    return pw::Status();
  }

  PW_LOG_WARN(
      "**** Debug GPIO and PWM DAC are enabled. DO NOT CONNECT TRACExxx Pins "
      "to Mataric PCB");
  __HAL_RCC_TIM15_CLK_ENABLE();

  // Config Timer15 CH1 and CH2 as PWM Outputs (TRACE2 and TRACE3).
  MX_TIM15_Init();

  if (HAL_OK != HAL_TIM_PWM_Start((TIM_HandleTypeDef*)htim, TIM_CHANNEL_1)) {
    return pw::Status::Internal();
  }
  if (HAL_OK != HAL_TIM_PWM_Start((TIM_HandleTypeDef*)htim, TIM_CHANNEL_2)) {
    return pw::Status::Internal();
  }

  GPIO_InitTypeDef gpio_init_struct;

  // Test pin (TRACE1).
  // Set the debugger TRACE1 pin (PE4) as a GPIO for uses as a debug output.
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
  gpio_init_struct.Alternate = 0;
  gpio_init_struct.Pin = GPIO_PIN_4;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &gpio_init_struct);

  return pw::Status();
}

GpioDebug& GpioDebug::Get() {
  static GpioDebug gpio_debug;
  gpio_debug.htim = &htim15;
  return gpio_debug;
}

GpioDebugInterface& GpioDebugInterface::Get() { return GpioDebug::Get(); }

}  // namespace barkour
