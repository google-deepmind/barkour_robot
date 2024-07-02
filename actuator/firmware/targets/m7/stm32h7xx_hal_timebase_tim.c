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

#include "stm32h7xx_hal.h" // NOLINT

static TIM_HandleTypeDef tim6_handle;

#define kMicrosecondsPerSecond 1000000U

// Configures STM32 HAL Timebase.
//
// This function will configure and initialize TIM12 to provide the timebase
// for the STM32 HAL. TIM12 is set to fire once per millisecond.
//
// This function will be called when the HAL is initialized (HAL_Init()), and
// also any time the clock is reconfigured (HAL_RCC_ClockConfig())
//
// Args:
// - tick_priority: The priority of the tick interrupt.
HAL_StatusTypeDef HAL_InitTick(uint32_t tick_priority) {
  // Configure the TIM6 IRQ priority.
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, tick_priority, 0);

  // Enable TIM6 global Interrupt.
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  // Enable TIM6 clock.
  __HAL_RCC_TIM6_CLK_ENABLE();

  // Retrieve clock configuration.
  RCC_ClkInitTypeDef rcc_config;
  uint32_t flash_latency;
  HAL_RCC_GetClockConfig(&rcc_config, &flash_latency);

  // Determine TIM6 clock frequency [Hz].
  uint32_t timer_clock_frequency;
  if (rcc_config.APB1CLKDivider == RCC_HCLK_DIV1) {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  } else {
    timer_clock_frequency = 2 * HAL_RCC_GetPCLK1Freq();
  }

  // Determine prescaler value, which when applied, will set the TIM6 counter
  // clock frequency to 1Mhz.
  uint32_t timer_prescaler_value =
      ((timer_clock_frequency / kMicrosecondsPerSecond) - 1);

  // Initialize and start TIM6.
  tim6_handle.Instance = TIM6;
  // 1000 ticks @ 1Mhz frequency yields 1ms period.
  tim6_handle.Init.Period = 1000 - 1;
  tim6_handle.Init.Prescaler = timer_prescaler_value;
  tim6_handle.Init.ClockDivision = 0;
  tim6_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&tim6_handle) == HAL_OK) {
    return HAL_TIM_Base_Start_IT(&tim6_handle);
  }

  return HAL_ERROR;
}

// Suspends the tick increment by disabling TIM6 update interrupt.
void HAL_SuspendTick() {
  __HAL_TIM_DISABLE_IT(&tim6_handle, TIM_IT_UPDATE);
}

// Resumes the tick increment by enabling the TIM6 update interrupt.
void HAL_ResumeTick() { __HAL_TIM_ENABLE_IT(&tim6_handle, TIM_IT_UPDATE); }

// Called by the STM32 HAL each time a timer period elapses.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer_handle) {
  if (timer_handle->Instance == TIM6) {
    HAL_IncTick();
  }
}

void TIM6_DAC_IRQHandler() { HAL_TIM_IRQHandler(&tim6_handle); }
