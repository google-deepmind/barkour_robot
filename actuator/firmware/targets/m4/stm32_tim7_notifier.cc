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

// It would be great if this module could be more generic, perhaps having the
// name stm32_tim_notifier instead. Users would ideally be able to create
// instances of Stm32TimNotifier for a specific timer by passing in all
// necessary configuration parameters upon construction.
//
// Unfortunately, it may prove difficult to achieve this level of generality
// without significant effort, given that STM32 HAL is lacking a few features
// that would make this easy.
//
// Such features include, but may not be limited to:
//  - A means to determine the peripheral clock enable function given the
//    contents of a TIM_HandleTypeDef structure.
//  - A means to determine the external interrupt number for a timer peripheral
//    given the contents of TIM_HandleTypeDef structure.
//
// For now, this module will continue to be specific to TIM7.
#include "actuator/firmware/targets/m4/stm32_tim7_notifier.h"

#include "pw_assert/check.h"

constexpr float kTim7DesiredFrequencyInHertz = 1e6;

TIM_HandleTypeDef tim7_handle = {
    .Instance = TIM7,
    .Init = {.CounterMode = TIM_COUNTERMODE_UP,
             .Period = 250,  // 1e6 [counts/sec] / 250 [counts] = 4000 Hz.
             .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE}};

static void HalTim7MspInit(TIM_HandleTypeDef* handle) {
  __HAL_RCC_TIM7_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

static void HalTim7MspDeInit(TIM_HandleTypeDef* handle) {
  __HAL_RCC_TIM7_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(TIM7_IRQn);
}

static void HalTimPeriodElapsed(TIM_HandleTypeDef* handle) {
  barkour::Stm32Tim7Notifier::Get().ReleaseNotification();
}

namespace barkour {

Stm32Tim7Notifier::Stm32Tim7Notifier() : timer_handle_(tim7_handle) {
  PW_CHECK_INT_EQ(
      HAL_TIM_RegisterCallback(&timer_handle_, HAL_TIM_BASE_MSPINIT_CB_ID,
                               HalTim7MspInit),
      HAL_OK);

  PW_CHECK_INT_EQ(
      HAL_TIM_RegisterCallback(&timer_handle_, HAL_TIM_BASE_MSPDEINIT_CB_ID,
                               HalTim7MspDeInit),
      HAL_OK);

  // Prescaler value of 0 corresponds clock division by 1, thus the -1 term.
  tim7_handle.Init.Prescaler =
      std::round(SystemCoreClock / kTim7DesiredFrequencyInHertz) - 1;

  PW_CHECK_INT_EQ(HAL_TIM_Base_Init(&timer_handle_), HAL_OK);

  PW_CHECK_INT_EQ(
      HAL_TIM_RegisterCallback(
          &timer_handle_, HAL_TIM_PERIOD_ELAPSED_CB_ID, HalTimPeriodElapsed),
      HAL_OK);
}

Stm32Tim7Notifier& Stm32Tim7Notifier::Get() {
  static Stm32Tim7Notifier notifier;
  return notifier;
}

pw::Status Stm32Tim7Notifier::RegisterThreadNotification(
    pw::sync::ThreadNotification& notification) {
  notification_ = &notification;
  return pw::OkStatus();
}

float Stm32Tim7Notifier::GetTickFrequency() {
  return SystemCoreClock / (float)(tim7_handle.Init.Prescaler + 1);
}

pw::Status Stm32Tim7Notifier::SetPeriod(uint32_t ticks) {
  if (ticks > std::numeric_limits<uint16_t>::max()) {
    return pw::Status::InvalidArgument();
  }
  timer_handle_.Instance->ARR = ticks;

  return pw::OkStatus();
}

uint32_t Stm32Tim7Notifier::GetPeriod() { return timer_handle_.Instance->ARR; }

pw::Status Stm32Tim7Notifier::Start() {
  if (!notification_) {
    return pw::Status::Internal();
  }

  if (HAL_TIM_Base_Start_IT(&timer_handle_) != HAL_OK) {
    return pw::Status::Internal();
  }

  return pw::OkStatus();
}

pw::Status Stm32Tim7Notifier::Stop() {
  if (!notification_) {
    return pw::Status::Internal();
  }

  if (HAL_TIM_Base_Stop_IT(&timer_handle_) != HAL_OK) {
    return pw::Status::Internal();
  }

  return pw::OkStatus();
}

void Stm32Tim7Notifier::ReleaseNotification() {
  if (notification_ != nullptr) {
    notification_->release();
  }
}

}  // namespace barkour.

extern "C" {
void TIM7_IRQHandler(void) { HAL_TIM_IRQHandler(&tim7_handle); }
}  // extern C.
