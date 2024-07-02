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

#include "actuator/firmware/barkour/m4/gpio.h"

#include <cstdint>

#include "actuator/firmware/barkour/m4/ecat_device.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

GPIO::GPIO() {}

GPIO& GPIO::Get() {
  static GPIO instance;
  return instance;
}

void GPIO::EnableInterrupts() {
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,
                       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

}  // namespace barkour

#ifdef __cplusplus
extern "C" {
#endif

void EXTI15_10_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    case GPIO_PIN_11: {
      return barkour::EthercatDevice::Get().EofInterrupt();
    }
    case GPIO_PIN_14: {
      return barkour::EthercatDevice::Get().Sync0Interrupt();
    }
    case GPIO_PIN_15: {
      return barkour::EthercatDevice::Get().Sync1Interrupt();
    }
    default: {
      return;
    }
  }
}

#ifdef __cplusplus
}
#endif
