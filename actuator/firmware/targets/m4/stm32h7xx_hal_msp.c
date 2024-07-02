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

#include "stm32h7xx_hal.h"

// User-supplied initialization function called by STM32 HAL.
//
// System-level initialization should be performed in this function.
void HAL_MspInit() {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_VREF_CLK_ENABLE();
  HAL_SYSCFG_DisableVREFBUF();
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);
}
