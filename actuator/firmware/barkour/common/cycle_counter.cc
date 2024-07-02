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

#include "actuator/firmware/barkour/common/cycle_counter.h"

#include "stm32h7xx_hal.h"  // NOLINT

namespace barkour {

CycleCounter::CycleCounter() {}

CycleCounter& CycleCounter::Get() {
  static CycleCounter cycle_counter;
  return cycle_counter;
};

void CycleCounter::Enable() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void CycleCounter::Disable() { DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; }

bool CycleCounter::IsEnabled() const {
  return DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t CycleCounter::GetCount() const { return DWT->CYCCNT; }

void CycleCounter::Reset() { DWT->CYCCNT = 0; }

void CycleCounter::BlockForMicroseconds(uint32_t us) const {
  uint32_t start = DWT->CYCCNT;
  uint32_t target = us * (HAL_RCC_GetHCLKFreq() / 1000000);
  while (DWT->CYCCNT - start < target) {
  }
}

}  // namespace barkour
