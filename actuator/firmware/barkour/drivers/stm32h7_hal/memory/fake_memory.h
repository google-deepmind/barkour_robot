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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_FAKE_MEMORY_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_FAKE_MEMORY_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/memory_interface.h"
#include "pw_status/status.h"

namespace barkour {

// FakeMemory implements a fake behaviour of the memory for unit testing.
class FakeMemory {
 public:
  static FakeMemory& Get();
  void InterruptCall(pw::Status flash_op_status);
  void Cleanup();
  void SetFlashBankState(FlashBank flash_bank, bool erased);
  bool GetFlashBankState(FlashBank flash_bank);

  void SetFlashBankErasable(FlashBank flash_bank, bool eraseable);
  bool GetFlashBankErasable(FlashBank flash_bank);

  void SetInterruptability(bool interruptable);
  bool GetInterruptability();

 private:
  FakeMemory() = default;
  bool flash_bank_1_erased_;
  bool flash_bank_1_erasable_;
  bool flash_bank_2_erased_;
  bool flash_bank_2_erasable_;
  bool interruptable_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_FAKE_MEMORY_H_
