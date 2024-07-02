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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_MEMORY_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_MEMORY_H_

#include <cstddef>
#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/memory_interface.h"
#include "pw_status/status.h"

namespace barkour {

// Implements memory access
class MemoryDriver : public MemoryInterface {
 public:
  static MemoryDriver& Get();
  pw::Status EraseFlash(FlashBank flash_bank, bool interruptable) override;
  pw::Status FlashLock(bool lock_on) override;
  pw::Status WriteFlash(uint32_t flash_address, uint32_t data_address) override;
  pw::Status ValidateFlash(uint32_t flash_address, uint32_t data_address,
                           size_t length) override;
  pw::Status SwapFlashBanks() override;

  MemoryDriver();
  ~MemoryDriver() override = default;

 private:
  // Called from an interrupt to provide a callback.
  friend void flash_irq_handler(pw::Status flash_op_status);
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MEMORY_MEMORY_H_
