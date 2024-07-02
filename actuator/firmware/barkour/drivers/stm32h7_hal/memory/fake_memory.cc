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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/fake_memory.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/interfaces/memory_interface.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/memory.h"
#include "pw_status/status.h"

namespace barkour {

FakeMemory& FakeMemory::Get() {
  static FakeMemory fake_memory;
  return fake_memory;
}

void FakeMemory::Cleanup() {
  flash_bank_1_erased_ = false;
  flash_bank_1_erasable_ = false;
  flash_bank_2_erased_ = false;
  flash_bank_2_erasable_ = false;
  interruptable_ = false;
}

void FakeMemory::SetFlashBankState(FlashBank flash_bank, bool erased) {
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      flash_bank_1_erased_ = erased;
      break;
    }
    case FlashBank::kFlashBank2: {
      flash_bank_2_erased_ = erased;
      break;
    }
  }
}

void FakeMemory::SetFlashBankErasable(FlashBank flash_bank, bool eraseable) {
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      flash_bank_1_erasable_ = eraseable;
      break;
    }
    case FlashBank::kFlashBank2: {
      flash_bank_2_erasable_ = eraseable;
      break;
    }
  }
}

bool FakeMemory::GetFlashBankErasable(FlashBank flash_bank) {
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      return flash_bank_1_erasable_;
    }
    case FlashBank::kFlashBank2: {
      return flash_bank_2_erasable_;
    }
  }
}

bool FakeMemory::GetFlashBankState(FlashBank flash_bank) {
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      return flash_bank_1_erased_;
    }
    case FlashBank::kFlashBank2: {
      return flash_bank_2_erased_;
    }
  }
}

void FakeMemory::SetInterruptability(bool interruptable) {
  interruptable_ = interruptable;
}

bool FakeMemory::GetInterruptability() { return interruptable_; }

MemoryDriver& MemoryDriver::Get() {
  static MemoryDriver memory_driver;
  return memory_driver;
}

pw::Status MemoryDriver::EraseFlash(FlashBank flash_bank, bool interruptable) {
  if (!FakeMemory::Get().GetInterruptability() && interruptable) {
    return pw::Status::Internal();
  }

  if (FakeMemory::Get().GetFlashBankErasable(flash_bank)) {
    return pw::OkStatus();
  }

  return pw::Status::Internal();
}

pw::Status MemoryDriver::FlashLock(bool lock_on) {
  (void)lock_on;
  return pw::OkStatus();
}

pw::Status MemoryDriver::WriteFlash(uint32_t flash_address,
                                    uint32_t data_address) {
  (void)flash_address;
  (void)data_address;
  return pw::OkStatus();
}

pw::Status MemoryDriver::ValidateFlash(uint32_t flash_address,
                                       uint32_t data_address, size_t length) {
  (void)flash_address;
  (void)data_address;
  (void)length;
  return pw::OkStatus();
}

pw::Status MemoryDriver::SwapFlashBanks() { return pw::OkStatus(); }

MemoryDriver::MemoryDriver() = default;

void flash_irq_handler(pw::Status flash_op_status) {
  MemoryDriver::Get().flash_callback_(flash_op_status);
}

void FakeMemory::InterruptCall(pw::Status flash_op_status) {
  flash_irq_handler(flash_op_status);
}

}  // namespace barkour
