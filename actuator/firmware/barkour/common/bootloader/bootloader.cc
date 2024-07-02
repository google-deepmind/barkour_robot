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

#include "actuator/firmware/barkour/common/bootloader/bootloader.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/interfaces/memory_interface.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/memory.h"
#include "pw_log/log.h"
#include "pw_status/status.h"

namespace barkour {

void MemoryCallback(pw::Status flash_op_status) {
    if (flash_op_status.ok()) {
      Bootloader::Get().bootloader_state_ = BootloaderState::kReady;
      PW_LOG_INFO("Memory erasure success.");
    } else {
      Bootloader::Get().bootloader_state_ = BootloaderState::kError;
      PW_LOG_INFO("Memory erasure failure.");
    }
}

Bootloader& Bootloader::Get() {
  static Bootloader bootloader_;
  return bootloader_;
}

Bootloader::Bootloader()
    : memory_driver_(MemoryDriver::Get()) {
  memory_driver_.AssignFlashCallback(&MemoryCallback);
  bootloader_state_ = BootloaderState::kInitialised;
}

void Bootloader::PrepareMemory() {
  // Wait until memory is erased (triggered by interrupt)
  bootloader_state_ = BootloaderState::kPreparingMemory;
  memory_driver_.EraseFlash(FlashBank::kFlashBank2, true);
}

pw::Status Bootloader::UpdateFirmware(uint32_t flash_address,
                                      uint32_t data_address, size_t length) {
  // Sanity check on incoming data.
  if (length == 0) {
    PW_LOG_ERROR("Empty payload.");
    return pw::Status::InvalidArgument();
  } else if (bootloader_state_ != BootloaderState::kReady) {
    PW_LOG_ERROR("Bootloader is not ready.");
    return pw::Status::FailedPrecondition();
  }

  // Unlock FLASH access to start writing firmware.
  if (!memory_driver_.FlashLock(false).ok()) {
    PW_LOG_ERROR("Flash lock failed.");
    return pw::Status::Internal();
  }

  // Write FW data chunk to respective FLASH memory.
  if (!memory_driver_.WriteFlash(flash_address, data_address).ok()) {
    PW_LOG_ERROR("Flash write failed.");
    return pw::Status::Internal();
  }

  // Writing to FLASH finished so lock access.
  if (!memory_driver_.FlashLock(true).ok()) {
    PW_LOG_ERROR("Flash lock failed.");
    return pw::Status::Internal();
  }

  // Validate flashed firmware data.
  if (!memory_driver_.ValidateFlash(flash_address, data_address, length).ok()) {
    PW_LOG_ERROR("Flash validate failed.");
    return pw::Status::Internal();
  }

  return pw::OkStatus();
}

pw::Status Bootloader::BootNewFirmware() {
  // Basic sanity check to validate that the other bank is not empty.
  uint32_t* m4_address_to_check =
      reinterpret_cast<uint32_t*>(kFwUpdateCortexM4Addr);
  uint32_t* m7_address_to_check =
      reinterpret_cast<uint32_t*>(kFwUpdateCortexM7Addr);
  if (*m4_address_to_check == 0x00000000 ||
      *m4_address_to_check == 0xFFFFFFFF ||
      *m7_address_to_check == 0x00000000 ||
      *m7_address_to_check == 0xFFFFFFFF) {
    PW_LOG_ERROR("Sanity check on new firmware failed.");
    return pw::Status::FailedPrecondition();
  }

  // Swap flash banks.
  if (!memory_driver_.FlashLock(false).ok()) {
    PW_LOG_ERROR("Flash lock failed.");
    return pw::Status::Internal();
  }
  if (!memory_driver_.SwapFlashBanks().ok()) {
    PW_LOG_ERROR("Flash bank swap failed.");
    return pw::Status::Internal();
  }
  if (!memory_driver_.FlashLock(true).ok()) {
    PW_LOG_ERROR("Flash lock failed.");
    return pw::Status::Internal();
  }
  PW_LOG_INFO("Flash banks will swap after next reset.");
  return pw::OkStatus();
}

}  // namespace barkour
