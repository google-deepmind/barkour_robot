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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_MEMORY_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_MEMORY_INTERFACE_H_

#include <cstddef>
#include <cstdint>

#include "pw_status/status.h"

namespace barkour {

// Reference to flash banks.
enum class FlashBank : uint8_t {
  kFlashBank1 = 0,
  kFlashBank2,
};

// Interface for interacting with memory.
class MemoryInterface {
 public:
  virtual ~MemoryInterface() = default;

  // Erases all sectors in the specified flash bank. Can be used with interrupt
  // by first assigning the callback using AssignFlashCallback.
  //
  // Args:
  // - flash_bank: reference to flash bank to be erased.
  // - interruptable: whether flash operation interrupt is enabled or not.
  //
  // Returns status of initiating flash erasure operation.
  virtual pw::Status EraseFlash(FlashBank flash_bank, bool interruptable) = 0;

  // Gives access to operations on flash.
  //
  // Args:
  // - lock_on: locks flash operations when true, unlocks when false.
  //
  // Returns status of locking/unlocking flash operations.
  virtual pw::Status FlashLock(bool lock_on) = 0;

  // Writes a 32-byte-long data from data_address to flash_address.
  //
  // Args:
  // - flash_address: address to write to.
  // - data_address: address to read from.
  //
  // Returns status of writing flash data.
  virtual pw::Status WriteFlash(uint32_t flash_address,
                                uint32_t data_address) = 0;

  // Validates data expected in flash by comparing it with data buffer.
  //
  // Args:
  // - flash_address: address of data already stored in flash.
  // - data_address: address of data buffer to compare flash with.
  // - length: buffer length.
  //
  // Returns status of validating flash data.
  virtual pw::Status ValidateFlash(uint32_t flash_address,
                                   uint32_t data_address, size_t length) = 0;

  // Assigns a callback to be called when flash operation triggers an interrupt.
  void AssignFlashCallback(void (*fct)(pw::Status)) {
    flash_callback_ = fct;
  }

  // Swaps flash banks.
  //
  // Returns status of swapping flash banks.
  virtual pw::Status SwapFlashBanks() = 0;

 protected:
  void (*flash_callback_)(pw::Status) = nullptr;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_MEMORY_INTERFACE_H_
