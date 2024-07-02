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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOOTLOADER_BOOTLOADER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOOTLOADER_BOOTLOADER_H_

#include <stdint.h>

#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/memory.h"
#include "pw_status/status.h"
#include "pw_string/string.h"

namespace barkour {

// Defines bootloader's internal state.
enum class BootloaderState : uint8_t {
  kUninitialised = 0,
  kInitialised,
  kPreparingMemory,
  kReady,
  kError,
};

// Bootloader class to update new firmware.
class Bootloader {
 public:
  static Bootloader& Get();

  // Prepares memory before receiving new firmware data. Changes bootloader's
  // state to kPreparing_Memory and waits for an interrupt from Flash.
  void PrepareMemory();

  // Updates firmware with new firmware data and runs memory validation check
  // after writing to memory.
  //
  // Args:
  // - flash_address: address to write data to.
  // - data_address: address with data to write from.
  // - length: length of data to write in bytes.
  //
  // Returns Status::Ok() when memory validation succeeds after data is written.
  pw::Status UpdateFirmware(uint32_t flash_address, uint32_t data_address,
                            size_t length);

  // Runs a basic sanity check and boots up new firmware by swapping flash banks
  // and restarting the device.
  //
  // Returns Status::Ok() on success, but the device should restart before
  // returning.
  pw::Status BootNewFirmware();

  static constexpr uint32_t kFwUpdateCortexM4Addr = 0x08180000;
  static constexpr uint32_t kFwUpdateCortexM7Addr = 0x08100000;
  static constexpr uint32_t kFlashSectorSizeInBytes = 128 * 1024;
  static constexpr uint32_t kFlashBankSizeInBytes = 1024 * 1024;
  static constexpr uint8_t kFwNameLengthMax = 10;
  static constexpr pw::InlineString<kFwNameLengthMax> kFwNameM4 = "m4.bin";
  static constexpr pw::InlineString<kFwNameLengthMax> kFwNameM7 = "m7.bin";

 private:
  Bootloader();
  ~Bootloader() = default;
  MemoryInterface& memory_driver_;
  friend void MemoryCallback(pw::Status flash_op_status);

 protected:
  BootloaderState bootloader_state_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_BOOTLOADER_BOOTLOADER_H_
