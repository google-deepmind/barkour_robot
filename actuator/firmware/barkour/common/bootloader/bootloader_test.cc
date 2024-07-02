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

#include <cstddef>
#include <cstdint>

#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/fake_memory.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "pw_status/status.h"

namespace {

TEST(BootloaderPreparesMemoryTest,  // NOLINT
     BootloaderFailsToUpdateFirmwareWhenNotReady) {
  barkour::Bootloader& bootloaderTest = barkour::Bootloader::Get();
  bootloaderTest.PrepareMemory();
  size_t payload_length_in_bytes = 32;
  uint32_t flash_address = 0;
  uint32_t data_address = 0;
  pw::Status status;
  status = bootloaderTest.UpdateFirmware(flash_address, data_address,
                                         payload_length_in_bytes);
  ASSERT_TRUE(status.IsFailedPrecondition());  // NOLINT
}

TEST(BootloaderPreparesMemoryTest,
     BootloaderFailsToUpdateFirmwareWhenEmptyPayload) {
  barkour::Bootloader& bootloaderTest = barkour::Bootloader::Get();
  bootloaderTest.PrepareMemory();
  size_t payload_length_in_bytes = 0;
  uint32_t flash_address = 0;
  uint32_t data_address = 0;
  pw::Status status;
  status = bootloaderTest.UpdateFirmware(flash_address, data_address,
                                         payload_length_in_bytes);
  ASSERT_TRUE(status.IsInvalidArgument());
}

TEST(BootloaderPreparesMemoryTest,
     BootloaderFailsToUpdateFirmwareWhenFlashErasureFailed) {
  barkour::Bootloader& bootloaderTest = barkour::Bootloader::Get();
  barkour::FakeMemory& fake_memory = barkour::FakeMemory::Get();
  bootloaderTest.PrepareMemory();
  fake_memory.InterruptCall(pw::Status::Internal());

  size_t payload_length_in_bytes = 32;
  uint32_t flash_address = 0;
  uint32_t data_address = 0;
  pw::Status status;
  status = bootloaderTest.UpdateFirmware(flash_address, data_address,
                                         payload_length_in_bytes);
  ASSERT_TRUE(status.IsFailedPrecondition());
  fake_memory.Cleanup();
}

}  // namespace
