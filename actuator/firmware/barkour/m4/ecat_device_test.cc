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

#include "actuator/firmware/barkour/m4/ecat_device.h"

#include <cstdint>

#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {
namespace {

TEST(EcatDeviceTest, FlashProgramMemory) {
  uint8_t data[64];
  // No data to write
  ASSERT_EQ(flash_firmware_test_only(0, 0, data, 0), 0U);

  for (uint8_t i = 0; i < 64; ++i) {
    data[i] = i;
  }

  // Writes a few bytes and checks their value.
  ASSERT_EQ(flash_firmware_test_only(131072, 131072 + 64, data, 64), 0U);
  for (uint8_t i = 0; i < 64; ++i) {
    ASSERT_EQ(*reinterpret_cast<uint8_t*>(0x08100000 + 131072 + i), i);
  }

  // Erases sector 1 to leave flash in known state after test.
  ASSERT_EQ(HAL_FLASH_Unlock(), HAL_OK);
  uint32_t page_error = 0;
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Banks = FLASH_BANK_2;
  EraseInitStruct.Sector = 1;
  EraseInitStruct.NbSectors = 1;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_4;
  ASSERT_EQ(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error), HAL_OK);
  ASSERT_EQ(page_error, 0xFFFFFFFF);
  ASSERT_EQ(HAL_FLASH_Lock(), HAL_OK);

  // Can only write multiples of 32 bytes.
  ASSERT_NE(flash_firmware_test_only(0, 63, data, 63), 0U);

  // Writing past memory size should fail.
  ASSERT_NE(
      flash_firmware_test_only(1024 * 1024 + 1, 1024 * 1024 + 64, data, 64),
      0U);
}

}  // namespace
}  // namespace barkour
