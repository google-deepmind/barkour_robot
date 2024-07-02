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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/memory/memory.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/interfaces/memory_interface.h"
#include "FreeRTOS.h" // NOLINT
#include "pw_log/config.h"
#include "pw_log/log.h"
#include "pw_log/options.h"
#include "pw_status/status.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

MemoryDriver& MemoryDriver::Get() {
  static MemoryDriver memory_driver;
  return memory_driver;
}

pw::Status MemoryDriver::EraseFlash(FlashBank flash_bank, bool interruptable) {
  // Turn Flash interrupt on if interrupt-driven.
  if (interruptable) {
    // Turn interrupt on.
    HAL_NVIC_EnableIRQ(FLASH_IRQn);
  }

  // Unlock flash bank.
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      if (HAL_FLASHEx_Unlock_Bank1() != HAL_OK) {
        PW_LOG_ERROR("Could not unlock flash memory.");
        HAL_NVIC_DisableIRQ(FLASH_IRQn);
        return pw::Status::Internal();
      }
      break;
    }
    case FlashBank::kFlashBank2: {
      if (HAL_FLASHEx_Unlock_Bank2() != HAL_OK) {
        PW_LOG_ERROR("Could not unlock flash memory.");
        HAL_NVIC_DisableIRQ(FLASH_IRQn);
        return pw::Status::Internal();
      }
      break;
    }
    default: {
      PW_LOG_ERROR("Undefined flash bank.");
      HAL_NVIC_DisableIRQ(FLASH_IRQn);
      return pw::Status::Internal();
    }
  }

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
  // Choose the correct FLASH bank.
  EraseInitStruct.Banks =
      flash_bank == FlashBank::kFlashBank1 ? FLASH_BANK_1 : FLASH_BANK_2;
  EraseInitStruct.Sector = 0;
  EraseInitStruct.NbSectors = 8;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_4;
  if (HAL_FLASHEx_Erase_IT(&EraseInitStruct) != HAL_OK) {
    PW_LOG_ERROR("Flash memory erasure failure.");
    HAL_NVIC_DisableIRQ(FLASH_IRQn);
    return pw::Status::Internal();
  }

  // Lock flash bank.
  switch (flash_bank) {
    case FlashBank::kFlashBank1: {
      if (HAL_FLASHEx_Lock_Bank1() != HAL_OK) {
        PW_LOG_ERROR("Could not lock flash memory.");
        HAL_NVIC_DisableIRQ(FLASH_IRQn);
        return pw::Status::Internal();
      }
      break;
    }
    case FlashBank::kFlashBank2: {
      if (HAL_FLASHEx_Lock_Bank2() != HAL_OK) {
        PW_LOG_ERROR("Could not lock flash memory.");
        HAL_NVIC_DisableIRQ(FLASH_IRQn);
        return pw::Status::Internal();
      }
      break;
    }
    default: {
      PW_LOG_ERROR("Undefined flash bank.");
      HAL_NVIC_DisableIRQ(FLASH_IRQn);
      return pw::Status::Internal();
    }
  }

  return pw::OkStatus();
}

pw::Status MemoryDriver::FlashLock(bool lock_on) {
  if (lock_on) {
    if (HAL_FLASH_Lock() != HAL_OK) {
      return pw::Status::Internal();
    }
  } else {
    if (HAL_FLASH_Unlock() != HAL_OK) {
      return pw::Status::Internal();
    }
  }
  return pw::OkStatus();
}

pw::Status MemoryDriver::WriteFlash(uint32_t flash_address,
                                    uint32_t data_address) {
  return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_address,
                            data_address) == HAL_OK
              ? pw::OkStatus()
              : pw::Status::Internal());
}

pw::Status MemoryDriver::ValidateFlash(uint32_t flash_address,
                                       uint32_t data_address, size_t length) {
  if (memcmp(reinterpret_cast<uint32_t*>(data_address),
             reinterpret_cast<uint32_t*>(flash_address), length)) {
    return pw::Status::Internal();
  }
  return pw::OkStatus();
}

pw::Status MemoryDriver::SwapFlashBanks() {
  if (HAL_FLASH_OB_Unlock() != HAL_OK) {
    PW_LOG_ERROR("HAL_FLASH_OB_Unlock failed.");
    return pw::Status::Internal();
  }

  // Retrieves the dual boot configuration status to swap banks before booting
  FLASH_OBProgramInitTypeDef OBInit;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  OBInit.OptionType = OPTIONBYTE_USER;
  OBInit.USERType = OB_USER_SWAP_BANK;

  // Invert bank swap configuration
  if (READ_BIT(FLASH->OPTSR_PRG, FLASH_OPTSR_SWAP_BANK_OPT)) {
    OBInit.USERConfig = OB_SWAP_BANK_DISABLE;
  } else {
    OBInit.USERConfig = OB_SWAP_BANK_ENABLE;
  }

  if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK) {
    PW_LOG_ERROR("HAL_FLASHEx_OBProgram failed. Aborting bank swap.");
    return pw::Status::Internal();
  }
  HAL_FLASH_OB_Launch();

  if (HAL_FLASH_OB_Lock() != HAL_OK) {
    PW_LOG_ERROR("Could not lock option bytes memory.");
    return pw::Status::Internal();
  }
  return pw::OkStatus();
}

MemoryDriver::MemoryDriver() {
  // Set FLASH interrupt priority.
  HAL_NVIC_SetPriority(FLASH_IRQn,
                       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3, 0);
}

void flash_irq_handler(pw::Status flash_op_status) {
  MemoryDriver::Get().flash_callback_(flash_op_status);
}

#ifdef __cplusplus
extern "C" {
#endif

void FLASH_IRQHandler(void) { HAL_FLASH_IRQHandler(); }

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue) {
  HAL_NVIC_DisableIRQ(FLASH_IRQn);
  flash_irq_handler(pw::OkStatus());
}

void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue) {
  HAL_NVIC_DisableIRQ(FLASH_IRQn);
  flash_irq_handler(pw::Status::Internal());
}

#ifdef __cplusplus
}
#endif

}  // namespace barkour
