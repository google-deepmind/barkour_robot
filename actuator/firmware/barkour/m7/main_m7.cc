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

#include <array>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include "actuator/firmware/barkour/common/shared_memory.h"
#include "actuator/firmware/barkour/m7/board_setup_m7.h"
#include "FreeRTOS.h" // NOLINT
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT
#include "pw_assert/check.h"
#include "pw_system/init.h"

#define HSEM_ID_0 (0U)
constexpr int kMainTaskStackSizeBytes = 1024;

// If something goes wrong during initialisation, then enter prvErrorHandler.
// Presently, the handler does nothing, and will spin forever.
static void prvErrorHandler(void);

// Primary FreeRTOS task function for the M7 core.
// Presently blinks LED1 to indicate that the FreeRTOS kernel is operating.
static void prvM7MainTask(void* pvParameters);

// FreeRTOS tasks
std::array<StackType_t, configMINIMAL_STACK_SIZE> freertos_idle_stack;
StaticTask_t freertos_idle_tcb;

StaticTask_t main_tcb;
std::array<StackType_t, kMainTaskStackSizeBytes> main_task_stack;

int main(void) {
  barkour::MPU_Config();
  barkour::CPU_CACHE_Enable();

  // Copy the Unique ID from system memory (only accessible by the M7 core) to
  // shared memory.

  // Fixed address, from the STM32h745/755 reference manual.
  uint32_t* kStm32UniqueIdBaseAddress =
      reinterpret_cast<uint32_t*>(0x1ff1'e800);

  std::memcpy(reinterpret_cast<uint32_t*>(barkour::kUidWord0Address),
              kStm32UniqueIdBaseAddress, sizeof(uint32_t));
  std::memcpy(reinterpret_cast<uint32_t*>(barkour::kUidWord1Address),
              kStm32UniqueIdBaseAddress + 1, sizeof(uint32_t));
  std::memcpy(reinterpret_cast<uint32_t*>(barkour::kUidWord2Address),
              kStm32UniqueIdBaseAddress + 2, sizeof(uint32_t));

  // Wait for M4 to boot and enter STOP mode.
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) {
  }

  if (HAL_Init() != HAL_OK) {
    prvErrorHandler();
  }

  // If the boot addresses for the M4 and M7 do not match the
  // start of the program memory (see linker files), we will
  // update them and reboot the device.
  // In particular, the M4 boots up from 0x08100000 by default,
  // which doesn't match the FoE firmware update setup.

  FLASH_OBProgramInitTypeDef init_config;
  HAL_FLASHEx_OBGetConfig(&init_config);
  uint32_t m4_boot_add_0 = init_config.CM4BootAddr0;
  uint32_t m7_boot_add_0 = init_config.BootAddr0;

  const uint32_t m4_boot_add_0_default = 0x08080000;
  const uint32_t m7_boot_add_0_default = 0x08000000;

  if (m4_boot_add_0 == m4_boot_add_0_default &&
      m7_boot_add_0 == m7_boot_add_0_default) {
    // Valid.
  } else {
    // Invalid. Write boot addresses.
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    FLASH_OBProgramInitTypeDef boot_options_m4;
    boot_options_m4.OptionType = OPTIONBYTE_CM4_BOOTADD;
    boot_options_m4.CM4BootConfig = OB_BOOT_ADD0;
    boot_options_m4.CM4BootAddr0 = m4_boot_add_0_default;

    FLASH_OBProgramInitTypeDef boot_options_m7 = boot_options_m4;
    boot_options_m7.OptionType |= OPTIONBYTE_CM7_BOOTADD;
    boot_options_m7.BootConfig = OB_BOOT_ADD0;
    boot_options_m7.BootAddr0 = m7_boot_add_0_default;
    HAL_FLASHEx_OBProgram(&boot_options_m7);
    HAL_FLASH_OB_Launch();
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    // Reboot if a write occurred, to make sure the M4 can boot correctly.
    NVIC_SystemReset();
  }

  barkour::SystemClock_Config();

  // Upon releasing the HSEM, CM4 will wake up from STOP mode.
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);

  xTaskCreateStatic(prvM7MainTask, "MainTask", main_task_stack.size(), NULL,
                    tskIDLE_PRIORITY + 1, main_task_stack.data(), &main_tcb);

  // Start Pigweed logging, RPC and work queue threads.
  pw::system::Init();

  // Start the FreeRTOS scheduler.
  vTaskStartScheduler();

  while (1) {
  }
}

// This will run once after pw::system::Init() completes. This callback must
// return or it will block the work queue.
//
// This is the first thing run in a threaded context (specifically on the work
// queue thread).
namespace pw::system {
void UserAppInit() {}
}  // namespace pw::system

static void prvM7MainTask(void* pvParameters) {
  TickType_t xNextWakeTime;
  const TickType_t xCycleFrequency = pdMS_TO_TICKS(500UL);
  xNextWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xNextWakeTime, xCycleFrequency);
  }
}

static void prvErrorHandler(void) {
  while (1) {
  }
}

extern "C" {

// Provides the stack overflow hook function used by FreeRTOS.
//
// This functional will be called if configCHECK_FOR_STACK_OVERFLOW is defined
// to be 1 or 2, and a stack overflow is detected.
void vApplicationStackOverflowHook([[maybe_unused]] xTaskHandle xTask,
                                   char* pcTaskName) {
  static char safe_task_name[configMAX_TASK_NAME_LEN + 1] = {'\0'};
  strncat(safe_task_name, (const char*)pcTaskName, sizeof(safe_task_name) - 1);
  PW_CRASH("Stack overflow detected for task %s", safe_task_name);
}

// Provides pointers to FreeRTOS for the Idle task.
//
// This function is called when FreeRTOS is configured to support static memory
// allocation (configSUPPORT_STATIC_ALLOCATION).
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &freertos_idle_tcb;
  *ppxIdleTaskStackBuffer = freertos_idle_stack.data();
  *pulIdleTaskStackSize = freertos_idle_stack.size();
}

}  // extern C.
