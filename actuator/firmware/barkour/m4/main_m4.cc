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

#include <stdbool.h>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>

#include "actuator/firmware/barkour/common/barkour_system.h"
#include "actuator/firmware/barkour/common/async_uart_logger.h"
#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/common/code_version.h"
#include "actuator/firmware/barkour/common/cycle_counter.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "actuator/firmware/barkour/common/multiturn_encoder_wrapper.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/aksim2_encoder.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/ma732_encoder.h"
#include "actuator/firmware/barkour/m4/board_setup_m4.h"
#include "actuator/firmware/barkour/m4/controller_thread.h"
#include "actuator/firmware/barkour/m4/ecat/utypes.h"
#include "actuator/firmware/barkour/m4/ecat_device.h"
#include "actuator/firmware/barkour/m4/gpio.h"
#include "pw_assert/check.h"
#include "pw_function/function.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_system/init.h"
#include "pw_thread/detached_thread.h"
#include "FreeRTOS.h" // NOLINT
#include "task.h" // NOLINT

#if defined UNIT_TESTS_ENABLED || defined FUNCTIONAL_TESTS_ENABLED
#include "actuator/firmware/barkour/common/test_runner.h"
#endif

#define HSEM_ID_0 (0U)

// Number of bytes allocated to the stack of each RTOS.
constexpr uint16_t kLoggingTaskStackSizeBytes = 1024;
constexpr uint16_t kMainTaskStackSizeBytes = 2048;

// If something goes wrong in initialization, then enter prvErrorHandler.
// Presently, the handler does nothing, and will spin forever.
static void prvErrorHandler(void);

// Low-priority task for handling asynchronous logging.
static void prvLoggingTask(void* pvParameters);

// Primary FreeRTOS task function for the M4 core.
static void prvMainTask(void* pvParameters);

// FreeRTOS tasks
std::array<StackType_t, configMINIMAL_STACK_SIZE> freertos_idle_stack;
StaticTask_t freertos_idle_tcb;

StaticTask_t main_tcb;
std::array<StackType_t, kMainTaskStackSizeBytes> main_task_stack;

StaticTask_t logging_tcb;
std::array<StackType_t, kLoggingTaskStackSizeBytes> logging_task_stack;

UART_HandleTypeDef UartHandle;

int main(void) {
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
                          PWR_D2_DOMAIN);
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

  if (HAL_Init() != HAL_OK) {
    prvErrorHandler();
  }

  // Enable synchronous logging prior to starting the scheduler and
  // M4LoggingTask.
  barkour::AsyncUartLogger::Get().DisableAsynchronousLogging();
  PW_LOG_INFO("Booting");
  PW_LOG_INFO("Setting up GPIO");
  barkour::SetupGPIO();

  FLASH_OBProgramInitTypeDef init_config;
  HAL_FLASHEx_OBGetConfig(&init_config);
  PW_LOG_DEBUG("M4 boot address 0 %#lx", init_config.CM4BootAddr0);
  PW_LOG_DEBUG("M7 boot address 0 %#lx", init_config.BootAddr0);

  if (HAL_FLASH_OB_Unlock() != HAL_OK) {
    PW_LOG_ERROR("HAL_FLASH_OB_Unlock failed. Could not unlock option bytes.");
  }
  // Retrieves the dual boot configuration status.
  FLASH_OBProgramInitTypeDef OBInit;

  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);

  if ((OBInit.USERConfig & OB_SWAP_BANK_ENABLE) == OB_SWAP_BANK_DISABLE) {
    PW_LOG_INFO("Memory banks not swapped.");
  } else {
    PW_LOG_INFO("Memory banks swapped.");
  }
  if (HAL_FLASH_OB_Lock() != HAL_OK) {
    PW_LOG_ERROR("HAL_FLASH_OB_Lock failed. Could not lock option bytes.");
  }

  // Important: Set the address of the interrupt vector table.
  // This is located at the start of the ROM.
  // By default this value is 0, meaning the M4 will point its
  // interrupts to the M7's, which results in a less than optimal
  // experience.
  uint32_t* vtor = reinterpret_cast<uint32_t*>(0xE000ED08);
  *vtor = init_config.CM4BootAddr0;

  barkour::CycleCounter::Get().Enable();

  xTaskCreateStatic(prvLoggingTask, "M4LoggingTask", logging_task_stack.size(),
                    NULL, tskIDLE_PRIORITY + 1, logging_task_stack.data(),
                    &logging_tcb);
  xTaskCreateStatic(prvMainTask, "MainTask", main_task_stack.size(), NULL,
                    tskIDLE_PRIORITY + 2, main_task_stack.data(), &main_tcb);

  barkour::GPIO::Get().EnableInterrupts();

  // Start Pigweed logging, RPC and work queue threads.
  pw::system::Init();

  // This modules provides target-agnostic interfaces to hardware-backed
  // components used by the Barkour Motor Controller.

  // Initializes target-agnostic interfaces to hardware-backed components used
  // by the Barkour Motor Controller.
  PW_CHECK_OK(barkour::System::Get().Initialize());

  // Starts the FreeRTOS scheduler.
  vTaskStartScheduler();
}

// This will run once after pw::system::Init() completes. This callback must
// return or it will block the work queue.
//
// This is the first thing run in a threaded context (specifically on the work
// queue thread).
namespace pw::system {
void UserAppInit() {}
}  // namespace pw::system

static void prvErrorHandler(void) {
  while (1) {
  }
}

static void prvLoggingTask(void* pvParameters) {
  barkour::AsyncUartLogger::Get().EnableAsynchronousLogging();
  while (1) {
    barkour::AsyncUartLogger::Get().Update();
    vTaskDelay(20);
  }
}
#if defined UNIT_TESTS_ENABLED || defined FUNCTIONAL_TESTS_ENABLED
// Sleep for a short amount of time after each test, to avoid overloading the
// logger.
void PostTestSleep(void) { vTaskDelay(pdMS_TO_TICKS(100)); }
#endif
static void prvMainTask(void* pvParameters) {
  PW_LOG_INFO("Firmware built with code version: %s", barkour::kCodeVersion);

#if defined UNIT_TESTS_ENABLED || defined FUNCTIONAL_TESTS_ENABLED
  barkour::run_tests(pw::Function<void()>(PostTestSleep));
  PW_LOG_INFO("All unit tests done");

  for (uint8_t x = 5; x; x--) {
    vTaskDelay(pdMS_TO_TICKS(100));
    PW_LOG_INFO(" ");  // Flush the log.
  }
  barkour::BoardConfig::Get().DisableMotorPower();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

#endif

#if ENCODER_TYPE == ENCODER_MA732_ADA
  barkour::RotaryEncoder& encoder = barkour::Ma732Encoder::Get();
#elif ENCODER_TYPE == ENCODER_RLS_AKSIM2
  barkour::RotaryEncoder& encoder = barkour::AksIm2Encoder::Get();
#else
#error "Unknown encoder type."
#endif

  barkour::MultiturnEncoderWrapper encoder_wrapper(encoder);

  pw::sync::ThreadNotification controller_notification;
  pw::Result<barkour::TimerNotifierInterface*> maybe_timer_notifier =
      barkour::System::Get().GetControllerTimerNotifier();
  PW_CHECK_OK(maybe_timer_notifier.status());
  PW_CHECK_OK(maybe_timer_notifier.value()->RegisterThreadNotification(
      controller_notification));
  PW_CHECK_OK(maybe_timer_notifier.value()->Start());

  // Start controller thread.
  barkour::ControllerThread controller_thread(
      encoder_wrapper, controller_notification,
      1.0f / barkour::kControllerTimerNotifierUpdateFrequencyHz);
  pw::thread::DetachedThread(
      barkour::System::Get().GetControllerThreadOptions(), controller_thread);

  barkour::EthercatDevice& ecat_device = barkour::EthercatDevice::Get();
  ecat_device.Run();
}

extern "C" {
// Provides the stack overflow hook function used by FreeRTOS.
//
// This functional will be called if configCHECK_FOR_STACK_OVERFLOW is defined
// to be 1 or 2, and a stack overflow is detected.
void vApplicationStackOverflowHook([[maybe_unused]] xTaskHandle xTask,
                                   char* pcTaskName) {
  // Because pcTaskName may be an improperly formed string due to memory
  // corruption, strncat is used to safely limit the amount of data that will be
  // printed to the screen in the crash report.
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
#if defined(__GNUC__)
// Note: Logging is asynchronous until the log buffer fills up,
// at which point we will switch permanently to synchronous logging.
int _write(int fd, char* ptr, int len) {
  pw::Status status;
  status = barkour::AsyncUartLogger::Get().Log({ptr, (size_t)len});
  if (status.IsResourceExhausted()) {
    // Resource is exhausted, dump the current buffer and switch to blocking
    // logging.
    barkour::AsyncUartLogger::Get().DisableAsynchronousLogging();
    PW_LOG_WARN("Logging ring buffer overflow. Asynchronous logging disabled.");
  }
  return len;
}
#endif
// Memory map of the flash data for reading.
__attribute__((section(".ap_m4_data.flash_data"))) char flash_data[128000];
}  // extern C.
