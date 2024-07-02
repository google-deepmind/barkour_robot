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

#ifndef BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_FREERTOSCONFIG_H_
#define BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_FREERTOSCONFIG_H_

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

// Externally defined variables that must be forward-declared for FreeRTOS to
// use them.
extern uint32_t SystemD2Clock;

#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configSUPPORT_STATIC_ALLOCATION 1

#define configUSE_16_BIT_TICKS 0
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_CO_ROUTINES 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_IDLE_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_MUTEXES 1
#define configUSE_PREEMPTION 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_TICK_HOOK 0
#define configUSE_TIMERS 0
#define configUSE_TRACE_FACILITY 1

#define configGENERATE_RUN_TIME_STATS 0

// Because of the overhead associated with checking for stack overflow, this
// check is only enabled for debug builds.
// Method 2 provides improved likelihood of detecting stack overflow at the
// cost of checking for padding bytes during context switches.
#ifdef DEBUG
#define configCHECK_FOR_STACK_OVERFLOW 2
#endif

#define configCPU_CLOCK_HZ (SystemD2Clock)
#define configIDLE_SHOULD_YIELD 1
#define configMAX_CO_ROUTINE_PRIORITIES (2)
#define configMAX_PRIORITIES (7)
#define configMAX_TASK_NAME_LEN (16)
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t
#define configMINIMAL_STACK_SIZE ((uint16_t)128)
#define configQUEUE_REGISTRY_SIZE 8
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_PRIORITY (2)
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)
#define configTOTAL_HEAP_SIZE ((size_t)(20 * 1024))

// __NVIC_PRIO_BITS is defined in CMSIS.
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define INCLUDE_eTaskGetState 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskCleanUpResources 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xQueueGetMutexHolder 1
#define INCLUDE_xTaskGetSchedulerState 1

// Instead of defining configASSERT(), include a header that provides a
// definition that redirects to pw_assert.
#include "pw_third_party/freertos/config_assert.h"

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif  // BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M4_FREERTOSCONFIG_H_
