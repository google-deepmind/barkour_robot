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

#ifndef BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M7_STM32H7XX_HAL_CONF_H_
#define BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M7_STM32H7XX_HAL_CONF_H_

// Low Power Internal oscillator (CSI) frequency, in Hz.
#define CSI_VALUE ((uint32_t)4000000)

// I2S_CKIN external oscillator frequency, in Hz.
#define EXTERNAL_CLOCK_VALUE 0U

// High-Speed External oscillator (HSE) frequency, in Hz.
#define HSE_VALUE ((uint32_t)25000000)
#define HSE_STARTUP_TIMEOUT ((uint32_t)5000)

// High-Speed Internal oscillator (HSI) frequency, in Hz.
#define HSI_VALUE ((uint32_t)64000000)

// Low Speed External oscillator (LSE) frequency, in Hz.
#define LSE_VALUE ((uint32_t)32768)
#define LSE_STARTUP_TIMEOUT ((uint32_t)5000)

// Low-Speed Internal oscillator (LSI) frequency, in Hz.
#define LSI_VALUE ((uint32_t)32000)

// HAL Configuration.
#define VDD_VALUE ((uint32_t)3300)
#define TICK_INT_PRIORITY ((uint32_t)0x0F)
#define USE_RTOS 0U
#define USE_SD_TRANSCEIVER 1U

#define assert_param(expr) ((void)0U)

// Ethernet Configuration.
#define ETH_TX_DESC_CNT 4
#define ETH_RX_DESC_CNT 4
#define ETH_MAC_ADDR0 ((uint8_t)0x02)
#define ETH_MAC_ADDR1 ((uint8_t)0x00)
#define ETH_MAC_ADDR2 ((uint8_t)0x00)
#define ETH_MAC_ADDR3 ((uint8_t)0x00)
#define ETH_MAC_ADDR4 ((uint8_t)0x00)
#define ETH_MAC_ADDR5 ((uint8_t)0x00)

// HAL Headers: comment out defines + include to remove primary HAL headers.
#define HAL_CORTEX_MODULE_ENABLED
#include "stm32h7xx_hal_cortex.h"  // NOLINT
#define HAL_DMA_MODULE_ENABLED
#include "stm32h7xx_hal_dma.h"  // NOLINT
#define HAL_EXTI_MODULE_ENABLED
#include "stm32h7xx_hal_exti.h"  // NOLINT
#define HAL_GPIO_MODULE_ENABLED
#include "stm32h7xx_hal_gpio.h"  // NOLINT
#define HAL_RCC_MODULE_ENABLED
#include "stm32h7xx_hal_rcc.h"  // NOLINT

// remaining headers (can be commented out if desired).
#define HAL_FLASH_MODULE_ENABLED
#include "stm32h7xx_hal_flash.h"  // NOLINT

#define HAL_HSEM_MODULE_ENABLED
#include "stm32h7xx_hal_hsem.h"  // NOLINT

#define HAL_I2C_MODULE_ENABLED
#include "stm32h7xx_hal_i2c.h"  // NOLINT

#define HAL_PWR_MODULE_ENABLED
#include "stm32h7xx_hal_pwr.h"  // NOLINT

#define HAL_TIM_MODULE_ENABLED
#include "stm32h7xx_hal_tim.h"  // NOLINT

#define HAL_UART_MODULE_ENABLED
#include "stm32h7xx_hal_uart.h"  // NOLINT

#endif  // BARKOUR_ROBOT_FIRMWARE_TARGETS_BARKOUR_M7_STM32H7XX_HAL_CONF_H_
