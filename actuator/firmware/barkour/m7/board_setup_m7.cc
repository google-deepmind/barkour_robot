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

#include "actuator/firmware/barkour/m7/board_setup_m7.h"

#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {
static void prvErrorHandler(void);

void CPU_CACHE_Enable(void) {
  // Enable I-Cache.
  SCB_EnableICache();

  // Enable D-Cache.
  SCB_EnableDCache();
}

// Configure the MPU attributes as Not Cachable for Internal D3SRAM.
// The Base Address is 0x38000000 (D3_SRAM_BASE).
// The Configured Region Size is 64KB because same as D3SRAM size.
void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct;

  // Disable the MPU.
  HAL_MPU_Disable();

  // Configure the MPU attributes as WT for SRAM.
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Enable the MPU.
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

// System Clock Configuration.
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // Supply configuration update enable.
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  // Configure the main internal regulator output voltage.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  // Initializes the RCC Oscillators according to the specified parameters
  // in the RCC_OscInitTypeDef structure.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    prvErrorHandler();
  }

  // Initializes the CPU, AHB and APB buses clocks.
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    prvErrorHandler();
  }
  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_SPI5 | RCC_PERIPHCLK_SPI6 |
      RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_SPI3 | RCC_PERIPHCLK_SPI4;
  // 64Mhz -> /32 -> x120 -> /50 = 4.8Mhz
  PeriphClkInitStruct.PLL3.PLL3M = 32;
  PeriphClkInitStruct.PLL3.PLL3N = 120;
  PeriphClkInitStruct.PLL3.PLL3P = 10;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;  // 120Mhz
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL3;
  PeriphClkInitStruct.Usart234578ClockSelection =
      RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    prvErrorHandler();
  }
}

static void prvErrorHandler(void) {
  while (1) {
  }
}

}  // namespace barkour
