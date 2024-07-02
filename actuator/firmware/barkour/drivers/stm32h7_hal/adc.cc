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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/adc.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/interfaces/gpio_debug_interface.h"
#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_span/span.h"
#include "pw_status/status.h"

namespace barkour {

inline constexpr uint32_t kNumAdcOffsetCalibrationSamples = 100;

// Single channel DMA.
ADC_HandleTypeDef Adc1Handle = {
    .Instance = ADC1,
    .Init =
        {
            .ClockPrescaler = ADC_CLOCK_ASYNC_DIV2,  // 60MHz.
            .Resolution = ADC_RESOLUTION_16B,
            .ScanConvMode = ADC_SCAN_ENABLE,
            .EOCSelection = ADC_EOC_SINGLE_CONV,
            .LowPowerAutoWait = DISABLE,
            .ContinuousConvMode = ENABLE,
            .NbrOfConversion = 8,  // Value overwritten upon setup.
            .DiscontinuousConvMode = DISABLE,
            .ExternalTrigConv = ADC_SOFTWARE_START,
            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
            .ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR,
            .Overrun = ADC_OVR_DATA_OVERWRITTEN,
            .OversamplingMode = DISABLE,
        },
    .ConvCpltCallback = 0,
    .ConvHalfCpltCallback = 0,
    .LevelOutOfWindowCallback = 0,
    .ErrorCallback = 0,
    .InjectedConvCpltCallback = 0,
    .InjectedQueueOverflowCallback = 0,
    .LevelOutOfWindow2Callback = 0,
    .LevelOutOfWindow3Callback = 0,
    .EndOfSamplingCallback = 0,
    .MspInitCallback = 0,
    .MspDeInitCallback = 0,
};

ADC_HandleTypeDef Adc2Handle = {
    .Instance = ADC2,
    .Init =
        {
            .ClockPrescaler = ADC_CLOCK_ASYNC_DIV2,  // 60MHz.
            .Resolution = ADC_RESOLUTION_16B,
            .ScanConvMode = ADC_SCAN_ENABLE,
            .EOCSelection = ADC_EOC_SINGLE_CONV,
            .LowPowerAutoWait = DISABLE,
            .ContinuousConvMode = ENABLE,
            .NbrOfConversion = 1,  // Value overwritten upon setup.
            .DiscontinuousConvMode = DISABLE,
            .ExternalTrigConv = ADC_SOFTWARE_START,
            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
            .ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR,
            .Overrun = ADC_OVR_DATA_OVERWRITTEN,
            .OversamplingMode = DISABLE,
        },
    .ConvCpltCallback = 0,
    .ConvHalfCpltCallback = 0,
    .LevelOutOfWindowCallback = 0,
    .ErrorCallback = 0,
    .InjectedConvCpltCallback = 0,
    .InjectedQueueOverflowCallback = 0,
    .LevelOutOfWindow2Callback = 0,
    .LevelOutOfWindow3Callback = 0,
    .EndOfSamplingCallback = 0,
    .MspInitCallback = 0,
    .MspDeInitCallback = 0,
};

ADC_HandleTypeDef Adc3Handle = {
    .Instance = ADC3,
    .Init =
        {
            .ClockPrescaler = ADC_CLOCK_ASYNC_DIV2,  // 60MHz.
            .Resolution = ADC_RESOLUTION_16B,
            .ScanConvMode = ADC_SCAN_ENABLE,
            .EOCSelection = ADC_EOC_SINGLE_CONV,
            .LowPowerAutoWait = DISABLE,
            .ContinuousConvMode = ENABLE,
            .NbrOfConversion = 5,  // Value overwritten upon setup.
            .DiscontinuousConvMode = DISABLE,
            .ExternalTrigConv = ADC_SOFTWARE_START,
            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
            .ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR,
            .Overrun = ADC_OVR_DATA_OVERWRITTEN,
            .OversamplingMode = DISABLE,
        },
    .ConvCpltCallback = 0,
    .ConvHalfCpltCallback = 0,
    .LevelOutOfWindowCallback = 0,
    .ErrorCallback = 0,
    .InjectedConvCpltCallback = 0,
    .InjectedQueueOverflowCallback = 0,
    .LevelOutOfWindow2Callback = 0,
    .LevelOutOfWindow3Callback = 0,
    .EndOfSamplingCallback = 0,
    .MspInitCallback = 0,
    .MspDeInitCallback = 0,
};

ADC_ChannelConfTypeDef AdcChannelConfigBase = {
    .Channel = 0,  // Value provided on a per-channel basis in setup.
    .Rank = 0,     // Value provided on a per-channel basis in setup.
    .SamplingTime = ADC_SAMPLETIME_16CYCLES_5,  // with PLL3R@60Mhz: .275us.
    .SingleDiff = ADC_SINGLE_ENDED,
    .OffsetNumber = ADC_OFFSET_NONE,
    .Offset = 0,
};

// ADC Injected conversion is triggered by HRTIM's TRG2.
// With ADC clocked at 60MHz (120MHz/2), 16.5 clock cycles = .275us
ADC_InjectionConfTypeDef AdcInjectedConfigBase = {
    .InjectedChannel = 0,  // Value provided on a per-channel basis in setup.
    .InjectedRank = 0,     // Value provided on a per-channel basis in setup.
    .InjectedSamplingTime =
        ADC_SAMPLETIME_16CYCLES_5,  // @ 60MHz = 17ns, 17 * 16.5 = 275ns
                                    // (source impedance ~100ohm, 275ns > 162ns
                                    // (worst cases for low channel) per data
                                    // sheet)
    .InjectedSingleDiff = ADC_SINGLE_ENDED,
    .InjectedOffsetNumber = ADC_OFFSET_NONE,
    .InjectedOffset = 0,
    .InjectedNbrOfConversion = 1,
    .InjectedDiscontinuousConvMode = DISABLE,
    .AutoInjectedConv = DISABLE,
    .QueueInjectedContext = DISABLE,
    .ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HR1_ADCTRG2,
    .ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING,
    .InjecOversamplingMode = DISABLE,
};

typedef struct AdcChannelSettings {
  AdcAnalogSignal signal;
  ADC_TypeDef* adc;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint32_t channel;
  uint32_t rank;
  bool open_analog_switch;
} AdcChannelSettings;

constexpr size_t kAdcMaxChannels = 20;
ALIGN_32BYTES(static uint16_t adc1_sample_buffer[kAdcMaxChannels]);
ALIGN_32BYTES(static uint16_t adc2_sample_buffer[kAdcMaxChannels]);
ALIGN_32BYTES(static uint16_t adc3_sample_buffer[kAdcMaxChannels]);

std::array<AdcChannelSettings, 10> Adc1ChannelSettings = {{
    {AdcAnalogSignal::kPhaseACurrent, ADC1, GPIOA, GPIO_PIN_0, ADC_CHANNEL_0,
     ADC_REGULAR_RANK_1, true},
    {AdcAnalogSignal::kPhaseBCurrent, ADC1, GPIOA, GPIO_PIN_1, ADC_CHANNEL_1,
     ADC_REGULAR_RANK_2, true},
    {AdcAnalogSignal::kPhaseCCurrent, ADC1, GPIOF, GPIO_PIN_11, ADC_CHANNEL_2,
     ADC_REGULAR_RANK_3, false},
    {AdcAnalogSignal::kPhaseAVoltage, ADC1, GPIOA, GPIO_PIN_6, ADC_CHANNEL_3,
     ADC_REGULAR_RANK_4, false},
    {AdcAnalogSignal::kPhaseBVoltage, ADC1, GPIOC, GPIO_PIN_4, ADC_CHANNEL_4,
     ADC_REGULAR_RANK_5, false},
    {AdcAnalogSignal::kPhaseCVoltage, ADC1, GPIOB, GPIO_PIN_1, ADC_CHANNEL_5,
     ADC_REGULAR_RANK_6, false},
    {AdcAnalogSignal::kDrainSourceVoltageA, ADC1, GPIOF, GPIO_PIN_12,
     ADC_CHANNEL_6, ADC_REGULAR_RANK_7, false},
    {AdcAnalogSignal::kDrainSourceVoltageB, ADC1, GPIOB, GPIO_PIN_0,
     ADC_CHANNEL_9, ADC_REGULAR_RANK_8, false},
    {AdcAnalogSignal::kDrainSourceVoltageC, ADC1, GPIOC, GPIO_PIN_3,
     ADC_CHANNEL_13, ADC_REGULAR_RANK_9, false},
    {AdcAnalogSignal::k24VSafeVoltage, ADC1, GPIOA, GPIO_PIN_0, ADC_CHANNEL_16,
     ADC_REGULAR_RANK_10, true},
}};

std::array<AdcChannelSettings, 4> Adc2ChannelSettings = {{
    {AdcAnalogSignal::kMotorCurrent, ADC2, GPIOF, GPIO_PIN_13, ADC_CHANNEL_2,
     ADC_REGULAR_RANK_1, false},
    {AdcAnalogSignal::kBusVoltage, ADC2, GPIOF, GPIO_PIN_14, ADC_CHANNEL_6,
     ADC_REGULAR_RANK_2, false},
    {AdcAnalogSignal::kPhaseACurrent, ADC2, GPIOA, GPIO_PIN_0, ADC_CHANNEL_0,
     ADC_REGULAR_RANK_3, true},
    {AdcAnalogSignal::kPhaseBCurrent, ADC2, GPIOA, GPIO_PIN_1, ADC_CHANNEL_1,
     ADC_REGULAR_RANK_4, true},
}};

std::array<AdcChannelSettings, 5> Adc3ChannelSettings = {{
    {AdcAnalogSignal::kMotorTherm1, ADC3, GPIOC, GPIO_PIN_3, ADC_CHANNEL_1,
     ADC_REGULAR_RANK_1, true},
    {AdcAnalogSignal::kMotorTherm2, ADC3, GPIOF, GPIO_PIN_7, ADC_CHANNEL_3,
     ADC_REGULAR_RANK_2, false},
    {AdcAnalogSignal::kMotorTherm3, ADC3, GPIOF, GPIO_PIN_5, ADC_CHANNEL_4,
     ADC_REGULAR_RANK_3, false},
    {AdcAnalogSignal::kEncoderTherm, ADC3, GPIOF, GPIO_PIN_3, ADC_CHANNEL_5,
     ADC_REGULAR_RANK_4, false},
    {AdcAnalogSignal::kDriveTherm, ADC3, GPIOF, GPIO_PIN_10, ADC_CHANNEL_6,
     ADC_REGULAR_RANK_5, false},
}};

// Converts ST rank to DMA buffer index.
constexpr pw::Result<size_t> RankToDmaBufferIndex(const uint32_t rank) {
  size_t index = 0;
  switch (rank) {
    case ADC_REGULAR_RANK_1: {
      index = 0;
      break;
    }
    case ADC_REGULAR_RANK_2: {
      index = 1;
      break;
    }
    case ADC_REGULAR_RANK_3: {
      index = 2;
      break;
    }
    case ADC_REGULAR_RANK_4: {
      index = 3;
      break;
    }
    case ADC_REGULAR_RANK_5: {
      index = 4;
      break;
    }
    case ADC_REGULAR_RANK_6: {
      index = 5;
      break;
    }
    case ADC_REGULAR_RANK_7: {
      index = 6;
      break;
    }
    case ADC_REGULAR_RANK_8: {
      index = 7;
      break;
    }
    case ADC_REGULAR_RANK_9: {
      index = 8;
      break;
    }
    case ADC_REGULAR_RANK_10: {
      index = 9;
      break;
    }
    case ADC_REGULAR_RANK_11: {
      index = 10;
      break;
    }
    case ADC_REGULAR_RANK_12: {
      index = 11;
      break;
    }
    case ADC_REGULAR_RANK_13: {
      index = 12;
      break;
    }
    case ADC_REGULAR_RANK_14: {
      index = 13;
      break;
    }
    case ADC_REGULAR_RANK_15: {
      index = 14;
      break;
    }
    case ADC_REGULAR_RANK_16: {
      index = 15;
      break;
    }
    default: {
      return pw::Result<size_t>(pw::Status::OutOfRange());
    }
  }
  return pw::Result<size_t>(index);
}

void AdcErrorCB(ADC_HandleTypeDef* hadc) { PW_CRASH("ADC error encountered."); }

// This function configures ADC hardware resources.
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  static DMA_HandleTypeDef DmaHandleAdc1;
  static DMA_HandleTypeDef DmaHandleAdc2;
  static DMA_HandleTypeDef DmaHandleAdc3;
  HAL_StatusTypeDef result;

  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // PA0 - ADC_24vSafe (ADC1_INP16)
    // PA0_C -  MOTOR_ADC1 (ADC1_INP0)
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA1_C -  MOTOR_ADC2 (ADC1_INP1)
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA0_OPEN);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA6 - P2P_ADC1 (ADC1_INP3)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0 - Gate_Out_B (ADC1_INP9)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB1 - P2P_ADC3 (ADC1_INP5)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC3 - Gate_Out_C (ADC1_INP13)
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PC4 - P2P_ADC2 (ADC1_INP4)
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PF11 - MOTOR_ADC3 (ADC1_INP2)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // PF12 - Gate_Out_A (ADC1_INP6)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // DMA Configuration.
    DmaHandleAdc1.Instance = DMA1_Stream0;
    DmaHandleAdc1.Init.Request = DMA_REQUEST_ADC1;
    DmaHandleAdc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandleAdc1.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandleAdc1.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandleAdc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DmaHandleAdc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DmaHandleAdc1.Init.Mode = DMA_CIRCULAR;
    DmaHandleAdc1.Init.Priority = DMA_PRIORITY_MEDIUM;

    // Deinitialize  & Initialize the DMA for new transfer.
    result = HAL_DMA_DeInit(&DmaHandleAdc1);
    PW_CHECK(HAL_OK == result);
    result = HAL_DMA_Init(&DmaHandleAdc1);
    PW_CHECK(HAL_OK == result);

    // Associate the new DMA handle.
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandleAdc1);

    // NVIC configuration for DMA Input data interrupt.
    // Note: Leaving this commented out intentionally, as a reminder that DMA
    // interrupts can be enabled at a later time.
    // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    // NVIC configuration for Injected interrupt.
    HAL_NVIC_SetPriority(ADC_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  }

  if (hadc->Instance == ADC2) {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // PF13 -  MOTOR_ADC_IN (ADC2_INP2)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // PF14 - ADC_Pwr_in (ADC2_INP6)
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // DMA Configuration.
    DmaHandleAdc2.Instance = DMA1_Stream1;
    DmaHandleAdc2.Init.Request = DMA_REQUEST_ADC2;
    DmaHandleAdc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandleAdc2.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandleAdc2.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandleAdc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DmaHandleAdc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DmaHandleAdc2.Init.Mode = DMA_CIRCULAR;
    DmaHandleAdc2.Init.Priority = DMA_PRIORITY_MEDIUM;

    // Deinitialize  & Initialize the DMA for new transfer.
    result = HAL_DMA_DeInit(&DmaHandleAdc2);
    PW_CHECK(HAL_OK == result);
    result = HAL_DMA_Init(&DmaHandleAdc2);
    PW_CHECK(HAL_OK == result);

    // Associate the new DMA handle.
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandleAdc2);

    // NVIC configuration for DMA Input data interrupt.
    // Note: Leaving this commented out intentionally, as a reminder that DMA
    // interrupts can be enabled at a later time.
    // HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  }

  if (hadc->Instance == ADC3) {
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // PC2_C - CUR_HOTEL (ADC3_INP0)
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PC3_C -  ADC_Therm1 (ADC3_INP1)
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PF3 - ADC_Therm4 (ADC3_INP5)
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // PF5 - ADC_Therm3 (ADC3_INP4)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // PF7 - ADC_Therm2 (ADC3_INP3)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // PF10 - ADC_T_FET (ADC3_INP6)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // DMA Configuration.
    DmaHandleAdc3.Instance = DMA1_Stream2;
    DmaHandleAdc3.Init.Request = DMA_REQUEST_ADC3;
    DmaHandleAdc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandleAdc3.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandleAdc3.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandleAdc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DmaHandleAdc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DmaHandleAdc3.Init.Mode = DMA_CIRCULAR;
    DmaHandleAdc3.Init.Priority = DMA_PRIORITY_MEDIUM;

    // Deinitialize  & Initialize the DMA for new transfer.
    result = HAL_DMA_DeInit(&DmaHandleAdc3);
    PW_CHECK(HAL_OK == result);
    result = HAL_DMA_Init(&DmaHandleAdc3);
    PW_CHECK(HAL_OK == result);

    // Associate the new DMA handle.
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandleAdc3);

    // NVIC configuration for DMA Input data interrupt.
    // Note: Leaving this commented out intentionally, as a reminder that DMA
    // interrupts can be enabled at a later time.
    // HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  }
}

// This function frees ADC resources.
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC12_CLK_DISABLE();
    __HAL_RCC_ADC12_FORCE_RESET();
    __HAL_RCC_ADC12_RELEASE_RESET();

    // PA0 - ADC_24vSafe
    // PA0_C -  MOTOR_ADC1
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    // PA1_C -  MOTOR_ADC2
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    // PA6 - P2P_ADC1
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    // PB0 - Gate_Out_B
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

    // PB1 - P2P_ADC3
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    // PC3 - Gate_Out_C
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

    // PC4 - P2P_ADC2
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

    // PF11 - MOTOR_ADC3
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_11);

    // PF12 - ADC_24VSafe
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_12);

    // PF12 - Gate_Out_A
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_12);
  }

  if (hadc->Instance == ADC2) {
    __HAL_RCC_ADC12_CLK_DISABLE();
    __HAL_RCC_ADC12_FORCE_RESET();
    __HAL_RCC_ADC12_RELEASE_RESET();

    // PF13 - MOTOR_ADC_IN
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_13);

    // PF14 - ADC_Pwr_in
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14);
  }

  if (hadc->Instance == ADC3) {
    __HAL_RCC_ADC3_CLK_DISABLE();
    __HAL_RCC_ADC12_FORCE_RESET();
    __HAL_RCC_ADC12_RELEASE_RESET();

    // PC2_C -  CUR_HOTEL
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);

    // PC3_C -  ADC_Therm1
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

    // PF3 - ADC_Therm4
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3);

    // PF5 - ADC_Therm3
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_5);

    // PF7 - ADC_Therm2
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7);

    // PF10 - ADC_T_FET
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);
  }
}

Adc::Adc()
    : injected_sample_semaphore_(xSemaphoreCreateCountingStatic(
          std::numeric_limits<UBaseType_t>::max(), 0,
          &injected_sample_semaphore_buffer_)),
      phases_(PhaseSampleSelection::kABC),
      offset_adc1_0_(0x7FFF),
      offset_adc1_1_(0x7FFF),
      offset_adc2_0_(0x7FFF),
      offset_adc2_1_(0x7FFF),
      calibration_state_(Adc::CalibrationState::kAdcOcStatePending) {
  PW_ASSERT(injected_sample_semaphore_);

  // Fill signal_to_buffer_address_ for all channels in ADC1
  for (const auto& channel_setting : Adc1ChannelSettings) {
    auto index_result = RankToDmaBufferIndex(channel_setting.rank);
    PW_CHECK_OK(index_result.status());
    uint16_t* location = &adc1_sample_buffer[index_result.value()];
    signal_to_buffer_address_[(int)channel_setting.signal] = location;
  }

  // Fill signal_to_buffer_address_ for all channels in ADC2
  for (const auto& channel_setting : Adc2ChannelSettings) {
    auto index_result = RankToDmaBufferIndex(channel_setting.rank);
    PW_CHECK_OK(index_result.status());
    uint16_t* location = &adc2_sample_buffer[index_result.value()];
    signal_to_buffer_address_[(int)channel_setting.signal] = location;
  }

  // Fill signal_to_buffer_address_ for all channels in ADC3
  for (const auto& channel_setting : Adc3ChannelSettings) {
    auto index_result = RankToDmaBufferIndex(channel_setting.rank);
    PW_CHECK_OK(index_result.status());
    uint16_t* location = &adc3_sample_buffer[index_result.value()];
    signal_to_buffer_address_[(int)channel_setting.signal] = location;
  }

  SetupHardware();
}

Adc& Adc::Get() {
  static Adc adc;
  return adc;
}

void Adc::SetupHardware() {
  PW_LOG_INFO("Setting up ADCs.");
  HAL_StatusTypeDef result;

  // MspInit/MspDeInit callbacks must be registered prior to HAL_ADC_Init.
  result = HAL_ADC_RegisterCallback(&Adc1Handle, HAL_ADC_MSPINIT_CB_ID,
                                    &HAL_ADC_MspInit);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_RegisterCallback(&Adc2Handle, HAL_ADC_MSPINIT_CB_ID,
                                    &HAL_ADC_MspInit);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_RegisterCallback(&Adc3Handle, HAL_ADC_MSPINIT_CB_ID,
                                    &HAL_ADC_MspInit);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_DeInit(&Adc1Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_DeInit(&Adc2Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_DeInit(&Adc3Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  Adc1Handle.Init.NbrOfConversion = Adc1ChannelSettings.size();
  result = HAL_ADC_Init(&Adc1Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  Adc2Handle.Init.NbrOfConversion = Adc2ChannelSettings.size();
  result = HAL_ADC_Init(&Adc2Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  Adc3Handle.Init.NbrOfConversion = Adc3ChannelSettings.size();
  result = HAL_ADC_Init(&Adc3Handle);
  PW_CHECK(HAL_OK == result, "%u", result);

  result =
      HAL_ADC_RegisterCallback(&Adc1Handle, HAL_ADC_ERROR_CB_ID, &AdcErrorCB);
  PW_CHECK(HAL_OK == result, "%u", result);

  result =
      HAL_ADC_RegisterCallback(&Adc2Handle, HAL_ADC_ERROR_CB_ID, &AdcErrorCB);
  PW_CHECK(HAL_OK == result, "%u", result);

  result =
      HAL_ADC_RegisterCallback(&Adc3Handle, HAL_ADC_ERROR_CB_ID, &AdcErrorCB);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADCEx_Calibration_Start(&Adc1Handle, ADC_CALIB_OFFSET,
                                       ADC_SINGLE_ENDED);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADCEx_Calibration_Start(&Adc2Handle, ADC_CALIB_OFFSET,
                                       ADC_SINGLE_ENDED);
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADCEx_Calibration_Start(&Adc3Handle, ADC_CALIB_OFFSET,
                                       ADC_SINGLE_ENDED);
  PW_CHECK(HAL_OK == result, "%u", result);

  // Configure ADC1 channels.
  for (const auto& channel_setting : Adc1ChannelSettings) {
    AdcChannelConfigBase.Channel = channel_setting.channel;
    AdcChannelConfigBase.Rank = channel_setting.rank;
    result = HAL_ADC_ConfigChannel(&Adc1Handle, &AdcChannelConfigBase);
    PW_CHECK(HAL_OK == result, "%u", result);
  }

  // Configure ADC2 channels.
  for (const auto& channel_setting : Adc2ChannelSettings) {
    AdcChannelConfigBase.Channel = channel_setting.channel;
    AdcChannelConfigBase.Rank = channel_setting.rank;
    result = HAL_ADC_ConfigChannel(&Adc2Handle, &AdcChannelConfigBase);
    PW_CHECK(HAL_OK == result, "%u", result);
  }

  // Configure ADC3 channels.
  for (const auto& channel_setting : Adc3ChannelSettings) {
    AdcChannelConfigBase.Channel = channel_setting.channel;
    AdcChannelConfigBase.Rank = channel_setting.rank;
    result = HAL_ADC_ConfigChannel(&Adc3Handle, &AdcChannelConfigBase);
    PW_CHECK(HAL_OK == result, "%u", result);
  }

  result = HAL_ADC_Start_DMA(&Adc1Handle, (uint32_t*)adc1_sample_buffer,
                             Adc1ChannelSettings.size());
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_Start_DMA(&Adc2Handle, (uint32_t*)adc2_sample_buffer,
                             Adc2ChannelSettings.size());
  PW_CHECK(HAL_OK == result, "%u", result);

  result = HAL_ADC_Start_DMA(&Adc3Handle, (uint32_t*)adc3_sample_buffer,
                             Adc3ChannelSettings.size());
  PW_CHECK(HAL_OK == result, "%u", result);

  // only using 1 channel per ADC.
  AdcInjectedConfigBase.InjectedRank = ADC_INJECTED_RANK_1;
  HAL_ADCEx_DisableInjectedQueue(&Adc1Handle);
  HAL_ADCEx_DisableInjectedQueue(&Adc2Handle);

  // Gets over ridden by FOC.
  SetInjectedChannels(PhaseSampleSelection::kAB);

  // Override conversion interrupt mode for the injected conversions.
  Adc1Handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  // Assume both ADCs would complete at same time, just use adc1 interrupt
  // handler.
  result = HAL_ADCEx_InjectedStart_IT(&Adc1Handle);
  result = HAL_ADCEx_InjectedStart(&Adc2Handle);
  PW_CHECK(HAL_OK == result, "%u", result);
}

void Adc::SetInjectedChannels(PhaseSampleSelection phases) {
  uint32_t adc1_ch = ADC_CHANNEL_0;  // Phase A
  uint32_t adc2_ch = ADC_CHANNEL_1;  // Phase B

  phases_ = phases;
  switch (phases) {
    case PhaseSampleSelection::kAB: {
      adc1_ch = ADC_CHANNEL_0;  // A
      adc2_ch = ADC_CHANNEL_1;  // B
      break;
    }
    case PhaseSampleSelection::kAC: {
      adc1_ch = ADC_CHANNEL_2;  // C
      adc2_ch = ADC_CHANNEL_0;  // A
      break;
    }
    case PhaseSampleSelection::kBC: {
      adc1_ch = ADC_CHANNEL_2;  // C
      adc2_ch = ADC_CHANNEL_1;  // B
      break;
    }
    default: {
      break;
    }
  }

  // Adc1 injected channel config.
  AdcInjectedConfigBase.InjectedChannel = adc1_ch;
  HAL_StatusTypeDef result =
      HAL_ADCEx_InjectedConfigChannel(&Adc1Handle, &AdcInjectedConfigBase);
  PW_CHECK(HAL_OK == result, "%u", result);

  // Adc2 injected channel config.
  AdcInjectedConfigBase.InjectedChannel = adc2_ch;
  result = HAL_ADCEx_InjectedConfigChannel(&Adc2Handle, &AdcInjectedConfigBase);
  PW_CHECK(HAL_OK == result, "%u", result);
}

pw::Result<int32_t> Adc::GetSample(const AdcAnalogSignal& signal) {
  uint16_t* value_address =
      signal_to_buffer_address_[static_cast<uint8_t>(signal)];
  if (nullptr == value_address) {
    return pw::Result<int32_t>(pw::Status::NotFound());
  }
  taskENTER_CRITICAL();
  uint16_t value = *value_address;
  taskEXIT_CRITICAL();
  return pw::Result<int32_t>(value);
}

pw::Result<int32_t> Adc::GetInjectedSample(const AdcAnalogSignal& signal) {
  int32_t* sample_address;
  int32_t sample;
  switch (signal) {
    case AdcAnalogSignal::kPhaseACurrent: {
      sample_address = &motor_adc_samples_injected[0];
      break;
    }
    case AdcAnalogSignal::kPhaseBCurrent: {
      sample_address = &motor_adc_samples_injected[1];
      break;
    }
    case AdcAnalogSignal::kPhaseCCurrent: {
      sample_address = &motor_adc_samples_injected[2];
      break;
    }
    default: {
      return pw::Result<int32_t>(pw::Status::Unavailable());
    }
  }
  taskENTER_CRITICAL();
  sample = *sample_address;
  taskEXIT_CRITICAL();
  return pw::Result<int32_t>(sample);
}

uint32_t Adc::WaitForInjectedConversion(uint32_t block_duration) {
  int32_t count = uxSemaphoreGetCount(injected_sample_semaphore_);

  if (count) {
    xQueueReset(injected_sample_semaphore_);
  } else {
    xSemaphoreTake(injected_sample_semaphore_, block_duration);
    count = 1;
  }

  return count;
}

SemaphoreHandle_t Adc::GetInjectedSampleSemaphore() const {
  return injected_sample_semaphore_;
}

// Return active injected channel offsets.
void Adc::GetCalOffsets(int32_t& offset1, int32_t& offset2) {
  if (IsCalibrated()) {
    offset1 = (phases_ == barkour::PhaseSampleSelection::kAB ? offset_adc1_0_
                                                        : offset_adc1_1_);
    offset2 = (phases_ == barkour::PhaseSampleSelection::kAC ? offset_adc2_0_
                                                        : offset_adc2_1_);
  } else {
    offset1 = 0x7fff;
    offset2 = 0x7fff;
  }
}

// Initiate calibration of ADC channel offsets.
void Adc::StartOffsetCalibration(void) {
  calibration_state_ = Adc::CalibrationState::kAdcOcStateInit;
}

// Run the calibration state machine.
void Adc::DoCalibration(uint32_t adc1, uint32_t adc2) {
  switch (calibration_state_) {
    case Adc::CalibrationState::kAdcOcStatePending: {
      break;  // wait till calibration is started
    }

    // Setup to measure phase A/B
    case Adc::CalibrationState::kAdcOcStateInit: {
      offset_adc1_0_ = 0;
      offset_adc1_1_ = 0;
      offset_adc2_0_ = 0;
      offset_adc2_1_ = 0;
      SetInjectedChannels(barkour::PhaseSampleSelection::kAB);
      calibration_sample_count_ = 0;
      calibration_state_ = Adc::CalibrationState::kAdcOcStateAb;
      break;
    }

    // Read channel A/B offsets, setup to measure A/C
    case Adc::CalibrationState::kAdcOcStateAb: {
      offset_adc1_0_ += adc1;
      offset_adc2_1_ += adc2;
      calibration_sample_count_++;
      if (calibration_sample_count_ >= kNumAdcOffsetCalibrationSamples) {
        SetInjectedChannels(barkour::PhaseSampleSelection::kAC);
        calibration_sample_count_ = 0;
        calibration_state_ = Adc::CalibrationState::kAdcOcStateAc;
      }
      break;
    }

    // Read channel A/C offsets, setup to measure B/C
    case Adc::CalibrationState::kAdcOcStateAc: {
      offset_adc1_1_ += adc1;
      offset_adc2_0_ += adc2;
      calibration_sample_count_++;
      if (calibration_sample_count_ >= kNumAdcOffsetCalibrationSamples) {
        SetInjectedChannels(barkour::PhaseSampleSelection::kBC);
        calibration_sample_count_ = 0;
        calibration_state_ = Adc::CalibrationState::kAdcOcStateBc;
      }
      break;
    }

    // Read channel B/C offsets, setup to measure A/B
    case Adc::CalibrationState::kAdcOcStateBc: {
      offset_adc1_1_ += adc1;
      offset_adc2_1_ += adc2;
      calibration_sample_count_++;

      if (calibration_sample_count_ >= kNumAdcOffsetCalibrationSamples) {
        SetInjectedChannels(barkour::PhaseSampleSelection::kAB);

        // Calculate offsets as averages of measured values.
        offset_adc1_0_ /= kNumAdcOffsetCalibrationSamples;
        offset_adc1_1_ /= kNumAdcOffsetCalibrationSamples * 2;
        offset_adc2_0_ /= kNumAdcOffsetCalibrationSamples;
        offset_adc2_1_ /= kNumAdcOffsetCalibrationSamples * 2;

        PW_LOG_DEBUG(
            "ADC Calibration Done: offset 1/0=%ld, offset 1/1=%ld, offset "
            "2/0=%ld, offset 2/1=%ld",
            offset_adc1_0_, offset_adc1_1_, offset_adc2_0_, offset_adc2_1_);

        calibration_state_ = Adc::CalibrationState::kAdcOcStateDone;
      }
      break;
    }

    // All done
    case Adc::CalibrationState::kAdcOcStateDone: {
      break;
    }

    default: {
      break;
    }
  }
}

}  // namespace barkour

#ifdef __cplusplus
extern "C" {
#endif

// Process Injected ADC Interrupts.
void ADC_IRQHandler(void) {
  DBG_GPIO(true);
  volatile uint32_t tmp_isr = barkour::Adc1Handle.Instance->ISR;
  volatile uint32_t tmp_ier = barkour::Adc1Handle.Instance->IER;

  // ====== Check ADC group injected end of unitary conversion sequence
  // conversions =====
  if ((((tmp_isr & ADC_FLAG_JEOC) == ADC_FLAG_JEOC) &&
       ((tmp_ier & ADC_IT_JEOC) == ADC_IT_JEOC)) ||
      (((tmp_isr & ADC_FLAG_JEOS) == ADC_FLAG_JEOS) &&
       ((tmp_ier & ADC_IT_JEOS) == ADC_IT_JEOS))) {
    if (((tmp_isr & ADC_FLAG_JEOS) == ADC_FLAG_JEOS)) {
      // Only care about EOS.

      HAL_ADCEx_InjectedConvCpltCallback(&barkour::Adc1Handle);
      // Clear injected group eoc and eos conversion flag.
      __HAL_ADC_CLEAR_FLAG(&barkour::Adc1Handle, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    } else {
      // Clear injected group eoc conversion flag.
      __HAL_ADC_CLEAR_FLAG(&barkour::Adc1Handle, ADC_FLAG_JEOC);
    }

  } else {
    // Let default handler process non injected samples.
    HAL_ADC_IRQHandler(&barkour::Adc1Handle);
  }
  DBG_GPIO(false);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* adc_handle) {
  uint32_t raw_adc1 =
      HAL_ADCEx_InjectedGetValue(&barkour::Adc1Handle, ADC_INJECTED_RANK_1);
  uint32_t raw_adc2 =
      HAL_ADCEx_InjectedGetValue(&barkour::Adc2Handle, ADC_INJECTED_RANK_1);

  auto& adc = barkour::Adc::Get();
  barkour::PhaseSampleSelection phases = adc.GetSelectedPhases();

  if (adc.IsCalibrating()) {
    adc.DoCalibration(raw_adc1, raw_adc2);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adc.GetInjectedSampleSemaphore(),
                          &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  // Do current reconstruction for raw readings!!
  // This fakes out like we still use 3 injected samples,
  // one for each phase.
  int32_t offset1;
  int32_t offset2;

  adc.GetCalOffsets(offset1, offset2);  // get calibrated offsets

  int32_t motor_adc1 = (int32_t)raw_adc1 - offset1;
  int32_t motor_adc2 = (int32_t)raw_adc2 - offset2;
  int32_t motor_adc3 = -(motor_adc1 + motor_adc2);

  int32_t ia = 0;
  int32_t ib = 0;
  int32_t ic = 0;

  switch (phases) {
    case barkour::PhaseSampleSelection::kAB: {
      ia = motor_adc1;
      ib = motor_adc2;
      ic = motor_adc3;
      break;
    }
    case barkour::PhaseSampleSelection::kAC: {
      ia = motor_adc2;
      ib = motor_adc3;
      ic = motor_adc1;
      break;
    }
    case barkour::PhaseSampleSelection::kBC: {
      ia = motor_adc3;
      ib = motor_adc2;
      ic = motor_adc1;
      break;
    }
    default: {
      break;
    }
  }

  // swap b/c readings if phases are swapped
  if (barkour::foc_get_phase_order() == barkour::PhaseOrder::kAcb) {
    std::swap(ib, ic);
  }

  adc.motor_adc_samples_injected[0] = ia;
  adc.motor_adc_samples_injected[1] = ib;
  adc.motor_adc_samples_injected[2] = ic;

  // Run real time foc using latest ADC current readings.
  phases = barkour::foc_adc_isr(ia, ib, ic);

  adc.SetInjectedChannels(phases);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(adc.GetInjectedSampleSemaphore(),
                        &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA1_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(barkour::Adc1Handle.DMA_Handle);
}

void DMA1_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(barkour::Adc2Handle.DMA_Handle);
}

void DMA1_Stream2_IRQHandler(void) {
  HAL_DMA_IRQHandler(barkour::Adc3Handle.DMA_Handle);
}

#ifdef __cplusplus
}
#endif
