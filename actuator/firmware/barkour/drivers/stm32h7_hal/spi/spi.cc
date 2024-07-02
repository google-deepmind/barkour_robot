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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/spi/spi.h"

#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"
#include "pw_status/status.h"
#include "pw_log/log.h"
#include "stm32h7xx_hal.h" // NOLINT
#include "FreeRTOS.h" // NOLINT

namespace barkour {

struct SpiConfig {
  SPI_HandleTypeDef spi_config_handle;
  void (*spi_callback_handler)(void);
};

static SpiConfig spi4_config = {
  .spi_config_handle = {
    .Instance = SPI4,
    .Init = {
      .Mode = SPI_MODE_MASTER,
      .Direction = SPI_DIRECTION_2LINES,
      .DataSize = SPI_DATASIZE_8BIT,
      .CLKPolarity = SPI_POLARITY_HIGH,
      .CLKPhase = SPI_PHASE_2EDGE,
      .NSS = SPI_NSS_HARD_OUTPUT,
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .NSSPMode = SPI_NSS_PULSE_DISABLE,
      .NSSPolarity = SPI_NSS_POLARITY_LOW,
    },
  },
  .spi_callback_handler = nullptr,
};

static void spi4_complete_callback(SPI_HandleTypeDef* spi_handle) {
  (void)spi_handle;
  if (spi4_config.spi_callback_handler != nullptr) {
    spi4_config.spi_callback_handler();
  }
}

pw::Status Spi::Initialize(SpiReference spi_reference,
                        void (*spi_callback)()) {
  if (spi_callback == nullptr) {
    PW_LOG_ERROR("[SPI] Uninitialised callback");
    return pw::Status::Internal();
  }
  if (spi_reference == SpiReference::kUninitialized) {
    PW_LOG_ERROR("[SPI] Uninitialised SPI reference");
    return pw::Status::Internal();
  }

  switch (spi_reference) {
    case SpiReference::kSpi4: {
      // SPI4 Setup.
      __HAL_RCC_SPI4_CLK_ENABLE();
      // Initialise SPI.
      if (HAL_SPI_Init(&spi4_config.spi_config_handle) != HAL_OK) {
        PW_LOG_ERROR("[SPI4] Configuration failed");
        return pw::Status::Internal();
      }

      // Register callback if assigned.
      spi4_config.spi_callback_handler = spi_callback;
      if (HAL_SPI_RegisterCallback(&spi4_config.spi_config_handle,
                                   HAL_SPI_TX_RX_COMPLETE_CB_ID,
                                   spi4_complete_callback) != HAL_OK) {
        PW_LOG_ERROR("[SPI4] Registering TxRx Callback failed");
      }

      // NVIC configuration for SPI transfer complete interrupt (SPI4).
      HAL_NVIC_SetPriority(SPI4_IRQn,
                           configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
      HAL_NVIC_EnableIRQ(SPI4_IRQn);
      initialised_ = true;
      break;
    }
    case SpiReference::kSpi1:
    case SpiReference::kSpi2:
    case SpiReference::kSpi3:
    default: {
      PW_LOG_ERROR("[SPI] Unimplemented SPI reference");
      return pw::Status::Internal();
      break;
    }
  }
  spi_reference_ = spi_reference;
  return pw::OkStatus();
}

pw::Status Spi::Deinitialize() {
  if (initialised_ && spi_reference_ != SpiReference::kUninitialized) {
    switch (spi_reference_) {
      case SpiReference::kSpi4: {
        // Disable SPI4 and deinitialize all structures.
        HAL_NVIC_DisableIRQ(SPI4_IRQn);
        HAL_SPI_UnRegisterCallback(&spi4_config.spi_config_handle,
                                   HAL_SPI_TX_RX_COMPLETE_CB_ID);
        HAL_SPI_DeInit(&spi4_config.spi_config_handle);
        __HAL_RCC_SPI4_CLK_DISABLE();
        break;
      }
      case SpiReference::kSpi1:
      case SpiReference::kSpi2:
      case SpiReference::kSpi3:
      default: {
        PW_LOG_ERROR("[SPI] Unimplemented SPI reference");
        return pw::Status::Internal();
      }
    }
  } else {
    PW_LOG_WARN("[SPI] SPI is not initialised");
    return pw::Status::Internal();
  }
  initialised_ = false;
  spi_reference_ = SpiReference::kUninitialized;
  return pw::OkStatus();
}

pw::Status Spi::ExchangeData(uint8_t *data_rx, uint8_t* data_tx,
                          uint8_t data_length) {
  switch (spi_reference_) {
    case SpiReference::kSpi4: {
      if (HAL_SPI_TransmitReceive_IT(&spi4_config.spi_config_handle, data_tx,
                                     data_rx, data_length) != HAL_OK) {
        return pw::Status::Unavailable();
      }
      break;
    }
    case SpiReference::kSpi1:
    case SpiReference::kSpi2:
    case SpiReference::kSpi3:
    default: {
      PW_LOG_ERROR("[SPI] Unimplemented SPI reference");
      return pw::Status::Internal();
    }
  }
  return pw::OkStatus();
}

extern "C" {
void SPI4_IRQHandler(void) {
  HAL_SPI_IRQHandler(&spi4_config.spi_config_handle);
}
}

}  // namespace barkour
