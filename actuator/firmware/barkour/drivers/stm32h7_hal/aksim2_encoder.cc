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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/aksim2_encoder.h"

#include <cstdint>
#include <cstring>

#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_status/status.h"
#include "FreeRTOS.h" // NOLINT
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

// 2^19 = 524288 (19 bits)
constexpr int32_t kAksIm2EncoderMaxCount = 524288;

constexpr size_t kDmaBufferSize = 4;
SPI_HandleTypeDef encoder_spi_handle;
CRC_HandleTypeDef crc_handle = {};
DMA_HandleTypeDef dma_tx_handle = {};
DMA_HandleTypeDef dma_rx_handle = {};
uint8_t spi_dma_rx_buffer[kDmaBufferSize] = {};
uint8_t spi_dma_tx_buffer[kDmaBufferSize] = {};

void AksIm2_HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  if (hspi->Instance == SPI3) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // SPI NSS GPIO pin configuration.
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // SPI SCK GPIO pin configuration.
    // GPIO speed set to low frequency to prevent ringing, see b/231599101.
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // SPI MISO GPIO pin configuration.
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // SPI MOSI GPIO pin configuration.
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure the DMA handler for Transmission process.
    dma_tx_handle.Instance = DMA2_Stream3;
    dma_tx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma_tx_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    dma_tx_handle.Init.MemBurst = DMA_MBURST_INC4;
    dma_tx_handle.Init.PeriphBurst = DMA_PBURST_INC4;
    dma_tx_handle.Init.Request = DMA_REQUEST_SPI3_TX;
    dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
    dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_tx_handle.Init.Mode = DMA_CIRCULAR;
    dma_tx_handle.Init.Priority = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&dma_tx_handle);
    __HAL_LINKDMA(hspi, hdmatx, dma_tx_handle);

    // Configure the DMA handler for Transmission process.
    dma_rx_handle.Instance = DMA2_Stream2;
    dma_rx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma_rx_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    dma_rx_handle.Init.MemBurst = DMA_MBURST_INC4;
    dma_rx_handle.Init.PeriphBurst = DMA_PBURST_INC4;
    dma_rx_handle.Init.Request = DMA_REQUEST_SPI3_RX;
    dma_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
    dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_rx_handle.Init.Mode = DMA_CIRCULAR;
    dma_rx_handle.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&dma_rx_handle);
    __HAL_LINKDMA(hspi, hdmarx, dma_rx_handle);

    // NVIC configuration for DMA transfer complete interrupt (SPI3_TX).
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

    // NVIC configuration for DMA transfer complete interrupt (SPI3_RX).
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    // NVIC configuration for SPI transfer complete interrupt (SPI3).
    HAL_NVIC_SetPriority(SPI3_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  } else {
    PW_CRASH("AksIm2_HAL_SPI_MspInit called with wrong DMA handle.");
  }
}

void AksIm2_HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI3) {
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();

    // SPI SCK Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);

    // SPI MISO Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

    // SPI MOSI Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_DMA_DeInit(&dma_tx_handle);
    HAL_DMA_DeInit(&dma_rx_handle);

    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);

    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  }
}

bool CheckCRC(const uint8_t (&buffer)[kDmaBufferSize]) {
  uint32_t* non_const_buffer_ptr =
      const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(&(buffer[0])));
  uint8_t crc_computed =
      HAL_CRC_Calculate(&crc_handle, non_const_buffer_ptr, kDmaBufferSize - 1);
  if ((crc_computed & buffer[kDmaBufferSize - 1]) != 0x00) {
    return false;
  }
  return true;
}

void RxBufferToReading(const uint8_t (&buffer)[kDmaBufferSize],
                       AksIm2Reading& reading) {
  reading.warning = (~buffer[2] & 0x1);
  reading.error = (~buffer[2] & 0x02);
  reading.encoder_position = ((uint32_t)buffer[2]) >> 2;
  reading.encoder_position |= ((uint32_t)buffer[1]) << 6;
  reading.encoder_position |= ((uint32_t)buffer[0]) << 14;
  // Field is 22 bits, resolution is 19 bits, so the padding is shifted out.
  reading.encoder_position = reading.encoder_position >> 3;
  reading.timestamp = HAL_GetTick();
  reading.crc_valid = CheckCRC(buffer);
  return;
}

void AksIm2Encoder::SpiTxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  AksIm2Encoder& encoder = Get();
  if (CheckCRC(spi_dma_rx_buffer)) {
    AksIm2Reading reading;
    RxBufferToReading(spi_dma_rx_buffer, reading);
    if (reading.encoder_position < kAksIm2EncoderMaxCount) {
      encoder.latest_reading_ = reading;
    }
    encoder.have_valid_reading_ = true;
  }
}

AksIm2Encoder::AksIm2Encoder() {
  stats_ = {0};
  latest_reading_ = {0};
  dma_started_ = false;
  have_valid_reading_ = false;
  SetupHardware();
}

AksIm2Encoder& AksIm2Encoder::Get() {
  static AksIm2Encoder encoder;
  return encoder;
}

pw::Result<AksIm2Reading> AksIm2Encoder::GetLatestReading() const {
  if (have_valid_reading_) {
    return latest_reading_;
  }
  return pw::Result<AksIm2Reading>(pw::Status::NotFound());
}

void AksIm2Encoder::SetupHardware() {
  HAL_StatusTypeDef result;
  PW_LOG_INFO("Setting up AksIm2 Encoder.");

  // Initialize CRC.
  crc_handle.Instance = CRC;
  crc_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  crc_handle.Init.GeneratingPolynomial = 0x97;
  crc_handle.Init.CRCLength = CRC_POLYLENGTH_8B;
  crc_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  crc_handle.Init.InitValue = 0;
  crc_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  crc_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  crc_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  result = HAL_CRC_Init(&crc_handle);
  PW_CHECK(HAL_OK == result);

  // Initialize SPI.
  std::memset(&encoder_spi_handle, 0, sizeof(encoder_spi_handle));
  encoder_spi_handle.Instance = SPI3;
  // With PLL3M = 32, PLL3N = 120, PLL3P = 50, and lastly
  // SPI_BAUDRATEPRESCALER_2 SCLK will run at 2.4Mhz.
  encoder_spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  encoder_spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
  encoder_spi_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
  encoder_spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  encoder_spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;
  encoder_spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  encoder_spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
  encoder_spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  encoder_spi_handle.Init.CRCPolynomial = 7;
  encoder_spi_handle.Init.CRCLength = SPI_CRC_LENGTH_8BIT;
  encoder_spi_handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
  encoder_spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  encoder_spi_handle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  encoder_spi_handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  // Even through the AksIM2 encoder can run with up to 4Mhz clock rate, there
  // must be a 5uS setup time between NSS going low and the rising edge of the
  // SCK.
  encoder_spi_handle.Init.MasterSSIdleness = 13;
  encoder_spi_handle.Init.Mode = SPI_MODE_MASTER;

  result = HAL_SPI_RegisterCallback(&encoder_spi_handle, HAL_SPI_MSPINIT_CB_ID,
                                    AksIm2_HAL_SPI_MspInit);
  PW_CHECK(HAL_OK == result);

  result = HAL_SPI_RegisterCallback(
      &encoder_spi_handle, HAL_SPI_MSPDEINIT_CB_ID, AksIm2_HAL_SPI_MspDeInit);
  PW_CHECK(HAL_OK == result);

  result = HAL_SPI_Init(&encoder_spi_handle);
  PW_CHECK(HAL_OK == result);

  result = HAL_SPI_RegisterCallback(
      &encoder_spi_handle, HAL_SPI_TX_RX_COMPLETE_CB_ID, SpiTxRxCpltCallback);
  PW_CHECK(HAL_OK == result);
}

pw::Result<AksIm2Reading> AksIm2Encoder::GetReading() {
  memset(spi_dma_rx_buffer, 0, kDmaBufferSize);
  memset(spi_dma_tx_buffer, 0, kDmaBufferSize);
  HAL_StatusTypeDef result;
  result = HAL_SPI_TransmitReceive(
      &encoder_spi_handle, (uint8_t*)spi_dma_tx_buffer,
      (uint8_t*)spi_dma_rx_buffer, kDmaBufferSize, 1000);
  stats_.readings_counter++;
  if (HAL_OK != result) {
    return pw::Result<AksIm2Reading>(pw::Status::Internal());
  }

  AksIm2Reading reading;
  RxBufferToReading(spi_dma_rx_buffer, reading);

  if (!reading.crc_valid) {
    stats_.crc_error_counter++;
  }

  if (reading.error) {
    stats_.error_counter++;
  }

  if (reading.warning) {
    stats_.warning_counter++;
  }

  if (reading.encoder_position > kAksIm2EncoderMaxCount) {
    return pw::Result<AksIm2Reading>(pw::Status::OutOfRange());
  }
  taskENTER_CRITICAL();
  latest_reading_ = reading;
  taskEXIT_CRITICAL();

  have_valid_reading_ = true;
  return pw::Result<AksIm2Reading>(reading);
}

const AksIm2Stats& AksIm2Encoder::GetStatistics() { return stats_; }

void AksIm2Encoder::ResetStatistics() {
  stats_.readings_counter = 0;
  stats_.crc_error_counter = 0;
  stats_.error_counter = 0;
  stats_.warning_counter = 0;
}

pw::Status AksIm2Encoder::StartDma() {
  if (dma_started_) {
    return pw::Status::Internal();
  }
  memset(spi_dma_rx_buffer, 0, kDmaBufferSize);
  memset(spi_dma_tx_buffer, 0, kDmaBufferSize);
  HAL_StatusTypeDef result;
  result = HAL_SPI_TransmitReceive_DMA(&encoder_spi_handle,
                                       (uint8_t*)spi_dma_tx_buffer,
                                       (uint8_t*)spi_dma_rx_buffer,
                                       kDmaBufferSize);
  if (HAL_OK != result) {
    pw::Result<AksIm2Reading>(pw::Status::Internal());
  }
  dma_started_ = true;
  return pw::OkStatus();
}

pw::Status AksIm2Encoder::StopDma() {
  if (!dma_started_) {
    return pw::Status::Internal();
  }
  HAL_StatusTypeDef result;
  result = HAL_SPI_DMAStop(&encoder_spi_handle);
  if (HAL_OK != result) {
    pw::Result<AksIm2Reading>(pw::Status::Internal());
  }
  dma_started_ = false;
  return pw::OkStatus();
}

// Returns the current encoder data.
pw::Result<RotaryEncoderState> AksIm2Encoder::GetEncoderState() {
  pw::Result<RotaryEncoderState> state;

  portENTER_CRITICAL();
  state = state_;
  portEXIT_CRITICAL();

  return state;
}

RotaryEncoderState& AksIm2Encoder::GetEncoderStateFromISR() {
  return state_.value();
}

pw::Result<RotaryEncoderState> AksIm2Encoder::Update() {
  pw::Result<AksIm2Reading> state = GetReading();
  if (!state.ok())
    state_ = state.status();
  else
    state_ = RotaryEncoderState{.raw_encoder_counts =
                                    state.value().encoder_position};

  return state_;
}

// Returns the number of encoder ticks per turn.
uint32_t AksIm2Encoder::GetCountsPerTurn() const {
  return kAksIm2EncoderMaxCount;
}

}  // namespace barkour

extern "C" {
void DMA2_Stream2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&barkour::dma_tx_handle);
}

void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&barkour::dma_rx_handle);
}

void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc) { __HAL_RCC_CRC_CLK_ENABLE(); }
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc) {
  __HAL_RCC_CRC_FORCE_RESET();
  __HAL_RCC_CRC_RELEASE_RESET();
}
}
