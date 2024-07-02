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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/ma732_encoder.h"

#include <bitset>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h" // NOLINT
#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_status/status.h"
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

namespace {

constexpr bool kUseParity = true;
constexpr uint16_t kTimeoutDurationTicks = 1000;

constexpr uint32_t kMa732EncoderMaxCount = 65536;
// Chip internal filter for encoder readings.
// Some useful values below.
// See p. 22 in MA732 data sheet for all values.
// FW = FilterWindow, TC = Time Constant, FC = Cutoff Freq.,
// Res. = Effective Resolution.
// FW  | TC    | Res.    | FC
// 51  | 64us  | 9.5bit  | 6kHz
// 68  | 128us | 10bit   | 3kHz
// 85  | 256us | 10.5bit | 1.5kHz
// 102 | 512us | 11bit   | 740Hz
// 119 | 102us | 11.5bit | 370Hz (chip default)
constexpr uint8_t kMa732FilterWindow = 51;

constexpr uint8_t kFilterWindowRegisterAddress = 0x0E;
constexpr uint8_t kBCTRegisterAddress = 0x02;
constexpr uint8_t kETXYRegisterAddress = 0x03;

// Idle time required between a write command and a register readout.
// (delay necessary for non-volatile memory updates).
constexpr uint8_t kRegisterReadoutDelayMs = 20;

/// read/write commands to Ma732
constexpr uint8_t kReadRegCommand = 0x1 << 6;
constexpr uint8_t kWriteRegCommand = 0x1 << 7;
constexpr uint8_t kReadAngleCommand = 0x00;
constexpr uint8_t kAddrMask = 0x1F;

// Checks if the parity bit is set correctly for the encoder reading.
//
// The parity bit acts a 1-bit CRC, and can be used to determine if a bit-error
// has occurred for an odd number of bits.
//
// The Ma732 uses even parity, meaning that the parity bit is set to ensure that
// the number of set bits is even.
//
// Returns true if the reading passes the parity check, false otherwise.
bool ReadingPassesParityCheck(uint16_t reading, uint16_t parity_bit) {
  uint16_t p = reading ^ (reading >> 1);
  p = p ^ (p >> 2);
  p = p ^ (p >> 4);
  p = p ^ (p >> 8);

  // bit 0 of p now = 1 if parity of reading is odd, 0 if parity is even.
  return (p & 1) ==
         parity_bit;  // p == party_bit if 17 bit value parity was even.
}

void Ma732Encoder_HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  if (hspi->Instance == SPI3) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
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

    // NVIC configuration for SPI transfer complete interrupt (SPI3).
    HAL_NVIC_SetPriority(SPI3_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  } else {
    PW_CRASH("Ma732Encoder_HAL_SPI_MspInit called with wrong SPI handle.");
  }
}

void Ma732Encoder_HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI3) {
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();

    // SPI SCK Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);

    // SPI MISO Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

    // SPI MOSI Pin.
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  }
}

}  // namespace

Ma732Encoder::Ma732Encoder() : stats_({}) {
  HAL_StatusTypeDef result;

  // Initialize SPI.
  std::memset(&spi_handle_, 0, sizeof(spi_handle_));
  spi_handle_.Instance = SPI3;
  // With PLL3M = 32, PLL3N = 120, PLL3P = 10, and lastly
  // SPI_BAUDRATEPRESCALER_2 SCLK will run at 12Mhz.
  spi_handle_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi_handle_.Init.Direction = SPI_DIRECTION_2LINES;
  spi_handle_.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi_handle_.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi_handle_.Init.DataSize = SPI_DATASIZE_8BIT;
  spi_handle_.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_handle_.Init.TIMode = SPI_TIMODE_DISABLE;

  spi_handle_.Init.NSS = SPI_NSS_HARD_OUTPUT;
  spi_handle_.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  spi_handle_.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  spi_handle_.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  spi_handle_.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  spi_handle_.Init.Mode = SPI_MODE_MASTER;

  result = HAL_SPI_RegisterCallback(&spi_handle_, HAL_SPI_MSPINIT_CB_ID,
                                    Ma732Encoder_HAL_SPI_MspInit);
  PW_CHECK(HAL_OK == result);

  result = HAL_SPI_RegisterCallback(&spi_handle_, HAL_SPI_MSPDEINIT_CB_ID,
                                    Ma732Encoder_HAL_SPI_MspDeInit);
  PW_CHECK(HAL_OK == result);

  PW_CHECK(HAL_SPI_Init(&spi_handle_) == HAL_OK);

  vTaskDelay(pdMS_TO_TICKS(100));

  (void)ReadRegister(kFilterWindowRegisterAddress);
  (void)ReadRegister(kFilterWindowRegisterAddress);

  // Read filter window register.
  pw::Result<uint8_t> maybe_filter_window =
      ReadRegister(kFilterWindowRegisterAddress);
  PW_CHECK_OK(maybe_filter_window.status());

  // Only write the filter window register if it needs to be written, as the
  // device flash can only be written a limited number of time.
  if (maybe_filter_window.value() != kMa732FilterWindow) {
    PW_LOG_INFO("Updating Ma732 filter window to %u, was %u.",
                kMa732FilterWindow, maybe_filter_window.value());
    PW_CHECK_OK(
        WriteRegister(kFilterWindowRegisterAddress, kMa732FilterWindow));
  }

  // If taking advantage of the Ma732's parity bit, which is not documented in
  // the datasheet, the SPI peripheral will be re-initialized using 17-bit data
  // size. The 17th bit (the bit following the LSB) will be the parity bit.
  //
  // Details on the parity bit support for the Ma732 can be found here:
  //   https://www.monolithicpower.com/media/document/AN137_Using_the_MagAlpha_Serial_Interface_Advanced_Features.pdf
  //
  // While it is possible to continue using 8-bit data size, to do so would
  // increase encoder reading latency significantly, as the final transaction
  // contain an additional 7 bits.
  if (kUseParity) {
    result = HAL_SPI_DeInit(&spi_handle_);
    PW_CHECK(HAL_OK == result);

    spi_handle_.Init.DataSize = SPI_DATASIZE_17BIT;
    result = HAL_SPI_Init(&spi_handle_);
    PW_CHECK(HAL_OK == result);
  }
  PW_LOG_INFO("[ENC-MA732] Init Ok");
}

// Reads a device register.
//
// Returns:
// - uint16_t: Value read from the register.
// - pw::Status::Ok(): Register was read successfully.
// - pw::Status::Internal(): An error occurred.
pw::Result<uint16_t> Ma732Encoder::ReadRegister(uint8_t address) {
  HAL_StatusTypeDef result;
  memset(spi_tx_buffer_, 0, kMa732SpiBufferSize);
  memset(spi_rx_buffer_, 0, kMa732SpiBufferSize);

  spi_tx_buffer_[0] = (kReadRegCommand | (kAddrMask & address));

  result = HAL_SPI_TransmitReceive(&spi_handle_, (uint8_t*)spi_tx_buffer_,
                                   (uint8_t*)spi_rx_buffer_, 2,
                                   kTimeoutDurationTicks);
  if (HAL_OK != result) {
    return pw::Status::Internal();
  }

  vTaskDelay(pdMS_TO_TICKS(kRegisterReadoutDelayMs));
  memset(spi_tx_buffer_, 0, kMa732SpiBufferSize);

  result = HAL_SPI_TransmitReceive(&spi_handle_, (uint8_t*)spi_tx_buffer_,
                                   (uint8_t*)spi_rx_buffer_, 2,
                                   kTimeoutDurationTicks);
  if (HAL_OK != result) {
    return pw::Status::Internal();
  }

  if (spi_rx_buffer_[1] != 0x00) {
    PW_LOG_ERROR(
        "MA732 register read might have failed. Did not receive all zero "
        "second byte after writing. Received: %u",
        spi_rx_buffer_[1]);
    return pw::Status::Internal();
  }
  return spi_rx_buffer_[0];
}

// Writes a device register.
//
// Returns:
// - Ok: Register was read successfully.
// - Internal: An error occurred.
pw::Status Ma732Encoder::WriteRegister(uint8_t address, uint8_t value) {
  HAL_StatusTypeDef result;
  memset(spi_tx_buffer_, 0, kMa732SpiBufferSize);
  memset(spi_rx_buffer_, 0, kMa732SpiBufferSize);

  spi_tx_buffer_[0] = (kWriteRegCommand | (kAddrMask & address));
  spi_tx_buffer_[1] = value;

  result = HAL_SPI_TransmitReceive(&spi_handle_, (uint8_t*)spi_tx_buffer_,
                                   (uint8_t*)spi_rx_buffer_, 2,
                                   kTimeoutDurationTicks);
  if (HAL_OK != result) {
    return pw::Status::Internal();
  }

  memset(spi_tx_buffer_, 0, kMa732SpiBufferSize);
  vTaskDelay(pdMS_TO_TICKS(kRegisterReadoutDelayMs));

  result = HAL_SPI_TransmitReceive(&spi_handle_, (uint8_t*)spi_tx_buffer_,
                                   (uint8_t*)spi_rx_buffer_, 2,
                                   kTimeoutDurationTicks);
  if (HAL_OK != result) {
    return pw::Status::Internal();
  }

  if (spi_rx_buffer_[0] != value) {
    PW_LOG_ERROR("MA732 register write failed. Wrote %u, but read %u", value,
                 spi_rx_buffer_[0]);
    return pw::Status::Internal();
  }
  if (spi_rx_buffer_[1] != 0x00) {
    PW_LOG_ERROR(
        "MA732 register write might have failed. Did not receive all zero "
        "second byte after writing. Received: %u",
        spi_rx_buffer_[1]);
    return pw::Status::Internal();
  }

  return pw::OkStatus();
}

Ma732Encoder& Ma732Encoder::Get() {
  static Ma732Encoder encoder;
  return encoder;
}

RotaryEncoderState& Ma732Encoder::GetEncoderStateFromISR() {
  return state_.value();
}

pw::Result<RotaryEncoderState> Ma732Encoder::GetEncoderState() {
  pw::Result<RotaryEncoderState> state;

  portENTER_CRITICAL();

  state = state_;

  portEXIT_CRITICAL();

  return state;
}

pw::Result<RotaryEncoderState> Ma732Encoder::Update(void) {
  memset(spi_rx_buffer_, 0, kMa732SpiBufferSize);
  memset(spi_tx_buffer_, 0, kMa732SpiBufferSize);
  HAL_StatusTypeDef result;

  uint16_t transfer_size = 2;
  if (kUseParity) {
    transfer_size = 1;
  }

  stats_.num_readings++;
  result =
      HAL_SPI_TransmitReceive(&spi_handle_, (uint8_t*)spi_tx_buffer_,
                              (uint8_t*)spi_rx_buffer_, transfer_size, 1000);
  if (HAL_OK != result) {
    state_ = pw::Status::Internal();
    return state_;
  }

  RotaryEncoderState reading;

  if (kUseParity) {
    // When the SPI peripheral is configured for 17-bit transfers, the value is
    // read into spi_rx_buffer_ as follows (and can be read as a 32 bit value):
    //   MSB                                                       LSB
    //   | spi_rx_buffer_[2] | spi_rx_buffer_[1] | spi_rx_buffer_[0] |
    //   |  - - - - - - - R  |  R R R R R R R R  |  R R R R R R R P  |
    //
    //  Where R are the encoder reading bits, and P is the parity bit, and - are
    uint32_t reading_raw = *reinterpret_cast<uint32_t*>(spi_rx_buffer_);
    uint16_t parity_bit = reading_raw & 1;
    reading.raw_encoder_counts = reading_raw >> 1;

    if (!ReadingPassesParityCheck(reading.raw_encoder_counts, parity_bit)) {
      stats_.parity_error_count++;
      state_ = pw::Status::DataLoss();
      return state_;
    }
  } else {
    reading.raw_encoder_counts = spi_rx_buffer_[1];
    reading.raw_encoder_counts |= spi_rx_buffer_[0] << 8;
  }

  if (reading.raw_encoder_counts > kMa732EncoderMaxCount) {
    state_ = pw::Status::OutOfRange();
    return state_;
  }

  state_ = reading;
  return state_;
}

// Returns the number of encoder ticks per turn.
uint32_t Ma732Encoder::GetCountsPerTurn() const {
  return kMa732EncoderMaxCount;
}

// Returns parity error statistics.
Ma732Stats Ma732Encoder::GetStatistics() { return stats_; }

// Resets the internal statistics structure.
void Ma732Encoder::ResetStatistics() { stats_ = {}; }

}  // namespace barkour
