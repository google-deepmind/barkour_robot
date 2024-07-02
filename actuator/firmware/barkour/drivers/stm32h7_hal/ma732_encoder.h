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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MA732_ENCODER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MA732_ENCODER_H_

#include <optional>
#include <cstdint>

#include "pw_result/result.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

typedef struct Ma732Stats {
  uint32_t num_readings;        // Total number of calls to GetEncoderState.
  uint32_t parity_error_count;  // Number of parity errors experienced.
} Ma732Stats;

class Ma732Encoder : public RotaryEncoder {
 public:
  // Singleton access to Ma732Encoder.
  static Ma732Encoder& Get();

  // Returns the current encoder state.
  pw::Result<RotaryEncoderState> GetEncoderState() override;
  RotaryEncoderState& GetEncoderStateFromISR() override;

  pw::Result<RotaryEncoderState> Update() override;

  // Returns the number of encoder ticks per turn.
  uint32_t GetCountsPerTurn() const override;

  // Returns parity error statistics.
  Ma732Stats GetStatistics();

  // Resets the internal statistics structure.
  void ResetStatistics();

 private:
  Ma732Encoder();

  pw::Result<RotaryEncoderState> state_;

  pw::Result<uint16_t> ReadRegister(uint8_t address);

  pw::Status WriteRegister(uint8_t address, uint8_t value);

  Ma732Stats stats_;

  // Number of bytes to exchange over SPI.
  // must be 4, SPI device reads/writes data > 16 bits as a 32 bit value
  static inline constexpr int kMa732SpiBufferSize = 4;
  uint8_t spi_tx_buffer_[kMa732SpiBufferSize] = {};
  uint8_t spi_rx_buffer_[kMa732SpiBufferSize] = {};

  SPI_HandleTypeDef spi_handle_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_MA732_ENCODER_H_
