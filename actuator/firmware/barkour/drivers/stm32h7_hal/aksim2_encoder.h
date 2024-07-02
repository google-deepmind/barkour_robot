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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_AKSIM2_ENCODER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_AKSIM2_ENCODER_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "pw_result/result.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

typedef struct AksIm2Reading {
  // Warning field, see Renishaw MBD01 for more details.
  bool warning = false;

  // Error field, see Renishaw MBD01 for more details.
  bool error = false;

  // Position, represented in counts.
  // This value will range between 0 and 524288.
  uint32_t encoder_position = 0;

  // True if the encoder reading passed the CRC check.
  bool crc_valid = false;

  // A rough timestamp, represented in milliseconds.
  // This field is populated by HAL_GetTick().
  uint32_t timestamp = 0;
} AksIm2Reading;

typedef struct AksIm2Stats {
  uint32_t readings_counter = 0;
  uint32_t crc_error_counter = 0;
  uint32_t warning_counter = 0;
  uint32_t error_counter = 0;
} AksIm2Stats;

class AksIm2Encoder : public RotaryEncoder {
 public:
  // Singleton access to AksIm2Encoder.
  static AksIm2Encoder& Get();

  // Returns the most recent encoder reading.
  //
  // If the encoder has not yet been read, pw::Status::NotFound() will
  // be returned.
  pw::Result<AksIm2Reading> GetLatestReading() const;

  // Performs a blocking read of the encoder.
  //
  // In performing a blocking read of the encoder, this function will also
  // update the latest reading returned by GetLatestReading.
  //
  // In the case of a CRC error pw::Status::DataLoss() will be returned.
  //
  // Note: error flags, warning flags and failed CRC checks will not cause this
  //   this function to fail. It is the responsibility of the caller to check
  //   these fields in the resulting AksIm2Reading.
  //
  // Returns pw::Status::OutOfRange() if the encoder_position is not a valid
  // position.
  pw::Result<AksIm2Reading> GetReading();

  // Returns the Warning and Error Flag statistics.
  // These values will be updated each time the encoder is read.
  const AksIm2Stats& GetStatistics();

  // Resets the flag statistics back to zero.
  void ResetStatistics();

  // Starts DMA transfers.
  //
  // When DMA transfers are enabled, the encoder will be continually read.
  // Upon completion of each reading, latest_reading_ will be updated.
  // Returns pw::Status::Internal() if DMA transfer was already started.
  pw::Status StartDma();

  // Stops DMA transfers.
  //
  // Returns pw::Status::Internal() if DMA has not yet been started.
  pw::Status StopDma();

  // RotaryEncoder Interface

  // Returns the current encoder data.
  pw::Result<RotaryEncoderState> GetEncoderState() override;
  RotaryEncoderState& GetEncoderStateFromISR() override;
  pw::Result<RotaryEncoderState> Update() override;

  // Returns the number of encoder ticks per turn.
  uint32_t GetCountsPerTurn() const override;

 private:
  static void SpiTxRxCpltCallback(SPI_HandleTypeDef* hspi);

  AksIm2Stats stats_;
  AksIm2Reading latest_reading_;
  bool dma_started_;
  bool have_valid_reading_;

  pw::Result<RotaryEncoderState> state_;

  AksIm2Encoder();
  // Configures the hardware and starts sampling.
  void SetupHardware();
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_AKSIM2_ENCODER_H_
