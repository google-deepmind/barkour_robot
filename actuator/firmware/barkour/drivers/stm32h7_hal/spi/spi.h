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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_SPI_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_SPI_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"
#include "pw_status/status.h"

namespace barkour {

// SPI driver.
class Spi : public SpiInterface {
 public:
  Spi() = default;

  pw::Status Initialize(SpiReference spi_reference,
                        void (*spi_callback)()) override;

  pw::Status Deinitialize() override;

  pw::Status ExchangeData(uint8_t *data_rx, uint8_t* data_tx,
                          uint8_t data_length) override;
 private:
  bool initialised_;
  SpiReference spi_reference_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_SPI_H_
