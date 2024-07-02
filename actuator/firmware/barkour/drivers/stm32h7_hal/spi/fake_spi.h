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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_FAKE_SPI_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_FAKE_SPI_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"

namespace barkour {

// Fake SPI implementation.
class FakeSpi {
 public:
  static void set_next_incoming_payload(uint8_t* data_to_overwrite,
                                        uint8_t data_length,
                                        SpiReference spi_ref);

  static void set_interrupt_failure(SpiReference spi_ref);

  static void reset_spi_data(SpiReference spi_ref);
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_SPI_FAKE_SPI_H_
