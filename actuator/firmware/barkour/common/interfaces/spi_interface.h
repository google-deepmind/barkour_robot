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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_SPI_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_SPI_INTERFACE_H_

#include <cstdint>

#include "pw_status/status.h"

namespace barkour {

// Reference to SPI instance.
enum class SpiReference : uint8_t {
  kSpi1 = 0,
  kSpi2 = 1,
  kSpi3 = 2,
  kSpi4 = 3,
  kUninitialized = 4,
};

// Provides access to SPI.
class SpiInterface {
 public:
  virtual ~SpiInterface() = default;

  // Initializes the SPI instance. It should initialize all peripherals.
  //
  // Args:
  // - spi_reference: reference to SPI instance to be initialized.
  // - spi_callback: callback to be invoked when SPI interrupts.
  //
  // Returns:
  // - Status of the initialization.
  virtual pw::Status Initialize(SpiReference spi_reference,
                               void (*spi_callback)()) = 0;

  // Exchanges data over previously initialized SPI instance.
  //
  // Args:
  // - spi_reference: reference to SPI instance to be initialized.
  // - spi_callback: callback to be invoked when SPI interrupts.
  //
  // Returns:
  // - Status of data exchange.
  virtual pw::Status ExchangeData(uint8_t *data_rx, uint8_t* data_tx,
                                  uint8_t data_length) = 0;

  // Deinitializes the SPI instance. It should deinitialize all peripherals.
  //
  // Returns:
  // - Status of deinitialization.
  virtual pw::Status Deinitialize() = 0;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_SPI_INTERFACE_H_
