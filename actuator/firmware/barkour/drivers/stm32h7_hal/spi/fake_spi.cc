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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/spi/fake_spi.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/spi/spi.h"
#include "pw_status/status.h"

namespace barkour {

constexpr uint8_t kIncomingPayloadMaxLength = 20;
constexpr uint8_t kSpiNumberOfChannels = 4;

struct fake_spi_data {
  bool interrupt_failure_on;
  uint8_t next_incoming_payload[kIncomingPayloadMaxLength];
  void (*fake_spi_callback)();
};

static fake_spi_data spi_channels[kSpiNumberOfChannels];

void FakeSpi::set_next_incoming_payload(uint8_t* data_to_overwrite,
                                        uint8_t data_length,
                                        SpiReference spi_ref) {
  memcpy(spi_channels[static_cast<uint8_t>(spi_ref)].next_incoming_payload,
         data_to_overwrite, data_length);
}

void FakeSpi::set_interrupt_failure(SpiReference spi_ref) {
  spi_channels[static_cast<uint8_t>(spi_ref)].interrupt_failure_on = true;
}

void FakeSpi::reset_spi_data(SpiReference spi_ref) {
  memset(&spi_channels[static_cast<uint8_t>(spi_ref)], 0,
         sizeof(fake_spi_data));
}

pw::Status Spi::Initialize(SpiReference spi_reference,
                              void (*spi_callback)()) {
  if (spi_reference_ == SpiReference::kUninitialized ||
      spi_callback == nullptr) {
    return pw::Status::InvalidArgument();
  }

  spi_reference_ = spi_reference;
  initialised_ = true;
  spi_channels[static_cast<uint8_t>(spi_reference)].
      fake_spi_callback = spi_callback;
  spi_channels[static_cast<uint8_t>(spi_reference)]
      .interrupt_failure_on = false;
  memset(spi_channels[static_cast<uint8_t>(spi_reference)]
         .next_incoming_payload, 0, sizeof(uint8_t)*kIncomingPayloadMaxLength);
  return pw::OkStatus();
}

pw::Status Spi::Deinitialize() {
  memset(&spi_channels[static_cast<uint8_t>(spi_reference_)], 0,
         sizeof(fake_spi_data));
  initialised_ = false;
  spi_reference_ = SpiReference::kUninitialized;
  return pw::OkStatus();
}

pw::Status Spi::ExchangeData(uint8_t *data_rx, uint8_t* data_tx,
                          uint8_t data_length) {
  (void)data_tx;
  // Copy data from fake incoming packet into data_rx given data_length size.
  memcpy(data_rx, spi_channels[static_cast<uint8_t>(spi_reference_)]
         .next_incoming_payload, data_length);
  // Invoke a registered callback if no interrupt failure is expected.
  if (!spi_channels[static_cast<uint8_t>(spi_reference_)]
      .interrupt_failure_on) {
    spi_channels[static_cast<uint8_t>(spi_reference_)].fake_spi_callback();
  }
  return pw::OkStatus();
}

}  // namespace barkour
