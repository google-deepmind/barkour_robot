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

// ESC hardware layer functions for TMC8462 attached via SPI.
//
// Function to read and write commands to the ESC. Used to read/write ESC
// registers and memory.

#include <sys/param.h>
#include <cstring>

#include "actuator/firmware/barkour/m4/ecat_device.h"
#include "actuator/firmware/barkour/m4/ecat/ecat_options.h"

#define ESC_NOP 0
#define ESC_READ 2
#define ESC_READ_WITH_WAIT_STATE 3
#define ESC_WRITE 4
#define ESC_ADDRESS_EXTENSION 6

#define SPI_HEADER 4
uint8_t tx_buf[MAX(MBXSIZE, MBXSIZEBOOT) + SPI_HEADER];
uint8_t rx_buf[MAX(MBXSIZE, MBXSIZEBOOT) + SPI_HEADER];

extern "C" {

#include "soes/esc.h"

// 3-byte addressing mode, esp for >0x1FFF
static void ESC_command3(uint8_t* buf, uint8_t command, uint16_t address) {
  // 5.1.1 SPI protocol description
  //
  // Byte0: A12 A11 A10 A09 A08 A07 A06 A05
  // Byte1: A04 A03 A02 A01 A00   1   1   0
  // Byte2: A15 A14 A13  C2  C1  C0   0   0
  // A: 16 bit address
  // C: command
  *buf++ = address >> 5;
  *buf++ = (address << 3) + ESC_ADDRESS_EXTENSION;
  *buf++ = (address >> 8 & 0xE0) + (command << 2);
}

// ESC read function used by the device stack.
//
// @param[in]   address     = address of ESC register to read
// @param[out]  buf         = pointer to buffer to read in
// @param[in]   len         = number of bytes to read
void ESC_read(uint16_t address, void* buf, uint16_t len) {
  ESC_command3(tx_buf, ESC_READ_WITH_WAIT_STATE, address);
  tx_buf[3] = 0xFF;

  memset(tx_buf + 4, 0, len);
  tx_buf[len + 3] = 0xFF;

  barkour::EthercatDevice::Get().exchange(tx_buf, rx_buf, len + 4);
  memcpy(buf, rx_buf + 4, len);

  // ALevent register 0x220 is received in header bytes for free
  ESCvar.ALevent = etohs(*(uint16_t*)rx_buf);
}

// ESC write function used by the device stack.
//
// @param[in]   address     = address of ESC register to write
// @param[out]  buf         = pointer to buffer to write from
// @param[in]   len         = number of bytes to write
void ESC_write(uint16_t address, void* buf, uint16_t len) {
  ESC_command3(tx_buf, ESC_WRITE, address);
  memcpy(tx_buf + 3, buf, len);

  barkour::EthercatDevice::Get().exchange(tx_buf, rx_buf, len + 3);

  ESCvar.ALevent = etohs(*(uint16_t*)rx_buf);
}

// Initialize SPI communication with TMC.
//
void ESC_init(const esc_cfg_t* config) {}

}  // extern "C"
