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

// Protocol definitions for LSM9DS1 IMU.
#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_PROTOCOL_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_PROTOCOL_H_

// Lsm9Ds1Imu read/write transaction:
// MSB                                             LSB
// |    spi_tx_buffer[1]         | spi_tx_buffer[0]  |
// | Read/Write  | Address bits  |   Data in bits    |
// |           R | A A A A A A A |   I I I I I I I I |
//
// MSB                                             LSB
// |    spi_rx_buffer[0]         | spi_rx_buffer[1]  |
// |      unused bits            |     Data out bits |
// |             0 0 0 0 0 0 0 0 |   D D D D D D D D |
// R: Read/Write bit (1: read, 0: write).
// A: Address bits.
// I: Data input bits (data to write).
// D: Data output bits (data which was read).
//
// Note: It is possible to read multiple consecutive registers by extending
// the read transaction. See the LSM9DS1 datasheet for more detail.

#include <cstdint>

namespace barkour {

// Defines IMU memory registers by their 8-bit addresses.
// The full register map is available in LSM9DS1 datasheet.
enum class ImuRegisterType : uint8_t {
  kActThs = 0x04,
  kActDur = 0x05,
  kIntGenCfgXl = 0x06,
  kIntGenThsXl = 0x07,
  kIntGenThsYl = 0x08,
  kIntGenThsZl = 0x09,
  kIntGenDurXl = 0x0A,
  kReferenceG = 0x0B,
  kInt1Ctrl = 0x0C,
  kInt2Ctrl = 0x0D,
  kWhoAmI = 0x0F,     // Device identification register.
  kCtrlReg1G = 0x10,  // Angular rate sensor Control Register 1.
  kCtrlReg2G = 0x11,
  kCtrlReg3G = 0x12,
  kOrientCfgG = 0x13,
  kIntGenSrcG = 0x14,
  kOutTempL = 0x15,
  kOutTempH = 0x16,
  kStatusReg1 = 0x17,
  kOutXLG = 0x18,  // Angular rate around X-axis, LSB.
  kOutXHG = 0x19,  // Angular rate around X-axis, MSB.
  kOutYLG = 0x1A,
  kOutYHG = 0x1B,
  kOutZLG = 0x1C,
  kOutZHG = 0x1D,
  kCtrlReg4 = 0x1E,
  kCtrlReg5XL = 0x1F,
  kCtrlReg6XL = 0x20,  // Linear acceleration sensor control register 6.
  kCtrlReg7XL = 0x21,
  kCtrlReg8 = 0x22,
  kCtrlReg9 = 0x23,
  kCtrlReg10 = 0x24,
  kIntGenSrcXl = 0x26,
  kStatusReg2 = 0x27,
  kOutXLXL = 0x28,  // Linear acceleration sensor X-axis output register, LSB.
  kOutXHXL = 0x29,  // Linear acceleration sensor X-axis output register, MSB.
  kOutYLXL = 0x2A,
  kOutYHXL = 0x2B,
  kOutZLXL = 0x2C,
  kOutZHXL = 0x2D,
  kFifoCtrl = 0x2E,
  kFifoSrc = 0x2F,
  kIntGenCfgG = 0x30,
  kIntGenThsXHG = 0x31,
  kIntGenThsXLG = 0x32,
  kIntGenThsYHG = 0x33,
  kIntGenThsYLG = 0x34,
  kIntGenThsZHG = 0x35,
  kIntGenThsZLG = 0x36,
  kIntGenDurG = 0x37,
};

// Shared constants.
constexpr uint8_t kRWBitShift = 7U;
constexpr uint8_t kRegAddrMask = 0x7F;
constexpr uint8_t kReadBit = 0x01 << kRWBitShift;
constexpr uint8_t kWriteBit = 0x00;
constexpr uint8_t kTransferTimeoutInMs = 10;
constexpr uint8_t kSingleReadDataByte = 1;

// Angular rate sensor Control Register 1 (kCtrlReg1G).
constexpr uint8_t kCtrlReg1G_PowerDown = 0x00;  // No data output
constexpr uint8_t kCtrlReg1G_14_9Hz = 0x20;     // Output data rate @ 14.9Hz
constexpr uint8_t kCtrlReg1G_59_5Hz = 0x40;     // Output data rate @ 59.5Hz
constexpr uint8_t kCtrlReg1G_119Hz = 0x50;      // Output data rate @ 119Hz
constexpr uint8_t kCtrlReg1G_238Hz = 0x80;      // Output data rate @ 238Hz
constexpr uint8_t kCtrlReg1G_476Hz = 0x90;      // Output data rate @ 476Hz
constexpr uint8_t kCtrlReg1G_952Hz = 0xC0;      // Output data rate @ 952Hz

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_PROTOCOL_H_
