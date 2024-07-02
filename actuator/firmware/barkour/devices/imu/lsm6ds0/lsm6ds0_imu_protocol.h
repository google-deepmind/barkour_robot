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

// LSMx Protocol defines protocol variables shared between LSM6DS0 and LSM9DS1
// IMUs.
#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_PROTOCOL_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_PROTOCOL_H_

// Lsm9Ds1Imu/Lsm6Ds0Imu read/write transaction:
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
// the read transaction. See the LSM9DS1/LSM6DS0 datasheet for more detail.

#include <cstdint>

namespace barkour {

// Defines IMU memory registers by their 8-bit addresses.
// The full register map is available in LSM9DS1/LSM6DS0 datasheets.
enum class ImuRegisterType : uint8_t {
  kFuncCfgAccess = 0x01,
  kPinCtrl = 0x02,
  kFifoCtrl1 = 0x07,
  kFifoCtrl2 = 0x08,
  kFifoCtrl3 = 0x09,
  kFifoCtrl4 = 0x0A,
  kCounterBdrReg1 = 0x0B,
  kCounterBdrReg2 = 0x0C,
  kInt1Ctrl = 0x0D,
  kInt2Ctrl = 0x0E,
  kWhoAmIRegister = 0x0F,  // Device identification register.
  kCtrl1Xl = 0x10,         // Accelerometer control register 1.
  kCtrl2G = 0x11,          // Gyroscope control register 2.
  kCtrl3C = 0x12,
  kCtrl4C = 0x13,
  kCtrl5C = 0x14,
  kCtrl6C = 0x15,
  kCtrl7C = 0x16,
  kCtrl8Xl = 0x17,
  kCtrl9Xl = 0x18,
  kCtrl10C = 0x19,
  kAllIntSrc = 0x1A,
  kWakeUpSrc = 0x1B,
  kTapSrc = 0x1C,
  kD6DSrc = 0x1D,
  kStatusReg = 0x1E,
  kOutTempL = 0x20,
  kOutTempH = 0x21,
  kOutXLG = 0x22,
  kOutXHG = 0x23,
  kOutYLG = 0x24,
  kOutYHG = 0x25,
  kOutZLG = 0x26,
  kOutZHG = 0x27,
  kOutXLA = 0x28,
  kOutXHA = 0x29,
  kOutYLA = 0x2A,
  kOutYHA = 0x2B,
  kOutZLA = 0x2C,
  kOutZHA = 0x2D,
  kEmbFuncStatusMainpage = 0x35,
  kFsmStatusAMainpage = 0x36,
  kFsmStatusBMainpage = 0x37,
  kStatusMasterMainpage = 0x39,
  kFifoStatus1 = 0x3A,
  kFifoStatus2 = 0x3B,
  kTimestamp0 = 0x40,
  kTimestamp1 = 0x41,
  kTimestamp2 = 0x42,
  kTimestamp3 = 0x43,
  kTapCfg0 = 0x56,
  kTapCfg1 = 0x57,
  kTapCfg2 = 0x58,
  kTapThs6D = 0x59,
  kIntDur2 = 0x5A,
  kWakeUpThs = 0x5B,
  kWakeUpDur = 0x5C,
  kFreeFall = 0x5D,
  kMd1Cfg = 0x5E,
  kMd2Cfg = 0x5F,
  kI3CBusAvb = 0x62,
  kInternalFreqFine = 0x63,
  kIntOis = 0x6F,
  kCtrl1Ois = 0x70,
  kCtrl2Ois = 0x71,
  kCtrl3Ois = 0x72,
  kXOfsUsr = 0x73,
  kYOfsUsr = 0x74,
  kZOfsUsr = 0x75,
  kFifoDataOutTag = 0x78,
  kFifoDataOutXL = 0x79,
  kFifoDataOutXH = 0x7A,
  kFifoDataOutYL = 0x7B,
  kFifoDataOutYH = 0x7C,
  kFifoDataOutZL = 0x7D,
  kFifoDataOutZH = 0x7E,
};

// Constants.
constexpr uint8_t kRWBitShift = 7U;
constexpr uint8_t kRegAddrMask = 0x7F;
constexpr uint8_t kReadBit = 0x01 << kRWBitShift;
constexpr uint8_t kWriteBit = 0x00;
constexpr uint8_t kTransferTimeoutInMs = 10;
constexpr uint8_t kSingleReadDataByte = 1;
constexpr float kGravity = 9.807;               // [m/s^2]
constexpr float kDpsToRadPerSec = 0.01745329f;  // [rad/degree]

// Acceleration sensitivity multipliers
constexpr float kAccelRate2G_AccelInGPerBit = 0.000061f;
constexpr float kAccelRate2G_AccelPerBit =
    kAccelRate2G_AccelInGPerBit * kGravity;  // [m/s^2]

// Angular rate sensitivity multipliers
constexpr float kAngularRate250dps_DpsPerBit = 0.00875f;
constexpr float kAngularRate250dps_RadPerSecPerBit =
    kAngularRate250dps_DpsPerBit * kDpsToRadPerSec;  // [rad/s]

// Control Register 3.
constexpr uint8_t kCtrl3C_IfIncBit = (1 << 2U);

// Output data rate for accelerometer and gyroscope.
constexpr uint8_t kOdr1_66khz = 0x80;  // 1.66kHz output rate

// Gyroscope control register 2.
constexpr uint8_t kCtrl2G_Fs_125dps = 0x02;   // Gyro full-scale 125 dps
constexpr uint8_t kCtrl2G_Fs_250dps = 0x00;   // Gyro full-scale 250 dps
constexpr uint8_t kCtrl2G_Fs_500dps = 0x04;   // Gyro full-scale 500 dps
constexpr uint8_t kCtrl2G_Fs_1000dps = 0x08;  // Gyro full-scale 1000 dps
constexpr uint8_t kCtrl2G_Fs_2000dps = 0x0C;  // Gyro full-scale 2000 dps

// Accelerometer control register 1 (assumed XL_FS_MODE='0').
constexpr uint8_t kCtrl1Xl_Fs_2g = 0x00;   // Accelerometer full-scale 2g
constexpr uint8_t kCtrl1Xl_Fs_16g = 0x04;  // Accelerometer full-scale 16g
constexpr uint8_t kCtrl1Xl_Fs_4g = 0x08;   // Accelerometer full-scale 4g
constexpr uint8_t kCtrl1Xl_Fs_8g = 0x0C;   // Accelerometer full-scale 8g

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_PROTOCOL_H_
