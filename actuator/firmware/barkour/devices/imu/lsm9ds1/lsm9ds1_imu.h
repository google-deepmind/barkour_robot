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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "pw_bytes/span.h"
#include "pw_result/result.h"
#include "pw_span/span.h"
#include "pw_status/status.h"
#include "pw_sync/timed_thread_notification.h"
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

class Lsm9Ds1Imu : public ImuInterface {
 public:
  Lsm9Ds1Imu();
  ~Lsm9Ds1Imu();

  // Initializes the IMU.
  //
  // Must be called to calling GetImuState().
  pw::Status Initialize() override;

  // Deinitializes the IMU and all its associated peripherals.
  //
  // Returns: pw::StatusOk() when successful
  pw::Status Deinitialize() override;

  // Returns pw::Status::FailedPrecondition() if the IMU is not initialized.
  pw::Result<ImuState> GetImuState() override;

 private:
  // Reads one, or multiple registers of the LSM9DS1.
  //
  // Registers are read incrementally by address, e.g:
  //   `ReadRegisters(0x28, output)`, with `output.size() == 3`, will read
  //   registers 0x28, 0x29, and 0x2A.
  //
  // Note: Calls to this function may trigger a burst read, depending upon the
  //  device configuration, read address, and the number of registers being
  //  read. Please see the device datasheet for further detail.
  pw::Status ReadRegisters(uint8_t address, pw::ByteSpan output);

  // Writes data to registers, starting from address.
  //
  // Registers are written incrementally by address, e.g:
  //   `WriteRegisters(0x10, data)`, `with data.size() == 3`, will write
  //    registers 0x10, 0x11, and 0x12.
  pw::Status WriteRegisters(uint8_t address, pw::ConstByteSpan data);

  bool initialized_;
  static constexpr uint8_t kLsm9ds1SpiBufferSize = 13;
  uint8_t spi_tx_buffer_[kLsm9ds1SpiBufferSize];
  uint8_t spi_rx_buffer_[kLsm9ds1SpiBufferSize];
  pw::sync::TimedThreadNotification read_write_completed_notification_;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM9DS1_LSM9DS1_IMU_H_
