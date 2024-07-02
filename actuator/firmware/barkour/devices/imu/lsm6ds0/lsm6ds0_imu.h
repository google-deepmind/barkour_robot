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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "actuator/firmware/barkour/devices/imu/lsm6ds0/lsm6ds0_imu_protocol.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_sync/timed_thread_notification.h"

namespace barkour {

// Implementation of LSM6DS0 IMU.
class Lsm6Ds0Imu : public ImuInterface {
 public:
  Lsm6Ds0Imu();
  ~Lsm6Ds0Imu();

  // Initialises the IMU.
  // Returns an error if the IMU initialisation fails.
  pw::Status Initialize() override;

  pw::Status Deinitialize() override;

  // Returns the current IMU data.
  pw::Result<ImuState> GetImuState() override;

 private:
  // Most up-to-date imu values.
  ImuState current_imu_state_;

  // Boolean to check if IMU is initialised.
  bool initialized_;

  // Reads a single register.
  //
  // Args:
  // - register_type: register to read.
  //
  // Returns value received in uint8_t or pw::Status during failure.
  pw::Result<uint8_t> ReadSingleRegister(
      ImuRegisterType register_type);

  // Reads multiple registers.
  //
  // Args:
  // - register_type: initial register address to read from.
  // - length_to_read_in_bytes: number of bytes to read starting from address of
  //   register_type.
  //
  // Note: addresses are auto-incremented when kCtrl3C is correctly
  // configured.
  //
  // Returns
  // - pw::StatusOK() when reading succeeded.
  pw::Status ReadMultipleRegisters(ImuRegisterType register_type,
                                              uint8_t length_to_read_in_bytes);

  // Writes to a single register.
  //
  // Args:
  // - register_type: register to read.
  // - value: value to write.
  //
  // Returns
  // - pw::StatusOK() when writing succeeded.
  pw::Status WriteSingleRegister(ImuRegisterType register_type,
                                            uint8_t value);

  // IMU Registers related data.
  static constexpr uint8_t kMaxPayloadLength = 20;

  // Buffers for read/write.
  uint8_t rx_buffer[kMaxPayloadLength];
  uint8_t tx_buffer[kMaxPayloadLength];

  // Synchronisation timeout.
  pw::sync::TimedThreadNotification read_write_notification_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_LSM6DS0_LSM6DS0_IMU_H_
