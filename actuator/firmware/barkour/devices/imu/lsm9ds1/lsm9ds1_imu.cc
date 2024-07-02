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

#include "actuator/firmware/barkour/devices/imu/lsm9ds1/lsm9ds1_imu.h"

#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/devices/imu/lsm9ds1/lsm9ds1_imu_protocol.h"
#include "pw_assert/check.h"
#include "pw_bytes/bit.h"
#include "pw_log/log.h"
#include "pw_status/try.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/spi/spi.h"
#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"

static_assert(
    pw::endian::native == pw::endian::little,
    "lsm9ds1_imu module only supports little-endian architectures, for now.");

namespace barkour {

// Timeouts.
constexpr std::chrono::milliseconds kTimeoutDuration =
    std::chrono::milliseconds(1000);

// IMU Default Values
static constexpr uint8_t kWhoAmIDefault = 0x68;

constexpr float kGravitationalAcceleration = 9.80665f;  // [meters/second^2]
constexpr float kAccelerometerSensitivityScalingFactor2g =
    2.0f * kGravitationalAcceleration;

constexpr float kGyroSensitivityScalingFactor245 = 245.0f;  // [degrees/second]

// OUT_XL_XL: Linear acceleration sensor X-axis output register, low-byte.
constexpr uint8_t kOutXlXl = 0x28;

pw::sync::TimedThreadNotification* read_write_completed_notification = nullptr;

static Spi spi;

static void spi_callback(void) {
  if (read_write_completed_notification != nullptr) {
    read_write_completed_notification->release();
  }
}

Lsm9Ds1Imu::Lsm9Ds1Imu() : initialized_(false){};

Lsm9Ds1Imu::~Lsm9Ds1Imu() {
  this->Deinitialize();
}

pw::Status Lsm9Ds1Imu::Initialize() {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  // Magnetometer CS pin configuration.
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Accelerometer/Gyro Data enable pin.
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Gyro/Acceleromter CS GPIO pin configuration.
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // SPI SCK GPIO pin configuration.
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // SPI MISO GPIO pin configuration.
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // SPI MOSI GPIO pin configuration.
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  if (read_write_completed_notification != nullptr) {
    PW_LOG_ERROR("[IMU-LSM9DS1] read_write_completed_notification already set");
    return pw::Status::Internal();
  }
  read_write_completed_notification = &read_write_completed_notification_;

  if (spi.Initialize(SpiReference::kSpi4, spi_callback) != pw::OkStatus()) {
    return pw::Status::Internal();
  }

  // CS_M: Disable magnetometer.
  // HIGH: Magnetometer SPI disabled/Magnetometer I2C enabled.
  // LOW:  Magnetometer SPI enabled/Magnetometer SPI disabled.
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);

  // CS_A/G: Enable accelerometer and gyroscope.
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

  // DEN_A/G: Accelerometer and gyroscope data enable.
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

  // Sanity check to ensure that registers can be read properly from the
  // device.
  std::array<std::byte, 1> who_am_i_response;
  PW_TRY(ReadRegisters(static_cast<uint8_t>(ImuRegisterType::kWhoAmI),
                       who_am_i_response));
  if (who_am_i_response[0] != std::byte(kWhoAmIDefault)) {
    PW_LOG_ERROR("[IMU-LSM9DS1] Chip ID test failed");
    return pw::Status::Internal();
  }

  // Enable the accelerometer and gyro, with 14.9Hz update frequency.
  std::array<const std::byte, 1> values_to_write = {
      std::byte(static_cast<uint8_t>(ImuRegisterType::kCtrlReg6XL))};
  PW_TRY(WriteRegisters(static_cast<uint8_t>(ImuRegisterType::kCtrlReg1G),
                        values_to_write));

  initialized_ = true;
  PW_LOG_INFO("[IMU-LSM9DS1] IMU Init OK");
  return pw::OkStatus();
}

pw::Status Lsm9Ds1Imu::Deinitialize() {
  // Magnetometer CS pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);

  // Accelerometer/Gyro CS pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);

  // SPI SCK pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12);

  // SPI MISO pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_13);

  // SPI MOSI pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_14);

  // Accelerometer/Gyro data enable pin.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_15);

  if (spi.Deinitialize() != pw::OkStatus()) {
    return pw::Status::Internal();
  }
  read_write_completed_notification = nullptr;
  initialized_ = false;
  PW_LOG_INFO("[IMU-LSM9DS1] Deinit ok");
  return pw::OkStatus();
}

pw::Result<ImuState> Lsm9Ds1Imu::GetImuState() {
  if (!initialized_) {
    return pw::Status::FailedPrecondition();
  }

  // When the accelerometer and gyroscope are both enabled, it is possible to
  // obtain sensor readings for both using a single burst transaction.
  //
  // Burst read response:
  // | Gyro. X   | Gyro. Y   | Gyro. Z   | Accel. X  | Accel. Y  | Accel. Z    |
  // | Low  High | Low  High | Low  High | Low  High | Low  High | Low   High  |
  // | x[0] x[1] | x[2] x[3] | x[4] x[5] | x[6] x[7] | x[8] x[9] | x[10] x[11] |
  //
  // Adjacent 8-bit sensor reading registers form a 16-bit word in two's
  // compliment.
  std::array<std::byte, 12> values;
  PW_TRY(ReadRegisters(static_cast<uint8_t>(ImuRegisterType::kOutXLG), values));

  // Convert gyroscope readings to float, units [radians/second].
  const int16_t* gyro_counts_signed =
      reinterpret_cast<const int16_t*>(values.data());

  // Normalize.
  float x_gyro = static_cast<float>(gyro_counts_signed[0]) /
                 std::numeric_limits<int16_t>::max();
  float y_gyro = static_cast<float>(gyro_counts_signed[1]) /
                 std::numeric_limits<int16_t>::max();
  float z_gyro = static_cast<float>(gyro_counts_signed[2]) /
                 std::numeric_limits<int16_t>::max();

  // Apply sensitivity scaling factor.
  x_gyro *= kGyroSensitivityScalingFactor245;
  y_gyro *= kGyroSensitivityScalingFactor245;
  z_gyro *= kGyroSensitivityScalingFactor245;

  // Finally, convert units from degrees/second to radians/sec.
  x_gyro *= kPi / 180.0f;
  y_gyro *= kPi / 180.0f;
  z_gyro *= kPi / 180.0f;

  // Convert accelerometer readings to float, units [meters/second^2]
  const int16_t* accel_counts_signed =
      reinterpret_cast<const int16_t*>(&values[6]);

  // Normalize.
  float x_accel = static_cast<float>(accel_counts_signed[0]) /
                  std::numeric_limits<int16_t>::max();
  float y_accel = static_cast<float>(accel_counts_signed[1]) /
                  std::numeric_limits<int16_t>::max();
  float z_accel = static_cast<float>(accel_counts_signed[2]) /
                  std::numeric_limits<int16_t>::max();

  // Apply sensitivity scaling factor.
  x_accel *= kAccelerometerSensitivityScalingFactor2g;
  y_accel *= kAccelerometerSensitivityScalingFactor2g;
  z_accel *= kAccelerometerSensitivityScalingFactor2g;

  return ImuState{
      .accelerometer = {{x_accel, y_accel, z_accel}},
      .gyroscope = {{x_gyro, y_gyro, z_gyro}},
  };
}

pw::Status Lsm9Ds1Imu::ReadRegisters(uint8_t address, pw::ByteSpan output) {
  if (output.size_bytes() + 1 > kLsm9ds1SpiBufferSize) {
    return pw::Status::InvalidArgument();
  }

  memset(spi_tx_buffer_, 0, output.size_bytes() + 1);
  memset(spi_rx_buffer_, 0, output.size_bytes() + 1);

  spi_tx_buffer_[0] = (kReadBit | (address & 0x3F));

  if (spi.ExchangeData(spi_rx_buffer_, spi_tx_buffer_, output.size_bytes() + 1)
      != pw::OkStatus()) {
    return pw::Status::Internal();
  }

  if (!read_write_completed_notification_.try_acquire_for(kTimeoutDuration)) {
    return pw::Status::DeadlineExceeded();
  }

  memcpy(output.data(), &spi_rx_buffer_[1], output.size_bytes());
  return pw::OkStatus();
}

pw::Status Lsm9Ds1Imu::WriteRegisters(uint8_t address, pw::ConstByteSpan data) {
  if (data.size() + 1 > kLsm9ds1SpiBufferSize) {
    return pw::Status::InvalidArgument();
  }

  memset(spi_rx_buffer_, 0, data.size() + 1);

  spi_tx_buffer_[0] = (kWriteBit | (address & 0x3F));
  memcpy(&spi_tx_buffer_[1], data.data(), data.size());

  if (spi.ExchangeData(spi_rx_buffer_, spi_tx_buffer_, data.size() + 1)
      != pw::OkStatus()) {
    return pw::Status::Internal();
  }

  if (!read_write_completed_notification_.try_acquire_for(kTimeoutDuration)) {
    return pw::Status::DeadlineExceeded();
  }

  return pw::OkStatus();
}

}  // namespace barkour.
