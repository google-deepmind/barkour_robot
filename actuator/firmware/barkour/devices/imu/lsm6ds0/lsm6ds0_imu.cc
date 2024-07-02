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

#include "actuator/firmware/barkour/devices/imu/lsm6ds0/lsm6ds0_imu.h"

#include <cstddef>
#include <cstring>

#include "actuator/firmware/barkour/devices/imu/lsm6ds0/lsm6ds0_imu_protocol.h"
#include "actuator/firmware/targets/m4/FreeRTOSConfig.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"
#include "stm32h7xx_hal.h" // NOLINT
#include "stm32h7xx_hal_gpio.h" // NOLINT
#include "actuator/firmware/barkour/drivers/stm32h7_hal/spi/spi.h"
#include "actuator/firmware/barkour/common/interfaces/spi_interface.h"

namespace barkour {

// Byte indexes for reading all IMU data (acceleration + angular rates).
static constexpr uint8_t kXGyro_LSB = 1U;
static constexpr uint8_t kXGyro_MSB = 2U;
static constexpr uint8_t kYGyro_LSB = 3U;
static constexpr uint8_t kYGyro_MSB = 4U;
static constexpr uint8_t kZGyro_LSB = 5U;
static constexpr uint8_t kZGyro_MSB = 6U;
static constexpr uint8_t kXAccel_LSB = 7U;
static constexpr uint8_t kXAccel_MSB = 8U;
static constexpr uint8_t kYAccel_LSB = 9U;
static constexpr uint8_t kYAccel_MSB = 10U;
static constexpr uint8_t kZAccel_LSB = 11U;
static constexpr uint8_t kZAccel_MSB = 12U;

// Timeouts.
static constexpr std::chrono::milliseconds kReadWriteTimeoutMs =
    std::chrono::milliseconds(10);
static constexpr uint8_t kDelayAfterWriteInMs = 1U;

// IMU Default Values
static constexpr uint8_t kWhoAmIDefault = 0x6C;

// IMU Registers related data.
static constexpr uint8_t kPayloadLength = 2;

// IMU data length with acceleration and angular rates from 3 axis.
static constexpr uint8_t kImuDataLength = 12;

static Spi spi_handler;
pw::sync::TimedThreadNotification* read_write_notification = nullptr;

static void spi_callback(void) {
  if (read_write_notification != nullptr) {
    read_write_notification->release();
  }
}

Lsm6Ds0Imu::Lsm6Ds0Imu() : initialized_(false) {
  memset(&current_imu_state_, 0, sizeof(ImuState));
  memset(rx_buffer, 0, sizeof(uint8_t) * kPayloadLength);
  memset(tx_buffer, 0, sizeof(uint8_t) * kPayloadLength);
}

Lsm6Ds0Imu::~Lsm6Ds0Imu() {
  this->Deinitialize();
}

pw::Status Lsm6Ds0Imu::Initialize() {
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

  // Accelerometer CS GPIO.
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Accelerometer CLK GPIO.
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Accelerometer MISO GPIO.
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Accelerometer MOSI GPIO.
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // SPI timeout synchronisation notification.
  if (read_write_notification == nullptr) {
    read_write_notification = &read_write_notification_;
  }

  // Configure communication with IMU over SPI4.
  PW_TRY(spi_handler.Initialize(SpiReference::kSpi4, spi_callback));

  // Check chip ID.
  uint8_t result = 0x00;
  PW_TRY_ASSIGN(
      result, ReadSingleRegister(ImuRegisterType::kWhoAmIRegister));
  if (result != kWhoAmIDefault) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Chip ID test failed %d", result);
    return pw::Status::FailedPrecondition();
  }

  // Verify automatic address increments during multi-byte reads.
  PW_TRY_ASSIGN(result,
                ReadSingleRegister(ImuRegisterType::kCtrl3C));
  if (!(result & kCtrl3C_IfIncBit)) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Auto address increments test failed");
    return pw::Status::FailedPrecondition();
  }

  // Configure accelerometer data output rate to 1.66kHz.
  PW_TRY(WriteSingleRegister(ImuRegisterType::kCtrl1Xl,
                                        (kOdr1_66khz | kCtrl1Xl_Fs_2g)));
  // Wait for kDelayAfterWriteInMs milliseconds before validating.
  vTaskDelay(pdMS_TO_TICKS(kDelayAfterWriteInMs));
  PW_TRY_ASSIGN(result,
                ReadSingleRegister(ImuRegisterType::kCtrl1Xl));
  if (!(result & (kOdr1_66khz | kCtrl1Xl_Fs_2g))) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Accelerometer output rate not set to 1.66kHz");
    return pw::Status::FailedPrecondition();
  }

  // Configure gyroscope data output rate to 1.66kHz.
  PW_TRY(WriteSingleRegister(ImuRegisterType::kCtrl2G,
                                        (kOdr1_66khz | kCtrl2G_Fs_250dps)));
  // Wait for kDelayAfterWriteInMs milliseconds before validating.
  vTaskDelay(pdMS_TO_TICKS(kDelayAfterWriteInMs));
  PW_TRY_ASSIGN(result,
                ReadSingleRegister(ImuRegisterType::kCtrl2G));
  if (!(result & (kOdr1_66khz | kCtrl2G_Fs_250dps))) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Gyroscope output rate not set to 1.66kHz");
    return pw::Status::FailedPrecondition();
  }

  initialized_ = true;
  PW_LOG_INFO("[IMU-LSM6DS0] Init ok");
  return pw::OkStatus();
}

pw::Status Lsm6Ds0Imu::Deinitialize() {
  // Accelerometer CS GPIO.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);

  // Accelerometer CLK GPIO.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12);

  // Accelerometer MISO GPIO.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_13);

  // Accelerometer MOSI GPIO.
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_14);

  if (spi_handler.Deinitialize() != pw::OkStatus()) {
    return pw::Status::Internal();
  }
  read_write_notification = nullptr;
  initialized_ = false;
  PW_LOG_INFO("[IMU-LSM6DS0] Deinit ok.");
  return pw::OkStatus();
}

pw::Result<ImuState> Lsm6Ds0Imu::GetImuState() {
  // Read acceleration and angular rates values.
  PW_TRY(ReadMultipleRegisters(ImuRegisterType::kOutXLG,
                                          kImuDataLength));

  // Convert raw data to acceleration[m/s^2] and angular rates[rad/s].
  current_imu_state_.gyroscope[ImuAxis::kImuX] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kXGyro_LSB] |
                                              rx_buffer[kXGyro_MSB] << 8U)) *
      kAngularRate250dps_RadPerSecPerBit;

  current_imu_state_.gyroscope[ImuAxis::kImuY] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kYGyro_LSB] |
                                              rx_buffer[kYGyro_MSB] << 8U)) *
      kAngularRate250dps_RadPerSecPerBit;

  current_imu_state_.gyroscope[ImuAxis::kImuZ] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kZGyro_LSB] |
                                              rx_buffer[kZGyro_MSB] << 8U)) *
      kAngularRate250dps_RadPerSecPerBit;

  current_imu_state_.accelerometer[ImuAxis::kImuX] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kXAccel_LSB] |
                                              rx_buffer[kXAccel_MSB] << 8U)) *
      kAccelRate2G_AccelPerBit;

  current_imu_state_.accelerometer[ImuAxis::kImuY] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kYAccel_LSB] |
                                              rx_buffer[kYAccel_MSB] << 8U)) *
      kAccelRate2G_AccelPerBit;

  current_imu_state_.accelerometer[ImuAxis::kImuZ] =
      static_cast<float>(static_cast<int16_t>(rx_buffer[kZAccel_LSB] |
                                              rx_buffer[kZAccel_MSB] << 8U)) *
      kAccelRate2G_AccelPerBit;

  return current_imu_state_;
}


pw::Status Lsm6Ds0Imu::WriteSingleRegister(
    ImuRegisterType register_type, uint8_t value) {
  tx_buffer[0] =
      kWriteBit | (static_cast<uint8_t>(register_type) & kRegAddrMask);
  tx_buffer[1] = value;

  // Send data and wait for the notification release called by SPI interrupt
  // callback in spi_callback for kReadWriteTimeoutMs milliseconds.
  if (spi_handler.ExchangeData(rx_buffer, tx_buffer, kPayloadLength)
      != pw::OkStatus()) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Writing to a single register over SPI failed");
    return pw::Status::Unavailable();
  }

  return pw::OkStatus();
}

pw::Result<uint8_t> Lsm6Ds0Imu::ReadSingleRegister(
    ImuRegisterType register_type) {
  tx_buffer[0] =
      kReadBit | (static_cast<uint8_t>(register_type) & kRegAddrMask);

  // Send data and wait for the notification release called by SPI interrupt
  // callback in spi_callback for kReadWriteTimeoutMs milliseconds.
  if (spi_handler.ExchangeData(rx_buffer, tx_buffer, kPayloadLength)
      != pw::OkStatus()) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Reading a single register over SPI failed");
    return pw::Status::Unavailable();
  }

  if (!read_write_notification_.try_acquire_for(kReadWriteTimeoutMs)) {
    return pw::Status::DeadlineExceeded();
  }

  return rx_buffer[kSingleReadDataByte];
}

pw::Status Lsm6Ds0Imu::ReadMultipleRegisters(
    ImuRegisterType register_type, uint8_t length_to_read_in_bytes) {
  tx_buffer[0] =
      kReadBit | (static_cast<uint8_t>(register_type) & kRegAddrMask);

  // Send data and wait for the notification release called by SPI interrupt
  // callback in spi_callback for kReadWriteTimeoutMs milliseconds.
  if (spi_handler.ExchangeData(rx_buffer, tx_buffer,
                               length_to_read_in_bytes + 1) != pw::OkStatus()) {
    PW_LOG_ERROR("[IMU-LSM6DS0] Reading multiple registers over SPI failed");
    return pw::Status::Unavailable();
  }

  if (!read_write_notification_.try_acquire_for(kReadWriteTimeoutMs)) {
    return pw::Status::DeadlineExceeded();
  }

  return pw::OkStatus();
}

}  // namespace barkour
