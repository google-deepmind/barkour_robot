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

#include "actuator/firmware/barkour/common/read_sensors.h"

#include <cstdint>

#include "actuator/firmware/barkour/common/adc_conversions.h"
#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

namespace {

pw::Result<float> GetSampleAndConvertToFloat(AdcInterface& adc,
                                             AdcAnalogSignal signal) {
  PW_TRY_ASSIGN(int32_t raw_value, adc.GetSample(signal));
  return AdcReadingToFloat(signal, raw_value);
}

}  // namespace

SensorReader::SensorReader(AdcInterface& adc, RotaryEncoder& encoder,
                           FocInterface& foc, ImuInterface& imu,
                           uint8_t max_consecutive_encoder_data_loss_errors)
    : adc_(adc),
      encoder_(encoder),
      foc_(foc),
      imu_(imu),
      max_consecutive_encoder_data_loss_errors_(
          max_consecutive_encoder_data_loss_errors),
      encoder_data_loss_error_counter_(0) {
}

pw::Result<SensorReadings> SensorReader::GetLatestReadingsFromAllSensors() {
  PW_TRY_ASSIGN(float bus_voltage,
                GetSampleAndConvertToFloat(adc_, AdcAnalogSignal::kBusVoltage));

  // Get latest data from Foc controller.
  FocInterface::FocState foc_state = foc_.GetState();

  // Thermistors.
  PW_TRY_ASSIGN(float thermistor_1, GetSampleAndConvertToFloat(
                                        adc_, AdcAnalogSignal::kMotorTherm1));
  PW_TRY_ASSIGN(float thermistor_2, GetSampleAndConvertToFloat(
                                        adc_, AdcAnalogSignal::kMotorTherm2));
  PW_TRY_ASSIGN(float thermistor_3, GetSampleAndConvertToFloat(
                                        adc_, AdcAnalogSignal::kMotorTherm3));
  PW_TRY_ASSIGN(float thermistor_4, GetSampleAndConvertToFloat(
                                        adc_, AdcAnalogSignal::kEncoderTherm));

  PW_TRY_ASSIGN(
      float sto_24v_safe_adc_voltage,
      GetSampleAndConvertToFloat(adc_, AdcAnalogSignal::k24VSafeVoltage));

  // Read the IMU.
  PW_TRY_ASSIGN(ImuState imu_state, imu_.GetImuState());

  // Encoder.
  pw::Result<RotaryEncoderState> encoder_reading_result =
      encoder_.GetEncoderState();

  int32_t reported_encoder_position = 0;
  uint32_t reported_raw_encoder_count = 0;

  switch (encoder_reading_result.status().code()) {
    case pw::Status::Code::PW_STATUS_OK: {
      // Require multiturn encoder readings to be available.
      if (!encoder_reading_result->multiturn_encoder_counts.has_value()) {
        return pw::Status::Internal();
      }
      reported_encoder_position =
          encoder_reading_result->multiturn_encoder_counts.value();
      last_good_encoder_reading_ = reported_encoder_position;

      reported_raw_encoder_count = encoder_reading_result->raw_encoder_counts;
      last_good_raw_encoder_reading_ = reported_raw_encoder_count;

      encoder_data_loss_error_counter_ = 0;
      break;
    }

    case pw::Status::Code::PW_STATUS_DATA_LOSS: {
      PW_LOG_DEBUG(
          "Got data loss error from encoder, incrementing counter. Current "
          "counter value: %d.",
          ++encoder_data_loss_error_counter_);
      if (!last_good_encoder_reading_.has_value()) {
        PW_LOG_WARN(
            "Encoder data loss error with no previous reading to fall back "
            "on.");
        return pw::Status::DataLoss();
      } else if (encoder_data_loss_error_counter_ >
                 max_consecutive_encoder_data_loss_errors_) {
        PW_LOG_WARN(
            "Exceeded maximum number of consecutive encoder data loss errors.");
        return pw::Status::DataLoss();
      }
      reported_encoder_position = last_good_encoder_reading_.value();
      reported_raw_encoder_count = last_good_raw_encoder_reading_.value();
      break;
    }

    default: {
      return encoder_reading_result.status();
    }
  }

  return SensorReadings{.raw_encoder_count = reported_raw_encoder_count,
                        .encoder_position = reported_encoder_position,
                        .electrical_angle = foc_state.elec_angle_,
                        .quadrature_current = foc_state.i_q_,
                        .direct_current = foc_state.i_d_,
                        .phase_a_current = foc_state.phase_a_current_,
                        .phase_b_current = foc_state.phase_b_current_,
                        .phase_c_current = foc_state.phase_c_current_,
                        .bus_voltage = bus_voltage,
                        .thermistor_1 = thermistor_1,
                        .thermistor_2 = thermistor_2,
                        .thermistor_3 = thermistor_3,
                        .thermistor_4 = thermistor_4,
                        .sto_24v_safe_adc_voltage = sto_24v_safe_adc_voltage,
                        .imu_state = imu_state};
}

}  // namespace barkour
