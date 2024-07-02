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

#include "actuator/firmware/barkour/common/canopen_unit_conversions.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "actuator/firmware/barkour/common/math_constants.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

pw::Result<int16_t> CurrentToCia402Torque(float amps, uint32_t torque_constant,
                                          uint32_t cia_402_rated_torque) {
  if (cia_402_rated_torque == 0) {
    PW_LOG_WARN(
        "Rated torque is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  if (torque_constant == 0) {
    PW_LOG_WARN(
        "Torque constant is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  float torque = (amps * torque_constant / cia_402_rated_torque * 1e3);
  float abs_torque = std::fabs(torque);

  if (abs_torque > std::numeric_limits<int16_t>::max() ||
      -abs_torque < std::numeric_limits<int16_t>::min()) {
    PW_LOG_WARN(
        "Current too high to represent as Cia402 torque value: %f. "
        "Must be < 2^15-1",
        torque);
    return pw::Status::InvalidArgument();
  }

  return torque;
}

pw::Result<int16_t> CurrentToCia402Current(float amps,
                                           uint32_t cia_402_rated_current) {
  if (cia_402_rated_current == 0) {
    PW_LOG_WARN(
        "Rated current is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  float current = (1e6f * amps / cia_402_rated_current);

  float abs_current = std::fabs(current);

  if (abs_current > std::numeric_limits<int16_t>::max() ||
      -abs_current < std::numeric_limits<int16_t>::min()) {
    PW_LOG_WARN(
        "Current too high to represent as Cia402 current value: %f. "
        "Must be < 2^15-1",
        current);
    return pw::Status::InvalidArgument();
  }

  return current;
}

pw::Result<float> Cia402TorqueToCurrent(int16_t cia_402_torque,
                                        uint32_t torque_constant,
                                        uint32_t cia_402_rated_torque) {
  if (cia_402_rated_torque == 0) {
    PW_LOG_WARN(
        "Rated torque is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  if (torque_constant == 0) {
    PW_LOG_WARN(
        "Torque constant is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  // Order matters here because rated torque is unsigned.
  // First convert the unsigned rated torque to float.
  return cia_402_rated_torque * 1e-3f * cia_402_torque / torque_constant;
}

pw::Result<float> Cia402CurrentToCurrent(int16_t cia_402_current,
                                         uint32_t cia_402_rated_current) {
  if (cia_402_rated_current == 0) {
    PW_LOG_WARN(
        "Rated current is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }
  // Order matters due to unsigned -> signed conversion.
  return (1e-3F * cia_402_rated_current) * 1e-3F * cia_402_current;
}

pw::Result<int32_t> PositionToCia402Position(float shaft_position,
                                             uint32_t counts_per_revolution) {
  if (counts_per_revolution == 0) {
    PW_LOG_WARN(
        "Counts per revolution is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  float encoder_counts = counts_per_revolution * (shaft_position / kTwoPi);
  float abs_counts = std::fabs(encoder_counts);

  if (abs_counts > static_cast<float>(std::numeric_limits<int32_t>::max()) ||
      -abs_counts < static_cast<float>(std::numeric_limits<int32_t>::min())) {
    PW_LOG_WARN(
        "Position too large to represent as Cia402 position value: %f. "
        "Must be < 2^31-1",
        encoder_counts);
    return pw::Status::InvalidArgument();
  }

  return std::round(encoder_counts);
}

pw::Result<float> Cia402PositionToPosition(int32_t cia_402_position,
                                           uint32_t counts_per_revolution) {
  if (counts_per_revolution == 0) {
    PW_LOG_WARN(
        "Counts per revolution is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  return kTwoPi * static_cast<float>(cia_402_position) /
         static_cast<float>(counts_per_revolution);
}

// Logic for velocity conversions is exactly the same as for position, but with
// different PW_LOG statements.
pw::Result<int32_t> VelocityToCia402Velocity(float shaft_velocity,
                                             uint32_t counts_per_revolution) {
  if (counts_per_revolution == 0) {
    PW_LOG_WARN(
        "Counts per revolution is 0. This is invalid and likely a "
        "configuration error.");
    return pw::Status::InvalidArgument();
  }

  float encoder_counts_per_second =
      counts_per_revolution * (shaft_velocity / kTwoPi);
  float abs_counts_per_second = std::fabs(encoder_counts_per_second);

  if (abs_counts_per_second >
          static_cast<float>(std::numeric_limits<int32_t>::max()) ||
      -abs_counts_per_second <
          static_cast<float>(std::numeric_limits<int32_t>::min())) {
    PW_LOG_WARN(
        "Velocity too large to represent as Cia402 velocity value: %f. "
        "Must be < 2^31-1",
        encoder_counts_per_second);
    return pw::Status::InvalidArgument();
  }

  return std::round(encoder_counts_per_second);
}

pw::Result<float> Cia402VelocityToVelocity(int32_t cia_402_velocity,
                                           uint32_t counts_per_revolution) {
  return Cia402PositionToPosition(cia_402_velocity, counts_per_revolution);
}

}  // namespace barkour
