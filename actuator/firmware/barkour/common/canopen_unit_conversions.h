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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CANOPEN_UNIT_CONVERSIONS_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CANOPEN_UNIT_CONVERSIONS_H_

// Utilities to convert CANopen units (mainly CiA402) and SI units.

#include <cstdint>

#include "pw_result/result.h"

namespace barkour {

// Converts a motor current (A) value to CiA 402 torque.
//
// Note: The torque constant is not part of CiA402, hence no cia_402_prefix for
// this argument.
//
// Args:
// - amps: Motor current in Ampere.
// - torque_constant: Torque constant in mNm/A
// - cia_402_rated_torque: Rated torque in mNm.
//
// Returns: Torque in CiA402 units (1000ths of rated torque).
// Returns InvalidArgument in case of an overflow or invalid rated torque.
pw::Result<int16_t> CurrentToCia402Torque(float amps,
                                          uint32_t torque_constant,
                                          uint32_t cia_402_rated_torque);

// Converts a CiA 402 torque value to a Cia 402 current value.
//
// Note: The torque constant is not part of CiA402, hence no cia_402_prefix for
// this argument.
//
// Args:
// - cia_402_torque: Torque in CiA402 units (1000ths of rated torque)
// - torque_constant: Torque constant in mNm/A
// - cia_402_rated_torque: Rated torque in mNm.
//
// Returns: Current in Amperes.
// Returns InvalidArgument in case of an invalid rated torque or torque
// constant.
pw::Result<float> Cia402TorqueToCurrent(int16_t cia_402_torque,
                                        uint32_t torque_constant,
                                        uint32_t cia_402_rated_torque);

// Converts a motor current (A) value to CiA 402 current value.
//
// Args:
// - amps: Motor current in Ampere.
// - cia_402_rated_current: Rated current in mA.
//
// Returns: Current in CiA402 units (1000ths of rated current).
// Returns InvalidArgument in case of an overflow or invalid rated current.
pw::Result<int16_t> CurrentToCia402Current(float amps,
                                           uint32_t cia_402_rated_current);

// Converts a CiA 402 current value to a motor current (A) value.
//
// Args:
// - amps: Current in CiA402 units (1000ths of rated current).
// - cia_402_rated_current: Rated current in mA.
//
// Returns: Current in Amperes.
// Returns InvalidArgument in case of an invalid rated current.
pw::Result<float> Cia402CurrentToCurrent(int16_t cia_402_current,
                                         uint32_t cia_402_rated_current);

// Converts a shaft position (Rad) value to CiA 402 position value.
//
// Args:
// - shaft_position: Shaft position in radians.
// - counts_per_revolution: Encoder counts per shaft revolution.
//
// Returns: Position in CiA402 units (encoder ticks).
// Returns InvalidArgument in case of an overflow or invalid counts per
// revolution.
pw::Result<int32_t> PositionToCia402Position(float shaft_position,
                                             uint32_t counts_per_revolution);

// Converts a CiA 402 position value to a shaft position (Rad) value.
//
// Args:
// - cia_402_position: Position in CiA402 units (encoder counts).
// - counts_per_revolution: Encoder counts per shaft revolution.
//
// Returns: Position in radians.
// Returns InvalidArgument in case of invalid counts per revolution.
pw::Result<float> Cia402PositionToPosition(int32_t cia_402_position,
                                           uint32_t counts_per_revolution);

// Converts a shaft velocity (Rad / s) value to CiA 402 velocity value.
//
// Args:
// - shaft_velocity: Shaft position in radians per second.
// - counts_per_revolution: Encoder counts per shaft revolution.
//
// Returns: Velocity in CiA402 units (encoder ticks per second).
// Returns InvalidArgument in case of an overflow or invalid counts per
// revolution.
pw::Result<int32_t> VelocityToCia402Velocity(float shaft_velocity,
                                             uint32_t counts_per_revolution);

// Converts a CiA 402 velocity value to a shaft position (Rad / s) value.
//
// Args:
// - cia_402_velocity: Velocity in CiA402 units (encoder counts per second).
// - counts_per_revolution: Encoder counts per shaft revolution.
//
// Returns: Velocity in radians per second.
// Returns InvalidArgument in case of invalid counts per revolution.
pw::Result<float> Cia402VelocityToVelocity(int32_t cia_402_velocity,
                                           uint32_t counts_per_revolution);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CANOPEN_UNIT_CONVERSIONS_H_
