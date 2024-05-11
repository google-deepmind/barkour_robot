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

#ifndef BARKOUR_ROBOT_HARDWARE_COMMON_DS402_OBJECTS_H_
#define BARKOUR_ROBOT_HARDWARE_COMMON_DS402_OBJECTS_H_

#include <array>
#include <cstdint>

#include "canopen.h"

namespace barkour {

// The following objects are described/defined in CiA 402.
// Please keep this sorted and don't forget to update the array at the end.

// Controlword - unsigned 16-bit int value.
inline constexpr CanObjectInfo<uint16_t> kControlwordObjectInfo(0x6040, 0);

// Statusword - unsigned 16-bit int value.
inline constexpr CanObjectInfo<uint16_t> kStatuswordObjectInfo(0x6041, 0);

// Modes of operation - signed 8-bit int value.
inline constexpr CanObjectInfo<int8_t> kModesObjectInfo(0x6060, 0);

// Modes of operation display - signed 8-bit int value.
inline constexpr CanObjectInfo<int8_t> kModesDisplayObjectInfo(0x6061, 0);

// Position actual value - signed 32-bit int value.
inline constexpr CanObjectInfo<int32_t> kPositionObjectInfo(0x6064, 0);

// Velocity actual value - signed 32-bit int value.
inline constexpr CanObjectInfo<int32_t> kVelocityObjectInfo(0x606c, 0);

// Target torque - signed 16-bit int value.
inline constexpr CanObjectInfo<int16_t> kTorqueSetpointObjectInfo(0x6071, 0);

// Maximum torque - unsigned 16-bit int value.
inline constexpr CanObjectInfo<uint16_t> kMaximumTorqueObjectInfo(0x6072, 0);

// Maximum current - unsigned 16-bit int value.
inline constexpr CanObjectInfo<uint16_t> kMaximumCurrentObjectInfo(0x6073, 0);

// Rated current - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kRatedCurrentObjectInfo(0x6075, 0);

// Rated torque - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kRatedTorqueObjectInfo(0x6076, 0);

// Torque actual value - signed 16-bit int value.
inline constexpr CanObjectInfo<int16_t> kTorqueObjectInfo(0x6077, 0);

// Target position - signed 32-bit int value.
inline constexpr CanObjectInfo<int32_t> kPositionSetpointObjectInfo(0x607A, 0);

// Homing offset - signed 32-bit int value.
inline constexpr CanObjectInfo<int32_t> kHomingOffsetObjectInfo(0x607C, 0);

// Position encoder increments - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kEncoderIncrementsObjectInfo(0x608F,
                                                                      1);

// Position encoder motor revolutions - unsigned 32-bit int value.
// Note: position encoder resolution (motor side) =
// kEncoderIncrementsObjectInfo / kEncoderMotorRevolutionsObjectInfo.
inline constexpr CanObjectInfo<uint32_t> kEncoderMotorRevolutionsObjectInfo(
    0x608F, 2);

// Gear ratio motor revolutions - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kGearMotorRevolutionsObjectInfo(0x6091,
                                                                         1);

// Gear ratio shaft revolutions - unsigned 32-bit int value.
// Note: gear ratio =
// kGearMotorRevolutionsObjectInfo / kGearShaftRevolutionssObjectInfo.
inline constexpr CanObjectInfo<uint32_t> kGearShaftRevolutionssObjectInfo(
    0x6091, 2);

// Target velocity -  signed 32-bit int value.
inline constexpr CanObjectInfo<int32_t> kVelocitySetpointObjectInfo(0x60FF, 0);

inline constexpr std::array<CanObjectInfoVariant, 19> kDs402Objects{
    kControlwordObjectInfo,
    kStatuswordObjectInfo,
    kModesObjectInfo,
    kModesDisplayObjectInfo,
    kPositionObjectInfo,
    kVelocityObjectInfo,
    kTorqueSetpointObjectInfo,
    kMaximumTorqueObjectInfo,
    kMaximumCurrentObjectInfo,
    kRatedCurrentObjectInfo,
    kRatedTorqueObjectInfo,
    kTorqueObjectInfo,
    kPositionSetpointObjectInfo,
    kHomingOffsetObjectInfo,
    kEncoderIncrementsObjectInfo,
    kEncoderMotorRevolutionsObjectInfo,
    kGearMotorRevolutionsObjectInfo,
    kGearShaftRevolutionssObjectInfo,
    kVelocitySetpointObjectInfo};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_COMMON_DS402_OBJECTS_H_
