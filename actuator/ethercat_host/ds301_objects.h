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

#ifndef BARKOUR_ROBOT_HARDWARE_COMMON_DS301_OBJECTS_H_
#define BARKOUR_ROBOT_HARDWARE_COMMON_DS301_OBJECTS_H_

#include <array>
#include <cstdint>

#include "canopen.h"

namespace barkour {

// The following objects are described/defined in DS 301.
// Please keep this sorted and don't forget to update the array at the end.
//
// Manufacturer status register - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kManufacturerStatusRegisterObjectInfo(
    0x1002, 0);

// Serial number - unsigned 32-bit int value.
inline constexpr CanObjectInfo<uint32_t> kSerialNumberObjectInfo(0x1018, 4);

inline constexpr std::array<CanObjectInfoVariant, 2> kDs301Objects{
    kManufacturerStatusRegisterObjectInfo, kSerialNumberObjectInfo};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_COMMON_DS301_OBJECTS_H_
