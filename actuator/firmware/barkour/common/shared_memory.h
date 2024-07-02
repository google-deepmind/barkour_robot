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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SHARED_MEMORY_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SHARED_MEMORY_H_

// Defines for shared memory addresses

#include <cstdint>

namespace barkour {

// Start of SRAM4, recommended to be used for shared memory.
#define SHARED_MEMORY_START_ADDR 0x38000000
#define SHARED_MEMORY_END_ADDR 0x3800FFFF

constexpr inline uint32_t kSharedMemoryStartAddress = 0x38000000;
constexpr inline uint32_t kSharedMemoryEndAddress = 0x3800FFFF;

// The shared addresses of the 32-bit words of the unique chip ID.
constexpr inline uint32_t kUidWord0Address = kSharedMemoryStartAddress;
constexpr inline uint32_t kUidWord1Address = kUidWord0Address + 0x04;
constexpr inline uint32_t kUidWord2Address = kUidWord1Address + 0x04;

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SHARED_MEMORY_H_
