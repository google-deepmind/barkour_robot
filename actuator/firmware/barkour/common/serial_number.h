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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SERIAL_NUMBER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SERIAL_NUMBER_H_

#include <cstdint>

namespace barkour {

// Returns a 64-bit serial number for the current device. This is the first 64
// bits of a SHA256 hash of the 96-bit Unique Device ID provided by STM.
//
// The first call to this function may be slow. Subsequent calls will return a
// cached value, and so will be much faster.
uint64_t Get64BitSerialNumber();

// Returns a 32-bit serial number for the current device. This is the first 32
// bits of a SHA256 hash of the 96-bit Unique Device ID provided by STM.
//
// Note this is equal to the first (i.e. most significant) 32 bits of the return
// value of Get64BitSerialNumber.
uint32_t Get32BitSerialNumber();

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SERIAL_NUMBER_H_
