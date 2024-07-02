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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_TRIGONOMETRY_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_TRIGONOMETRY_H_

// Header for 32-bit float implementations of trigonometric functions. This is
// used as a facade to allow ARM-specific optimized implementations on the
// device, and standard library versions on the host.

namespace barkour {

// Trigonometric functions, with inputs in radians.
float sin(float x);
float cos(float x);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_TRIGONOMETRY_H_
