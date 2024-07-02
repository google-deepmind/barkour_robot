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

#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/common/trigonometry.h"

// If external C linkage is not specified, compilation may fail.
// This is a known problem in cmsis_core, and may be fixed in the future.
// Discussion here: https://github.com/ARM-software/CMSIS_5/issues/617
extern "C" {
#include "arm_math.h"  // NOLINT
}

namespace barkour {

// Note: ARM trig functions only operate on input angles from [0,2pi], so we
//   must normalize.
float sin(float x) {
  float x_normalized = fmod(x, kTwoPi);
  return arm_sin_f32(x_normalized);
}

float cos(float x) {
  float x_normalized = fmod(x, kTwoPi);
  return arm_cos_f32(x_normalized);
}

}  // namespace barkour
