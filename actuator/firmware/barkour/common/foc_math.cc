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

#include "actuator/firmware/barkour/common/foc_math.h"

#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/common/trigonometry.h"

namespace barkour {

// Implement standard FOC transforms.
// Sign and axis conventions consistent with STM MotorControlSDK.
//
// Transforms assume a+b+c=0 and use simplified 2D transforms
// where appropriate.

FocAlphaBeta ClarkeTransform(float a, float b, float /* c unused */) {
  constexpr float oneOverSqrt3 = -1.0f / kSqrt3;

  return FocAlphaBeta({.alpha = a, .beta = oneOverSqrt3 * (a + b + b)});
}

FocABC InverseClarkeTransform(float alpha, float beta) {
  float sqrt3beta = kSqrt3 * beta;

  float a = alpha;
  float b = (-alpha - sqrt3beta) * 0.5f;
  float c = (-alpha + sqrt3beta) * 0.5f;
  return FocABC({.a = a, .b = b, .c = c});
}

FocDQ ParkTransform(float alpha, float beta, float angle) {
  float cos_el_angle = cos(angle);
  float sin_el_angle = sin(angle);
  float d = sin_el_angle * alpha + cos_el_angle * beta;
  float q = cos_el_angle * alpha - sin_el_angle * beta;
  return FocDQ({.d = d, .q = q});
}

FocAlphaBeta InverseParkTransform(float d, float q, float angle) {
  float cos_el_angle = cos(angle);
  float sin_el_angle = sin(angle);
  float alpha = sin_el_angle * d + cos_el_angle * q;
  float beta = cos_el_angle * d - sin_el_angle * q;
  return FocAlphaBeta({.alpha = alpha, .beta = beta});
}

}  // namespace barkour.
