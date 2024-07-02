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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_FOC_MATH_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_FOC_MATH_H_

// Contains math functions associated with Field-Oriented Control (FOC) of
// Brushless DC motors (BLDC).
//
// Sign and axis conventions consistent with STM MotorControlSDK.
//
// Transforms assume a+b+c=0 and use simplified 2D transforms
// where appropriate.

namespace barkour {

// Data types for transform results
struct FocAlphaBeta {
  float alpha;
  float beta;
};

struct FocDQ {
  float d;
  float q;
};

struct FocABC {
  float a;
  float b;
  float c;
};

// Applies the forward Clarke transformation.
//
// The Clarke transform is used in motor control to transform vectors
// representing quantities (a, b, c) in a balanced three-phase system into
// two quantities (alpha, beta) represented on two orthogonal axes.
//
// More information can be found at:
// - https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation
//
// Note: This is the magnitude invariant version of the Clarke transform.
//
// Args:
// - a: Quantity associated with phase a (voltage, current, etc).
// - b: Quantity associated with phase b (voltage, current, etc).
// - c: Quantity associated with phase b (voltage, current, etc).
//
// Returns the FocAlphaBeta:
// - alpha: alpha-axis component.
// - beta: beta-axis component.
FocAlphaBeta ClarkeTransform(float a, float b, float c);

// Applies the inverse Clarke transformation.
//
// See the forward transformation for further detail.
//
// Note: this the magnitude invariant version of the inverse Clarke transform.
//
// Args:
// - alpha: alpha-axis component.
// - beta: beta-axis component.
//
// Returns the FocABC:
// - a: Quantity associated with phase a (voltage, current, etc).
// - b: Quantity associated with phase b (voltage, current, etc).
// - c: Quantity associated with phase c (voltage, current, etc).
FocABC InverseClarkeTransform(float alpha, float beta);

// Applies the forward Park transformation.
//
// The park transformation is a rotation that projects two-dimensional vectors
// associated with the stator reference frame, onto the moving rotor reference
// frame.
//
// Args:
// - alpha: Stationary frame alpha component.
// - beta: Stationary frame beta component.
// - angle: Angle of rotation [Radians, electrical frame].
//
// Returns the FocDQ:
// - d: Rotating frame direct component.
// - q: Rotating frame quadrature component.
FocDQ ParkTransform(float alpha, float beta, float angle);

// Applies the inverse Park transformation.
//
// See the forward transformation for further detail.
//
// Args:
// - d: Rotating frame direct component.
// - q: Rotating frame quadrature component.
// - angle: Angle of rotation [Radians, electrical frame].
//
// Returns the FocAlphaBeta:
// - alpha: Stationary frame alpha component.
// - beta: Stationary frame beta component.
FocAlphaBeta InverseParkTransform(float d, float q, float angle);

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_FOC_MATH_H_
