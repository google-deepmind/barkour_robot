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

#include "actuator/firmware/barkour/common/commutation.h"

#include <algorithm>

#include "actuator/firmware/barkour/common/foc_math.h"

namespace barkour {

ThreePhasePwmCommands SinusoidalCommutation::DoCommutation(
    float u_q, float u_d, float electrical_angle, float bus_voltage) {
  // Use the inverse Park transformation to convert vectors in the rotor
  // reference frame (which is moving), to the stator reference frame, which
  // is stationary. The Park transformation is simply a 2d rotation.
  FocAlphaBeta alphabeta = InverseParkTransform(u_d, u_q, electrical_angle);

  // Now use the inverse Clarke transformation to convert vectors in the
  // stator alpha-beta frame to phase voltages.
  // Note: This is the magnitude invariant (not power invariant) version of
  //   the transformation, which means that the vector magnitudes do not
  //   change: |u_a| = |u_alpha|
  FocABC abc = InverseClarkeTransform(alphabeta.alpha, alphabeta.beta);

  // Compute SVPWM
  float u_min;
  float u_max;

  if (abc.a > abc.b) {
    u_min = abc.b;
    u_max = abc.a;
  } else {
    u_min = abc.a;
    u_max = abc.b;
  }

  if (abc.c > u_max) {
    u_max = abc.c;
  } else if (abc.c < u_min) {
    u_min = abc.c;
  }

  float com = (u_min + u_max) / 2.0f;

  abc.a -= com;
  abc.b -= com;
  abc.c -= com;

  // Voltages are all relative, so we adjust our reference such that pwm
  // values are positive.
  float scale = bus_voltage / 2.0f;
  float ua_shifted = abc.a / scale + 1.0;
  float ub_shifted = abc.b / scale + 1.0;
  float uc_shifted = abc.c / scale + 1.0;

  return {.phase_a = std::clamp<float>(ua_shifted / 2.0f, 0.0f, 1.0f),
          .phase_b = std::clamp<float>(ub_shifted / 2.0f, 0.0f, 1.0f),
          .phase_c = std::clamp<float>(uc_shifted / 2.0f, 0.0f, 1.0f)};
}

}  // namespace barkour
