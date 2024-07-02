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

#include <array>
#include <cmath>

#include "actuator/firmware/barkour/common/math_constants.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {

namespace {

constexpr float kFloatTolerance = 1.0e-3;

// Fills up arr with linearly spaced values from [a, b].
template <class T, int N>
void linear_space(std::array<T, N>& arr, T a, T b) {
  for (std::size_t i = 0; i < arr.size(); i++) {
    arr[i] = a + ((b - a) * i) / (arr.size() - 1);
  }
  arr[arr.size() - 1] = b;
}

TEST(FocMathTest, ClarkeTransformIntuitiveValuesCorrectness) {
  FocAlphaBeta ab = ClarkeTransform(1.0f, -0.50f, -0.50f);
  float alpha_expected = 1.0f;
  float beta_expected = 0.0f;
  EXPECT_LE(std::fabs(ab.alpha - alpha_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(ab.beta - beta_expected), kFloatTolerance);

  FocAlphaBeta ab2 = ClarkeTransform(0.0, kSqrt3 / 2.0f, -1.0f * kSqrt3 / 2.0f);
  alpha_expected = 0.0f;
  beta_expected = -1.0f;
  EXPECT_LE(std::fabs(ab2.alpha - alpha_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(ab2.beta - beta_expected), kFloatTolerance);
}

TEST(FocMathTest, ClarkeTransformPhaseAndMagnitudeCorrectness) {
  constexpr int kNumSamples = 100;
  std::array<float, kNumSamples> theta;

  // 100 evenly space samples between 0 and 2Pi.
  linear_space<float, kNumSamples>(theta, 0.0f, 2 * kPi);

  // Three evenly-shifted phase signals.
  for (int i = 0; i < kNumSamples; i++) {
    float a = std::cos(theta[i]);
    float b = std::cos(theta[i] - 2.0f * kPi / 3.0f);
    float c = std::cos(theta[i] + 2.0f * kPi / 3.0f);
    FocAlphaBeta ab = ClarkeTransform(a, b, c);

    // Magnitude and phase of alpha should be aligned with a.
    EXPECT_LE(std::fabs(ab.alpha - a), kFloatTolerance);

    // beta should be leading alpha by 90 degrees.
    float beta_expected = std::cos(theta[i] + kPi / 2.0f);
    EXPECT_LE(std::fabs(ab.beta - beta_expected), kFloatTolerance);
  }
}

TEST(FocMathTest, InverseClarkeTransformIntuitive) {
  FocABC abc = InverseClarkeTransform(1.0f, 0.0f);
  float a_expected = 1.0f;
  float b_expected = -0.5f;
  float c_expected = -0.5f;
  EXPECT_LE(std::fabs(abc.a - a_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(abc.b - b_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(abc.c - c_expected), kFloatTolerance);

  a_expected = 0.0f;
  b_expected = -kSqrt3 / 2.0f;
  c_expected = kSqrt3 / 2.0f;
  FocABC abc2 = InverseClarkeTransform(0.0f, 1.0f);
  EXPECT_LE(std::fabs(abc2.a - a_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(abc2.b - b_expected), kFloatTolerance);
  EXPECT_LE(std::fabs(abc2.c - c_expected), kFloatTolerance);
}

TEST(FocMathTest, InverseClarkeTransformPhaseAndMagnitudeCorrectness) {
  constexpr int kNumSamples = 100;
  std::array<float, kNumSamples> theta;

  // 100 evenly space samples between 0 and 2Pi.
  linear_space<float, kNumSamples>(theta, 0.0f, 2 * kPi);

  // Three evenly-shifted phase signals.
  for (int i = 0; i < kNumSamples; i++) {
    // Sweep an alpha-beta vector of magnitude of 1 around the unit circle.
    float alpha = std::cos(theta[i]);
    float beta = std::sin(theta[i]);

    float a_expected = std::cos(theta[i]);
    FocABC abc = InverseClarkeTransform(alpha, beta);

    // Magnitude and phase should be maintained.
    EXPECT_LE(std::fabs(abc.a - a_expected), kFloatTolerance);

    // Note: we do not compare the magnitude and phase of b and c to that of the
    // initial waveform, as they are not expected to be the same, only that
    // their sum should be zero. This is a somewhat surprising property of the
    // magnitude-invariant version of the Clarke transform.
    float sum = abc.a + abc.b + abc.c;
    EXPECT_LE(sum, kFloatTolerance);
  }
}

TEST(FocMathTest, ParkAndInverseParkAreCorrect) {
  // Select 8 points around the unit circle.
  constexpr int kNumSamples = 8;

  // Choose a starting vector, which will be rotated around the unit circle;
  float alpha = 0.0f;
  float beta = 0.0f;

  std::array<float, kNumSamples> theta;

  linear_space<float, kNumSamples>(theta, 0.0f, 2 * kPi);

  for (float i = 0; i < kNumSamples; i++) {
    float angle_to_rotate = theta[i];

    // After rotating it by kRadiansToRotate, we should end up with the next
    // vector in the test series.
    float d_expected = 0.0f;
    float q_expected = 1.0f;

    alpha = std::cos(angle_to_rotate);
    beta = -std::sin(angle_to_rotate);

    FocDQ dq = ParkTransform(alpha, beta, angle_to_rotate);

    EXPECT_LE(std::fabs(dq.d - d_expected), kFloatTolerance);
    EXPECT_LE(std::fabs(dq.q - q_expected), kFloatTolerance);

    // Applying the inverse transform should yield the original alpha and beta.
    FocAlphaBeta ab = InverseParkTransform(dq.d, dq.q, angle_to_rotate);

    EXPECT_LE(std::fabs(ab.alpha - alpha), kFloatTolerance);
    EXPECT_LE(std::fabs(ab.beta - beta), kFloatTolerance);
  }
}

}  // anonymous namespace

}  // namespace barkour
