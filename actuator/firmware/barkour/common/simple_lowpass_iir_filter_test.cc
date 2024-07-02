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

#include "actuator/firmware/barkour/common/simple_lowpass_iir_filter.h"

#include <cmath>

#include "actuator/firmware/barkour/common/math_constants.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr float kStepTime = 1.0e-2;

TEST(SimpleLowpassIirFilterTest, FilterValueIsCorrectAfterReset) {
  SimpleLowpassIirFilter filter(50.0, kStepTime);

  // Update with some arbitrary values before resetting.
  for (int i = 0; i < 100; ++i) {
    filter.UpdateFilter(i / 100);
  }

  float reset_value = 463.5f;
  filter.ResetFilter(reset_value);

  EXPECT_EQ(filter.GetFilteredValue(), reset_value);
}

TEST(SimpleLowpassIirFilterTest, FilterValueDecaysToConstantInput) {
  float cutoff_frequency = 0.1f / kStepTime;
  SimpleLowpassIirFilter filter(cutoff_frequency, kStepTime);
  filter.ResetFilter(10.0f);

  // After ~10x the time period of the cutoff frequency, the filter should have
  // settled to roughly the constant input value.
  float constant_update_value = 1.0f;
  for (float time = 0.0f; time < 10.0f / cutoff_frequency; time += kStepTime) {
    filter.UpdateFilter(constant_update_value);
  }

  EXPECT_LE(std::fabs(filter.GetFilteredValue() - constant_update_value),
            1.0e-2);
}

TEST(SimpleLowpassIirFilterTest, SignalAtCutoffFrequencyIsAttenuatedBy3db) {
  float cutoff_frequency = 10.0f;
  SimpleLowpassIirFilter filter(cutoff_frequency, kStepTime);
  filter.ResetFilter(0.0f);

  // The running sum of the square of the output values;
  float sum_square = 0.0f;

  for (int i = 0; i < 100; ++i) {
    filter.UpdateFilter(std::sin(kTwoPi * cutoff_frequency * i / 100));
    float filter_output = filter.GetFilteredValue();
    sum_square += filter_output * filter_output;
  }

  // The RMS value of the unfiltered readings is 1 / sqrt 2, so the RMS value of
  // the filtered readings should be ~0.5.
  float rms = std::sqrt(sum_square / 100.0f);
  EXPECT_LE(std::fabs(rms - 0.5f), 0.02f);
}

}  // namespace
}  // namespace barkour
