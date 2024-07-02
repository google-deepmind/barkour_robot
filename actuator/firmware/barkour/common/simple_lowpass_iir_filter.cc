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
#include "pw_assert/check.h"
#include "pw_log/log.h"

namespace barkour {

SimpleLowpassIirFilter::SimpleLowpassIirFilter(float cutoff_frequency,
                                               float step_time)
    : filtered_value_(0) {
  PW_CHECK(cutoff_frequency > 0,
           "SimpleLowpassIirFilter cut-off frequency must be greater than "
           "zero, got: %f.",
           cutoff_frequency);

  PW_CHECK(
      step_time > 0,
      "SimpleLowpassIirFilter step time must be greater than zero, got: %f.",
      step_time);

  float nyquist_freq = 0.5f / step_time;

  PW_CHECK(cutoff_frequency <= nyquist_freq,
           "SimpleLowpassIirFilter cutoff frequency must be no more than the "
           "Nyquist frequency for reliable results, cutoff freq: %f, Nyquist "
           "freq: %f",
           cutoff_frequency, nyquist_freq);

  PW_LOG_DEBUG(
      "Creating a SimpleLowpassIirFilter with cutoff_frequency %f Hz, "
      "step time %f s.",
      cutoff_frequency, step_time);

  // See https://dsp.stackexchange.com/a/28314 for a derivation of this formula.
  float normalized_cutoff_omega = 2.0f * kPi * cutoff_frequency * step_time;
  float cos_omega = std::cos(normalized_cutoff_omega);
  decay_constant_ = 2.0f - cos_omega -
                    std::sqrt(cos_omega * cos_omega - 4.0f * cos_omega + 3.0f);
}

void SimpleLowpassIirFilter::ResetFilter(float initial_value) {
  filtered_value_ = initial_value;
}

void SimpleLowpassIirFilter::UpdateFilter(float new_value) {
  filtered_value_ =
      decay_constant_ * filtered_value_ + (1 - decay_constant_) * new_value;
}

float SimpleLowpassIirFilter::GetFilteredValue() const {
  return filtered_value_;
}

}  // namespace barkour
