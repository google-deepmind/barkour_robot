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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SIMPLE_LOWPASS_IIR_FILTER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SIMPLE_LOWPASS_IIR_FILTER_H_

// Implementation of a 1st-order IIR low-pass filter.

#include "actuator/firmware/barkour/common/interfaces/signal_filter_interface.h"

namespace barkour {

class SimpleLowpassIirFilter : public SignalFilterInterface {
 public:
  // Constructor.
  //
  // After construction, the filtered value will be zero.
  //
  // Args:
  // - cutoff_frequency: Cut-off frequency of the filter, in Hz. Frequencies
  //   above this will be attenuated by at least 3 dB, i.e. a factor of
  //   approximately 1 / sqrt(2).
  // - step_time: Time between successive calls to `UpdateFilter`, in seconds.
  //   It is the user's responsibility to ensure that these calls take place at
  //   the right time interval.
  //
  // Both cutoff_frequency and step_time must be greater than zero. This filter
  // is designed to work with cutoff frequencies smaller than the sampling
  // frequency, if this is not satisfied then unexpected results may occur.
  SimpleLowpassIirFilter(float cutoff_frequency, float step_time);

  // Resets the filter.
  void ResetFilter(float initial_value) override;

  // Updates the filtered value.
  void UpdateFilter(float new_value) override;

  // Gets the filtered value.
  float GetFilteredValue() const override;

 private:
  float decay_constant_;
  float filtered_value_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_SIMPLE_LOWPASS_IIR_FILTER_H_
