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

#include "actuator/firmware/barkour/common/multiturn_encoder_wrapper.h"

#include <cstdint>
#include <cstdlib>
#include <limits>

#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "pw_log/log.h"
#include "pw_result/result.h"

namespace barkour {

MultiturnEncoderWrapper::MultiturnEncoderWrapper(RotaryEncoder& base_encoder)
    : base_encoder_(base_encoder),
      base_encoder_counts_per_turn_(base_encoder_.GetCountsPerTurn()) {
  if (base_encoder_counts_per_turn_ >
      static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
    PW_LOG_WARN(
        "MultiturnEncoderWrapper got a base encoder with counts per turn "
        "higher than INT32_MAX. This might result in incorrect calculations.");
  }
}

pw::Result<RotaryEncoderState> MultiturnEncoderWrapper::GetEncoderState() {
  return base_encoder_.GetEncoderState();
}

RotaryEncoderState& MultiturnEncoderWrapper::GetEncoderStateFromISR() {
  return base_encoder_.GetEncoderStateFromISR();
}

// Called from ISR.
pw::Result<RotaryEncoderState> MultiturnEncoderWrapper::Update() {
  pw::Result<RotaryEncoderState> result = base_encoder_.Update();
  if (!result.ok()) {
    return result;
  }

  // Get reference to actual base_encoder data structure so it can be updated.
  RotaryEncoderState& base_encoder_state = GetEncoderStateFromISR();

  if (!previous_encoder_counts_.has_value()) {
    previous_encoder_counts_ = base_encoder_state.raw_encoder_counts;
    base_encoder_state.multiturn_encoder_counts =
        previous_encoder_counts_.value();
  }

  int32_t previous_shaft_counts_normed =
      previous_encoder_counts_.value() % base_encoder_counts_per_turn_;

  int32_t displacement =
      base_encoder_state.raw_encoder_counts - previous_shaft_counts_normed;

  if (static_cast<uint32_t>(std::abs(displacement)) >
      base_encoder_counts_per_turn_ / 2) {
    if (displacement < 0) {
      // Positive-headed rollover occurred.
      displacement += base_encoder_counts_per_turn_;
    } else {
      // Negative-headed rollover occurred.
      displacement -= base_encoder_counts_per_turn_;
    }
  }
  int32_t new_shaft_counts = previous_encoder_counts_.value() + displacement;

  // if (!(std::abs(new_shaft_counts - previous_encoder_counts_.value()) <
  //      std::numeric_limits<int32_t>::max() / 2)) {
  // }

  previous_encoder_counts_ = new_shaft_counts;

  // Add multiturn information.
  base_encoder_state.multiturn_encoder_counts = new_shaft_counts;

  result = base_encoder_state;
  return result;
}

uint32_t MultiturnEncoderWrapper::GetCountsPerTurn() const {
  return base_encoder_counts_per_turn_;
}

}  // namespace barkour
