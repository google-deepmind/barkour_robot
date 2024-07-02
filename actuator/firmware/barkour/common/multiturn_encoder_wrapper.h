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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MULTITURN_ENCODER_WRAPPER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MULTITURN_ENCODER_WRAPPER_H_

#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "pw_result/result.h"

namespace barkour {

// Wrapper for a base RotaryEncoder instance, which unwraps the encoder readings
// of the base wrapper.
class MultiturnEncoderWrapper : public RotaryEncoder {
 public:
  explicit MultiturnEncoderWrapper(RotaryEncoder& base_encoder);

  // Returns the current encoder data. This also calls `GetEncoderState` on the
  // base encoder, and updates the internal state of this class.
  pw::Result<RotaryEncoderState> GetEncoderState() override;
  RotaryEncoderState& GetEncoderStateFromISR() override;

  pw::Result<RotaryEncoderState> Update() override;

  // Returns the same counts per turn as the base encoder.
  uint32_t GetCountsPerTurn() const override;

 private:
  // The base encoder to wrap.
  RotaryEncoder& base_encoder_;

  // Result of the `GetCountsPerTurn` method on the base encoder. Assumed to
  // stay constant.
  uint32_t base_encoder_counts_per_turn_;

  // The previous value of `multiturn_encoder_counts`, or nullopt
  // if `GetEncoderState` hasn't been called yet.
  std::optional<int32_t> previous_encoder_counts_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MULTITURN_ENCODER_WRAPPER_H_
