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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ROTARY_ENCODER_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ROTARY_ENCODER_INTERFACE_H_

#include <cstdint>
#include <optional>

#include "pw_result/result.h"

namespace barkour {

struct RotaryEncoderState {
  // Sensor position as provided directly by the encoder, represented as
  // encoder counts. Typically this would be the single turn position,
  // overflowing every GetCountsPerTurn()
  uint32_t raw_encoder_counts = 0;

  // Unwrapped (and possibly zero-calibrated) encoder counts, for encoders
  // instances which support this.
  std::optional<int32_t> multiturn_encoder_counts = std::nullopt;
};

class RotaryEncoder {
 public:
  virtual ~RotaryEncoder() = default;

  virtual pw::Result<RotaryEncoderState>
  Update() = 0;  // Read the actual encoder and update internal state.

  // Returns the current encoder data.
  virtual pw::Result<RotaryEncoderState> GetEncoderState() = 0;
  virtual RotaryEncoderState& GetEncoderStateFromISR() = 0;

  // Returns the number of encoder ticks per turn.
  virtual uint32_t GetCountsPerTurn() const = 0;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_ROTARY_ENCODER_INTERFACE_H_
