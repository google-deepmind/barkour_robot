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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ROTARY_ENCODER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ROTARY_ENCODER_H_

#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

class FakeRotaryEncoder : public RotaryEncoder {
 public:
  explicit FakeRotaryEncoder(uint32_t counts_per_turn)
      : counts_per_turn_(counts_per_turn) {
    enc_state_.raw_encoder_counts = 0;
  }

  // Read the actual encoder and update internal state.
  pw::Result<RotaryEncoderState> Update() override { return GetEncoderState(); }

  RotaryEncoderState& GetEncoderStateFromISR() override {
    if (multiturn_encoder_counts_.has_value()) {
      enc_state_.multiturn_encoder_counts = multiturn_encoder_counts_.value();
    }
    return enc_state_;
  }

  pw::Result<RotaryEncoderState> GetEncoderState() override {
    PW_TRY(return_status_);
    if (multiturn_encoder_counts_.has_value()) {
      enc_state_.multiturn_encoder_counts = multiturn_encoder_counts_.value();
    }
    return enc_state_;
  }

  uint32_t GetCountsPerTurn() const override { return counts_per_turn_; }

  void SetRawEncoderCounts(uint32_t count) {
    enc_state_.raw_encoder_counts = count;
  }

  void SetEncoderStateReturnStatus(pw::Status return_status) {
    return_status_ = return_status;
  }

  uint32_t counts_per_turn_;
  std::optional<uint32_t> multiturn_encoder_counts_;

  RotaryEncoderState enc_state_;
  pw::Status return_status_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ROTARY_ENCODER_H_
