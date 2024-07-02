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

#include "actuator/firmware/barkour/m4/shared_state.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_span/span.h"
#include "pw_status/status.h"

namespace barkour {

MotorDriverState::MotorDriverState() {
  for (uint8_t i = 0; i < kNumberOfStateWriters; ++i) {
    next_states_[i] = MotorDriverStateWriter();
  }
}

pw::Status MotorDriverState::Update(bool fail_on_multiple_writes) {
  {
    std::array<uint8_t, kNumMotorDriverStateDataVariables> dirty_flags;
    dirty_flags.fill(0);

    for (uint8_t i = 0; i < kNumberOfStateWriters; ++i) {
      auto& new_data = next_states_[i].data_;
      for (uint8_t j = 0; j < kNumMotorDriverStateDataVariables; ++j) {
        if (next_states_[i].dirty_flags_[j]) {
          size_t offset = kMotorDriverVariableToMotorDriverStateDataOffset[j];
          void* data_to = (((std::byte*)(&current_state_))) + offset;
          const void* data_from = (((std::byte*)(&new_data))) + offset;
          memcpy(data_to, data_from, kMotorDriverVariableSize[j]);
          ++dirty_flags[j];
        }
      }
      next_states_[i].ClearDirtyFlags();
    }

    pw::Status result = pw::OkStatus();
    if (fail_on_multiple_writes) {
      for (uint8_t i = 0; i < kNumMotorDriverStateDataVariables; ++i) {
        if (dirty_flags[i] > 1) {
          PW_LOG_ERROR(
              "Motor driver state variable %d was updated by multiple "
              "tasks (%d).",
              i, dirty_flags[i]);
          result = pw::Status::DataLoss();
        }
      }
    }
    return result;
  }
}

MotorDriverStateView MotorDriverState::GetReader() {
  return MotorDriverStateView(&current_state_);
}

pw::Result<MotorDriverStateWriter*> MotorDriverState::GetWriter(
    const MotorDriverStateWriters writer) {
  MotorDriverStateWriter* result = &next_states_[static_cast<int>(writer)];
  return pw::Result<MotorDriverStateWriter*>(result);
}

MotorDriverState& MotorDriverState::Get() {
  static MotorDriverState shared_state;
  return shared_state;
}

}  // namespace barkour
