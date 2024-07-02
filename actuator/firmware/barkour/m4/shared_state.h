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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_SHARED_STATE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_SHARED_STATE_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_span/span.h"
#include "pw_status/status.h"

namespace barkour {

// Shared state between multiple tasks or parts of the codebase.
enum class MotorDriverVariable : uint8_t {
  kTorque = 0,
  kVelocity = 1,
  kPosition = 2,
  kTargetTorque = 3,
  kTargetVelocity = 4,
  kTargetPosition = 5,
  kBlob = 6
};
constexpr uint8_t kNumMotorDriverStateDataVariables = 7;

struct MotorDriverStateData {
  int32_t torque_;
  int32_t velocity_;
  int32_t position_;
  int32_t target_torque_;
  int32_t target_velocity_;
  int32_t target_position_;

  // Example of binary data to be exchanged over EtherCAT.
  std::byte blob_[150];
};

constexpr int kMotorDriverVariableToMotorDriverStateDataOffset
    [kNumMotorDriverStateDataVariables] = {
        offsetof(MotorDriverStateData, torque_),
        offsetof(MotorDriverStateData, velocity_),
        offsetof(MotorDriverStateData, position_),
        offsetof(MotorDriverStateData, target_torque_),
        offsetof(MotorDriverStateData, target_velocity_),
        offsetof(MotorDriverStateData, target_position_),
        offsetof(MotorDriverStateData, blob_)};

constexpr uint8_t kMotorDriverVariableSize[kNumMotorDriverStateDataVariables] =
    {
        sizeof(MotorDriverStateData::torque_),
        sizeof(MotorDriverStateData::velocity_),
        sizeof(MotorDriverStateData::position_),
        sizeof(MotorDriverStateData::target_torque_),
        sizeof(MotorDriverStateData::target_velocity_),
        sizeof(MotorDriverStateData::target_position_),
        sizeof(MotorDriverStateData::blob_),
};

// The list of tasks that are allowed to write to the shared state.
enum class MotorDriverStateWriters : uint8_t {
  kEtherCat = 0,
  kMainThread = 1,
  kAdc = 2,
  kM7 = 3,
};
constexpr uint8_t kNumberOfStateWriters = 4;

class MotorDriverStateView {
 public:
  // Copies the contents of a variable from the shared state.
  template <typename T>
  pw::Status GetVariable(MotorDriverVariable variable, T& value) const {
    if (data_ == nullptr) {
      return pw::Status::Unavailable();
    }
    if (kMotorDriverVariableSize[static_cast<uint8_t>(variable)] != sizeof(T)) {
      PW_LOG_WARN(
          "GetVariable called with value of incorrect size: %d vs %d "
          "for variable %d.",
          sizeof(T), kMotorDriverVariableSize[static_cast<int>(variable)],
          static_cast<int>(variable));
      return pw::Status::InvalidArgument();
    }
    uint16_t offset =
        kMotorDriverVariableToMotorDriverStateDataOffset[static_cast<uint8_t>(
            variable)];
    memcpy(&value, ((std::byte*)(data_)) + offset,
           kMotorDriverVariableSize[static_cast<uint8_t>(variable)]);
    return pw::OkStatus();
  }

  // Provides direct read-only access to the shared state data structure.
  // This data is only valid if no call to Update() is in progress.
  const MotorDriverStateData& State() const { return *data_; }

 private:
  MotorDriverStateView(MotorDriverStateData* data) : data_(data) {}
  MotorDriverStateData* data_;
  friend class MotorDriverState;
};

class MotorDriverStateWriter {
 public:
  // Sets a variable to be updated during the next call to Update().
  template <typename T>
  pw::Status SetVariable(MotorDriverVariable variable, const T& value) {
    if (kMotorDriverVariableSize[static_cast<int>(variable)] != sizeof(T)) {
      PW_LOG_WARN(
          "SetVariable called with value of incorrect size: %d vs %d "
          "for variable %d.",
          sizeof(T), kMotorDriverVariableSize[static_cast<uint8_t>(variable)],
          static_cast<int>(variable));
      return pw::Status::InvalidArgument();
    }
    uint16_t offset =
        kMotorDriverVariableToMotorDriverStateDataOffset[static_cast<uint8_t>(
            variable)];
    void* data_value = ((std::byte*)(&data_)) + offset;
    memcpy(data_value, &value,
           kMotorDriverVariableSize[static_cast<uint8_t>(variable)]);
    dirty_flags_[static_cast<uint8_t>(variable)] = true;
    return pw::OkStatus();
  }

  // Tag the variable to be copied to the shared state during the next Update()
  // call.
  void SetDirtyFlag(MotorDriverVariable variable) {
    dirty_flags_[static_cast<uint8_t>(variable)] = true;
  }

  // Provides direct write-only access to the shared state data structure.
  // Data will only be valid until the next call to Update().
  // SetDirtyFlag must be called in the same cycle as writing to the shared data
  // structure, to ensure the data is copied to the shared state during the next
  // Update() call.
  MotorDriverStateData& State() { return data_; }

 private:
  MotorDriverStateWriter() { dirty_flags_.fill(false); }
  MotorDriverStateData data_;
  std::array<bool, kNumMotorDriverStateDataVariables> dirty_flags_;
  void ClearDirtyFlags() { dirty_flags_.fill(false); }
  friend class MotorDriverState;
};

// Keeps track of shared state between multiple writers and writers.
// This class assumes a synchronous update mechanism (think CANopen SYNC).
// During a cycle, all writers can write concurrently without locking.
// However, the shared state won't be reflected to the readers until the next
// call to Update(). At this point the writes from all writers are copied to the
// shared state. Care must be taken that no writes or reads are in progress
// during the call to Update(). For now, we use a simple implementation using
// flags to tag a variable as dirty. Alternative implementations are possible
// (e.g. using a queue of variables to be updated).
class MotorDriverState {
 public:
  // Singleton instance.
  static MotorDriverState& Get();

  // Returns a writer for the specific task.
  // Multiple writers can write concurrently to the shared state.
  pw::Result<MotorDriverStateWriter*> GetWriter(
      const MotorDriverStateWriters writer);

  // Returns a new reader.
  MotorDriverStateView GetReader();

  // Updates the shared state by processing pending writes from all writers and
  // ensuring they are reflected in the readers.
  // Args:
  // - fail_on_multiple_writes: If true, if multiple writers have pending writes
  // to the same memory location, this call will return pw::Status::DataLoss().
  pw::Status Update(bool fail_on_multiple_writes = false);

 private:
  MotorDriverState();
  MotorDriverStateData current_state_;
  MotorDriverStateWriter next_states_[kNumberOfStateWriters];
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_SHARED_STATE_H_
