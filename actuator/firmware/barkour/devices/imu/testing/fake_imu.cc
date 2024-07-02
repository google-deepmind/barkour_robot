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

#include "actuator/firmware/barkour/devices/imu/testing/fake_imu.h"

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

FakeImu::FakeImu() : initialized_(false), next_status_(pw::OkStatus()) {}
FakeImu::~FakeImu() { initialized_ = false; }

pw::Status FakeImu::Initialize() {
  if (!initialized_) {
    initialized_ = true;
  }
  return pw::OkStatus();
}

pw::Result<ImuState> FakeImu::GetImuState() {
  if (!initialized_) {
    return pw::Status::FailedPrecondition();
  } else if (next_status_ != pw::OkStatus()) {
    pw::Status return_status = next_status_;
    next_status_ = pw::OkStatus();
    return return_status;
  }
  return current_imu_state_;
}

pw::Status FakeImu::Deinitialize() {
  initialized_ = false;
  return pw::OkStatus();
}

}  // namespace barkour
