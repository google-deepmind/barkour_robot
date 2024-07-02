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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_IMU_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_IMU_H_

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

class FakeImu : public ImuInterface {
 private:
  ImuState imu_state_;
  pw::Status fake_status_;

 public:
  FakeImu()
      : imu_state_({{0, 0, 0}, {0, 0, 0}}), fake_status_(pw::OkStatus()) {}

  ~FakeImu() override = default;

  pw::Status Initialize() override { return pw::OkStatus(); }

  // Returns the current IMU data.
  pw::Result<ImuState> GetImuState() override {
    if (fake_status_ == pw::OkStatus()) {
      return imu_state_;
    }
    return fake_status_;
  }

  // Testing methods
  void setStatus(pw::Status status) { fake_status_ = status; };
  void setState(ImuState& state) { imu_state_ = state; };
};

};  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_IMU_H_
