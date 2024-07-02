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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_TESTING_FAKE_IMU_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_TESTING_FAKE_IMU_H_

#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

class FakeImu : public ImuInterface {
 public:
  FakeImu();
  ~FakeImu() override;

  // Initializes the fake IMU.
  //
  // Must be called to calling GetImuState().
  pw::Status Initialize() override;

  // Deinitializes the fake IMU.
  pw::Status Deinitialize() override;

  // Returns pw::Status::FailedPrecondition() if the IMU is not initialized.
  pw::Result<ImuState> GetImuState() override;

  // Sets the current IMU state.
  void set_state(ImuState state) { current_imu_state_ = state; }

  // Force a specific status on the next GetImuState
  void set_status(pw::Status status) { next_status_ = status; }

 private:
  bool initialized_;
  ImuState current_imu_state_;
  pw::Status next_status_;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DEVICES_IMU_TESTING_FAKE_IMU_H_
