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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_IMU_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_IMU_INTERFACE_H_

#include <array>
#include <cstdint>

#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// Enumeration of the IMU axes.
typedef enum : uint8_t {
  kImuX = 0,
  kImuY,
  kImuZ,
  kImuNumOfAxes,
} ImuAxis;

struct ImuState {
  // Accelerometer readings (X, Y, Z), in meters/second^2.
  std::array<float, ImuAxis::kImuNumOfAxes> accelerometer;

  // Gyroscope readings (X, Y, Z), in radians/second.
  std::array<float, ImuAxis::kImuNumOfAxes> gyroscope;
};

class ImuInterface {
 public:
  virtual ~ImuInterface() = default;

  virtual pw::Status Initialize() = 0;

  virtual pw::Status Deinitialize() = 0;

  // Returns the current IMU data.
  virtual pw::Result<ImuState> GetImuState() = 0;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_IMU_INTERFACE_H_
