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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CONSTANTS_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CONSTANTS_H_

#include <cstdint>

namespace barkour {

inline constexpr uint32_t kGoogleEthercatVendorId = 0x00600613;

// EtherCAT working counter increments.
//
// For details, see the Beckhoff EtherCAT Slave Controller Hardware Data Sheet,
// Section I - Technology, section 2.4 Working Counter.
constexpr inline int kEthercatWorkingCounterIncrementReadCommandFailure = 0;
constexpr inline int kEthercatWorkingCounterIncrementReadCommandSuccess = 1;

constexpr inline int kEthercatWorkingCounterIncrementWriteCommandFailure = 0;
constexpr inline int kEthercatWorkingCounterIncrementWriteCommandSuccess = 1;

constexpr inline int kEthercatWorkingCounterIncrementReadWriteCommandFailure =
    0;
constexpr inline int kEthercatWorkingCounterIncrementReadWriteCommandReadOnly =
    1;
constexpr inline int kEthercatWorkingCounterIncrementReadWriteCommandWriteOnly =
    2;
constexpr inline int kEthercatWorkingCounterIncrementReadWriteCommandSuccess =
    3;

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CONSTANTS_H_
