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

// Contains constant data on the PDOs used in the EtherLab EtherCAT host.
//
// The configurations exposed here are based on the results of running the
// `ethercat cstruct` command from the EtherLab command line tool.
#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_DEVICE_DATA_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_DEVICE_DATA_H_

#include "canopen.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "etherlab/include/ecrt.h"

namespace barkour {

// Spans to the device data arrays.
//
// These are spans rather than raw arrays to avoid exposing the underlying
// mutable arrays directly.
//
// WARNING: The `ec_pdo_entry_info_t*` elements of `kDevicePdos` are const
// pointers but not pointers to const, as required by EtherLab. This means it is
// possible to modify the underlying `ec_pdo_entry_info_t` structs, which  will
// cause the elements of `kDevicePdoEntries` to be changed. Please take care to
// avoid doing this!
extern absl::Span<const ec_pdo_entry_info_t> kEtherlabDevicePdoEntries;
extern absl::Span<const ec_pdo_info_t> kEtherlabDevicePdos;
extern absl::Span<const ec_sync_info_t> kEtherlabDeviceSyncs;

// Returns the length of the PDO at the given address in
// `kEtherlabDevicePdoEntries`, in bytes.
//
// If there is no such PDO, return a not found error.
//
// If the passed address has both index and subindex zero, returns an invalid
// argument error.
absl::StatusOr<int> GetEtherlabPdoByteLength(
    const CanObjectAddress& object_address);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_DEVICE_DATA_H_
