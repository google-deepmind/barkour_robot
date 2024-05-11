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

#include "ethercat_host.h"

#include "canopen.h"
#include "absl/status/status.h"

namespace barkour {

absl::Status EthercatHost::Connect() { return absl::OkStatus(); }

absl::Status EthercatHost::Disconnect() { return absl::OkStatus(); }

absl::Status EthercatHost::ConfigurePdoTransfer() { return absl::OkStatus(); }

absl::Status EthercatHost::StartCyclicOperation() { return absl::OkStatus(); }

absl::Status EthercatHost::StopCyclicOperation() { return absl::OkStatus(); }

absl::Status EthercatHost::RegisterPdoEntry(
    const EthercatDeviceAddress& device_address,
    const CanObjectInfoVariant& object_info, const PdoDirection direction) {
  return RegisterPdoEntry(
      device_address, CanObjectAddressFromCanObjectInfo(object_info),
      CanObjectSizeFromCanObjectInfo(object_info), direction);
}

absl::StatusOr<const EthercatHost::PdoMemoryMap* const>
EthercatHost::GetPdoMemoryMap() const {
  return absl::UnimplementedError(
      "EtherCAT host does not support direct PDO memory map access.");
}

}  // namespace barkour
