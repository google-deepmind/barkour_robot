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

#include "types.h"

#include <string>
#include <type_traits>
#include <variant>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace barkour {

std::string EthercatDeviceAddressToString(
    const EthercatDeviceAddress& address) {
  if (std::holds_alternative<EthercatBusPosition>(address)) {
    return absl::StrFormat(
        "EthercatDeviceAddress[bus_position = %d]",
        static_cast<std::underlying_type<EthercatBusPosition>::type>(
            std::get<EthercatBusPosition>(address)));
  } else if (std::holds_alternative<EthercatStationAlias>(address)) {
    return absl::StrFormat(
        "EthercatDeviceAddress[station_alias = %d]",
        static_cast<std::underlying_type<EthercatStationAlias>::type>(
            std::get<EthercatStationAlias>(address)));
  } else {
    // Shouldn't usually get here, but it is possible for a variant to be
    // valueless.
    return "INVALID ETHERCAT PARTIAL DEVICE ADDRESS";
  }
}

bool EthercatDeviceInfo::operator==(const EthercatDeviceInfo& other) const {
  return (bus_position == other.bus_position &&
          station_alias == other.station_alias &&
          vendor_id == other.vendor_id && product_code == other.product_code &&
          serial_number == other.serial_number && name == other.name);
}

bool EthercatDeviceInfo::operator!=(const EthercatDeviceInfo& other) const {
  return !(*this == other);
}

std::string EthercatDeviceInfo::ToString() const {
  return absl::StrFormat(
      "EthercatDeviceInfo[bus_position=%u, station_alias=%u, vendor_id=%d, "
      "product_code=%d, serial_number=%d, name=%s]",
      bus_position, station_alias, vendor_id, product_code, serial_number,
      name);
}

std::string EthercatApplicationLayerState::ToString() const {
  std::string base_string;
  switch (base_state) {
    case BaseEthercatApplicationLayerState::kBoot:
      base_string = "BOOT";
      break;
    case BaseEthercatApplicationLayerState::kInit:
      base_string = "INIT";
      break;
    case BaseEthercatApplicationLayerState::kPreop:
      base_string = "PREOP";
      break;
    case BaseEthercatApplicationLayerState::kSafeop:
      base_string = "SAFEOP";
      break;
    case BaseEthercatApplicationLayerState::kOp:
      base_string = "OP";
      break;
    case BaseEthercatApplicationLayerState::kUnknown:
    default:
      base_string = "UNKNOWN";
  }

  if (error) {
    absl::StrAppend(&base_string, " + ERROR");
  }
  return base_string;
}

}  // namespace barkour
