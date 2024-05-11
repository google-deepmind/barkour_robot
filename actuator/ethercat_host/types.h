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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_TYPES_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_TYPES_H_

#include <cstdint>
#include <string>
#include <variant>

namespace barkour {

// Each EtherCAT device on the bus has a position (determined by the physical
// position on the bus) and an optional "station alias", which is written into
// persistent memory, and used for logical addressing of the devices.
//
// Each is represented by a uint16 value, with an alias value of zero being
// equivalent to no alias.
//
// When requesting a specific device from the host side, either the bus position
// *or* the station alias may be used, but not both.
//
// Note: We use enum objects to avoid unchecked arithmetic operations on these
// types.
enum class EthercatBusPosition : uint16_t {};
enum class EthercatStationAlias : uint16_t {};

// Represents an address of an EtherCAT device, i.e. either the physical
// position or the configured station alias.
//
// This is designed to be used to request a specific device, e.g. for PDO
// communications or configuring device parameters.
typedef std::variant<EthercatBusPosition, EthercatStationAlias>
    EthercatDeviceAddress;

// Returns a string representation of an EthercatDeviceAddress.
std::string EthercatDeviceAddressToString(
    const EthercatDeviceAddress& device_address);

// Information about an EtherCAT device, which can be read over the bus.
struct EthercatDeviceInfo {
  EthercatBusPosition bus_position;
  EthercatStationAlias station_alias;
  uint32_t vendor_id;
  uint32_t product_code;
  uint32_t serial_number;
  std::string name;

  std::string ToString() const;

  bool operator==(const EthercatDeviceInfo& other) const;
  bool operator!=(const EthercatDeviceInfo& other) const;
};

// The EtherCAT application layer state is made up of two parts, the base state,
// and an error flag.
enum class BaseEthercatApplicationLayerState {
  kUnknown,
  kBoot,
  kInit,
  kPreop,
  kSafeop,
  kOp,
};

struct EthercatApplicationLayerState {
  BaseEthercatApplicationLayerState base_state;
  bool error;

  std::string ToString() const;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_TYPES_H_
