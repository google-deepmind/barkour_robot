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

#ifndef BARKOUR_ROBOT_HARDWARE_COMMON_CANOPEN_H_
#define BARKOUR_ROBOT_HARDWARE_COMMON_CANOPEN_H_

#include <cstdint>
#include <string>
#include <utility>
#include <variant>

#include "absl/status/statusor.h"

namespace barkour {

using CanDeviceIdType = uint8_t;
using CanObjectIndexType = uint16_t;
using CanObjectSubindexType = uint8_t;

struct CanObjectAddress {
  explicit constexpr CanObjectAddress(CanObjectIndexType idx,
                                      CanObjectSubindexType subidx)
      : index(idx), subindex(subidx) {}

  // Hash function to allow use of this struct as a key in an
  // absl::flat_hash_map.
  template <typename H>
  friend H AbslHashValue(H h, const CanObjectAddress& full_index) {
    return H::combine(std::move(h), full_index.index, full_index.subindex);
  }

  std::string ToString() const;

  bool operator==(const CanObjectAddress& other) const;
  bool operator!=(const CanObjectAddress& other) const;

  CanObjectIndexType index;
  CanObjectSubindexType subindex;
};

struct CanObjectAddressWithId {
  explicit constexpr CanObjectAddressWithId(CanDeviceIdType device_id,
                                            CanObjectIndexType idx,
                                            CanObjectSubindexType subidx)
      : id(device_id), index(idx), subindex(subidx) {}

  // Hash function to allow use of this struct as a key in an
  // absl::flat_hash_map.
  template <typename H>
  friend H AbslHashValue(H h, const CanObjectAddressWithId& full_index) {
    return H::combine(std::move(h), full_index.id, full_index.index,
                      full_index.subindex);
  }

  bool operator==(const CanObjectAddressWithId& other) const;
  bool operator!=(const CanObjectAddressWithId& other) const;

  std::string ToString() const;

  CanDeviceIdType id;
  CanObjectIndexType index;
  CanObjectSubindexType subindex;
};

// Contains the address and type information for a CAN object.
template <typename T>
struct CanObjectInfo {
  using ObjectType = T;

  explicit constexpr CanObjectInfo(CanObjectIndexType idx,
                                   CanObjectSubindexType subidx)
      : address(idx, subidx) {}

  // The size of the object, in bytes.
  static constexpr int size() { return sizeof(T); }

  // Convenience method to cast a value to the specific type for this object.
  // Useful to initialize default values for PDOs or SDOs.
  template <typename V>
  static constexpr T Cast(const V& value) {
    return static_cast<T>(value);
  }

  // Convenience method to cast a value contained in a variant to the specific
  // type for this object.
  // Useful to initialize default values for PDOs or SDOs.
  // Returns InvalidArgumentError if the type of value doesn't match T.
  template <typename... V>
  static constexpr absl::StatusOr<T> ValueFromVariant(
      const std::variant<V...>& value) {
    if (!std::holds_alternative<T>(value)) {
      return absl::InvalidArgumentError(
          "Variant doesn't contain the correct type.");
    }
    return std::get<T>(value);
  }

  CanObjectAddress address;
};

// A variant containing all potential types of CanObjectInfo objects.
typedef std::variant<
    CanObjectInfo<int8_t>, CanObjectInfo<int16_t>, CanObjectInfo<int32_t>,
    CanObjectInfo<int64_t>, CanObjectInfo<uint8_t>, CanObjectInfo<uint16_t>,
    CanObjectInfo<uint32_t>, CanObjectInfo<uint64_t>, CanObjectInfo<float>>
    CanObjectInfoVariant;

// A variant containing all potential types of CAN objects.
typedef std::variant<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t,
                     int64_t, uint64_t, float>
    CanObjectValueVariant;

// Syntactic sugar for CanObjectInfo objects.
template <typename T>
const CanObjectAddress& CanObjectAddressFromCanObjectInfo(
    const CanObjectInfo<T>& object_info) {
  return object_info.address;
}

template <typename T>
int CanObjectSizeFromCanObjectInfo(
    const CanObjectInfo<T>& object_info) {
  return object_info.size();
}

const CanObjectAddress& CanObjectAddressFromCanObjectInfo(
    const CanObjectInfoVariant& object_info);

int CanObjectSizeFromCanObjectInfo(
    const CanObjectInfoVariant& object_info);

// Defines the direction of a PDO for CANopen.
// Data is transferred cyclicly from the host to the device (RX) or from the
// device to the host (TX).
enum class PdoDirection {
  kRxHostToDevice,
  kTxDeviceToHost
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_COMMON_CANOPEN_H_
