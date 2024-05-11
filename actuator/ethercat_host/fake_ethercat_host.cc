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

#include "fake_ethercat_host.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/log/log.h"
#include "absl/types/span.h"
#include "canopen.h"
#include "ds301_objects.h"
#include "ds402_objects.h"
#include "constants.h"
#include "custom_objects.h"
#include "ethercat_host.h"
#include "types.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace barkour {

namespace {

absl::StatusOr<const EthercatDeviceInfo> EthercatDeviceAddressToDeviceInfo(
    const EthercatDeviceAddress& device_address,
    absl::Span<const EthercatDeviceInfo> devices) {
  struct AddressVisitor {
    absl::StatusOr<const EthercatDeviceInfo> operator()(
        EthercatBusPosition bus_position) {
      for (const EthercatDeviceInfo& device : devices) {
        if (device.bus_position == bus_position) return device;
      }
      return absl::NotFoundError("Device lookup failed.");
    }

    absl::StatusOr<const EthercatDeviceInfo> operator()(
        EthercatStationAlias device_alias) {
      for (const EthercatDeviceInfo& device : devices) {
        if (device.station_alias == device_alias) return device;
      }
      return absl::NotFoundError("Device lookup failed.");
    }
    absl::Span<const EthercatDeviceInfo> devices;
  };
  return std::visit(AddressVisitor{devices}, device_address);
}

// Note: Serial number object handled separately.
// Make sure to explicitly specify the type of each entry as this is checked.
constexpr std::pair<CanObjectInfoVariant, CanObjectValueVariant>
    kSdoDefaultValues[] = {
        {kRatedCurrentObjectInfo, uint32_t{50000}},
        {kRatedTorqueObjectInfo, uint32_t{18000}},
        {kEncoderIncrementsObjectInfo, uint32_t{65536}},
        {kEncoderMotorRevolutionsObjectInfo, uint32_t{1}},
        {kGearMotorRevolutionsObjectInfo, uint32_t{5}},
        {kGearShaftRevolutionssObjectInfo, uint32_t{1}},
        {kJointAngleOffsetCompletedObjectInfo, uint8_t{1}},
        // Force device into operation enabled mode
        {kStatuswordObjectInfo, uint16_t{0b0010'0111}},
};

}  // namespace

FakeEthercatHost::FakeEthercatHost()
    : connected_(false),
      pdos_configured_(false),
      started_(false),
      working_counter_(0) {}

absl::Status FakeEthercatHost::Connect() {
  if (connected_) {
    return absl::FailedPreconditionError("Connection already made.");
  }
  connected_ = true;
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::Disconnect() {
  if (started_) {
    return absl::FailedPreconditionError(
        "Cannot disconnect while cyclic operation is in process.");
  }
  pdos_configured_ = false;
  connected_ = false;
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::ConfigurePdoTransfer() {
  if (pdos_configured_) {
    return absl::FailedPreconditionError(
        "PDOs are already configured, cannot configure again.");
  }
  pdos_configured_ = true;
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::StartCyclicOperation() {
  if (!pdos_configured_) {
    return absl::FailedPreconditionError(
        "PDOs are not configured, cannot start cyclic operation.");
  }

  if (started_) {
    return absl::FailedPreconditionError(
        "FakeEthercatHost has already entered cyclic operation, cannot start "
        "again.");
  }

  started_ = true;
  working_counter_ = GetNumberOfRegisteredDevices() *
                     kEthercatWorkingCounterIncrementReadWriteCommandSuccess;

  return absl::OkStatus();
}

absl::Status FakeEthercatHost::StopCyclicOperation() {
  started_ = false;
  pdos_configured_ = false;

  // Clear PDO values, but not the entries (which are registered).
  for (auto& it : object_dictionary_) {
    it.second = std::monostate{};
  }
  return absl::OkStatus();
}

absl::StatusOr<absl::Span<const EthercatDeviceInfo>>
FakeEthercatHost::GetDevicesOnBus() {
  if (!connected_) {
    return absl::FailedPreconditionError(
        "Cannot get devices before connection.");
  }
  return devices_on_bus_;
}

absl::Status FakeEthercatHost::RegisterDeviceForPdoTransfer(
    const EthercatDeviceAddress& device_address) {
  if (pdos_configured_) {
    return absl::FailedPreconditionError(
        "Cannot register a device after PDOs have been configured.");
  }

  if (HasDeviceBeenRegistered(device_address)) {
    return absl::AlreadyExistsError(absl::StrFormat(
        "EtherCAT device with address: %shas already been registered.",
        EthercatDeviceAddressToString(device_address)));
  }

  registered_devices_.push_back(device_address);
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::ClearRegisteredDevices() {
  if (started_) {
    return absl::FailedPreconditionError(
        "Cannot clear registered devices after cyclic operation has started.");
  }

  registered_devices_.clear();
  return absl::OkStatus();
}

int FakeEthercatHost::GetNumberOfRegisteredDevices() const {
  return registered_devices_.size();
}

absl::StatusOr<EthercatApplicationLayerState>
FakeEthercatHost::GetDeviceApplicationLayerState(
    const EthercatDeviceAddress& device_address) {
  if (!connected_) {
    return absl::FailedPreconditionError(
        "Cannot get application layer states before connection.");
  }
  // Use std::visit and std::find to see if `device_address` matches any station
  // alias or bus position in `devices_on_bus_`.
  struct DeviceAddressMatcher {
    bool operator()(EthercatBusPosition bus_position) {
      return bus_position == info.bus_position;
    }
    bool operator()(EthercatStationAlias alias) {
      return alias == info.station_alias;
    }
    const EthercatDeviceInfo& info;
  };

  const auto& device_on_bus_it = std::find_if(
      devices_on_bus_.begin(), devices_on_bus_.end(),
      [&device_address](EthercatDeviceInfo device_info) {
        return std::visit(DeviceAddressMatcher{device_info}, device_address);
      });

  if (device_on_bus_it == devices_on_bus_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Device with address %s not found.",
                        EthercatDeviceAddressToString(device_address)));
  }

  // If it does match, try looking for a user-set application layer state.
  if (const auto& it = application_layer_states_.find(device_address);
      it != application_layer_states_.end()) {
    return it->second;
  }

  // Else, return a default based on whether we're in cyclic operation or not.
  BaseEthercatApplicationLayerState default_state =
      started_ ? BaseEthercatApplicationLayerState::kOp
               : BaseEthercatApplicationLayerState::kPreop;
  return EthercatApplicationLayerState{default_state,
                                       /*error = */ false};
}

absl::Status FakeEthercatHost::RegisterPdoEntry(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, int pdo_data_size_bytes,
    const PdoDirection direction) {
  if (pdos_configured_) {
    return absl::FailedPreconditionError(
        "Cannot register a device after PDOs have been configured.");
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }

  if (object_dictionary_.contains(
          {device_info->bus_position, object_address})) {
    VLOG(1) << absl::StreamFormat(
        "PDO entry for address [%s] already registered to "
        "FakeEthercatHost. RegisterPdoEntry request will have no effect.",
        object_address.ToString());
    return absl::OkStatus();
  }

  // The entry doesn't already exist, insert a monostate.
  object_dictionary_.insert(
      {{device_info->bus_position, object_address}, std::monostate()});

  return absl::OkStatus();
}

absl::Status FakeEthercatHost::ClearRegisteredPdos() {
  if (started_) {
    return absl::FailedPreconditionError(
        "Cannot clear registered PDOs after cyclic operation has started.");
  }

  object_dictionary_.clear();
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::WritePdoFromVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant value) {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }

  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("No PDO has been registered to address: %s.",
                        object_address.ToString()));
  }
  VLOG(1) << "Setting PDO: " << EthercatDeviceAddressToString(device_address)
          << " " << object_address.ToString();
  // All the alternatives in CanObjectValueVariant are valid to insert into
  // `pdo_object_dictionary_`, so we use std::visit with a single lambda where
  // the argument type is deduced.
  std::visit([&it](auto value) { it->second = value; }, value);

  return absl::OkStatus();
}

absl::Status FakeEthercatHost::ReadPdoToVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address,
    CanObjectValueVariant& output_variant) {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }
  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No entry found in the PDO object dictionary for object address: %s.",
        object_address.ToString()));
  }

  // Read the contents of the PDO object at the found address in
  // `object_dictionary_` into `read_value`.
  CanObjectValueVariant read_value;
  {
    absl::Status status = std::visit(
        [&read_value, &object_address](auto&& stored_value) {
          using T = std::decay_t<decltype(stored_value)>;
          if constexpr (std::is_same_v<T, std::monostate>) {
            return absl::NotFoundError(absl::StrFormat(
                "Tried to read PDO from an address (%s) that has "
                "not been written to yet.",
                object_address.ToString()));
          } else if constexpr (std::is_same_v<T, ByteSpan>) {
            return absl::NotFoundError(
                "Tried to read in integer-valued PDO from an address which "
                "contains a ByteSpan.");
          } else {
            read_value = stored_value;
            return absl::OkStatus();
          }
        },
        it->second);
    if (!status.ok()) {
      return status;
    }
  }

  // Now check if the `read_value` and `output_variant` have the same type.
  if (read_value.index() != output_variant.index()) {
    return absl::NotFoundError(absl::StrFormat(
        "Entry found in the PDO dictionary is not of the correct type: %s "
        "Type index %d vs %d.",
        object_address.ToString(), read_value.index(), output_variant.index()));
  }

  output_variant = read_value;
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::WriteSdoFromVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant value) {
  if (!connected_) {
    return absl::FailedPreconditionError(
        "Cannot download SDOs (write to device) while disconnected.");
  }
  if (started_) {
    return absl::FailedPreconditionError(
        "Cannot download SDOs (write to device) while in cyclic operation.");
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }

  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No entry found in the object dictionary for object address: %s.",
        object_address.ToString()));
  }

  size_t sdo_object_type = it->second.index();
  ObjectDictionaryValueType old_value = it->second;

  std::visit([&entry = it->second](auto& value) { entry = value; }, value);

  if (sdo_object_type != it->second.index()) {
    size_t new_sdo_object_type = it->second.index();
    // Restores old value to match hardware behavior.
    it->second = old_value;
    return absl::AbortedError(absl::StrCat(
        "Incorrect variant type for SDO write to ", object_address.ToString(),
        " on device ", EthercatDeviceAddressToString(device_address),
        ". Object dictionary contains a type with index ", sdo_object_type,
        " but this call received type ", new_sdo_object_type, "."));
  }

  return absl::OkStatus();
}

absl::Status FakeEthercatHost::ReadSdoToVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address,
    CanObjectValueVariant& output_variant) {
  if (!connected_) {
    return absl::FailedPreconditionError(
        "Cannot upload SDOs (read from device) while disconnected.");
  }
  if (started_) {
    return absl::FailedPreconditionError(
        "Cannot upload SDOs (read from device) while in cyclic operation.");
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }

  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::AbortedError(
        absl::StrCat("No entry found in the SDO object dictionary for device ",
                     EthercatDeviceAddressToString(device_address),
                     " at index ", object_address.ToString()));
  }

  CanObjectValueVariant read_value;
  {
    absl::Status status = std::visit(
        [&read_value](auto&& stored_value) {
          using T = std::decay_t<decltype(stored_value)>;
          if constexpr (std::is_same_v<T, std::monostate>) {
            return absl::AbortedError(
                "Tried to read in integer-valued SDO from an address which has "
                "not been written to yet.");
          } else if constexpr (std::is_same_v<T, ByteSpan>) {
            return absl::AbortedError(
                "Tried to read in integer-valued SDO from an address which "
                "contains a ByteSpan.");
          } else {
            read_value = stored_value;
            return absl::OkStatus();
          }
        },
        it->second);
    if (!status.ok()) {
      return status;
    }
  }

  // Now check if the `read_value` and `output_variant` have the same type.
  if (read_value.index() != output_variant.index()) {
    return absl::AbortedError(
        "Entry found in the SDO dictionary is not of the correct type.");
  }

  output_variant = read_value;
  return absl::OkStatus();
}

absl::StatusOr<EthercatHost::ByteSpan> FakeEthercatHost::GetPdoBytes(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address) {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }

  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }

  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No entry found in the PDO object dictionary for object address: %s.",
        object_address.ToString()));
  }

  ByteSpan* stored_span = std::get_if<ByteSpan>(&(it->second));

  if (stored_span == nullptr) {
    return absl::NotFoundError(
        "Entry found in the PDO dictionary is not of ByteSpan type.");
  }

  return *stored_span;
}

absl::StatusOr<int> FakeEthercatHost::GetWorkingCounter() const {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }
  return working_counter_;
}

absl::Status FakeEthercatHost::PreCycle() {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

absl::Status FakeEthercatHost::PostCycle() {
  {
    absl::Status status = CheckStarted();
    if (!status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

bool FakeEthercatHost::IsConnected() const { return connected_; }

bool FakeEthercatHost::ArePdosConfigured() const { return pdos_configured_; }

bool FakeEthercatHost::HasCyclicOperationStarted() const { return started_; }

absl::Status FakeEthercatHost::SetDevicesOnBus(
    absl::Span<const EthercatDeviceInfo> devices) {
  devices_on_bus_.assign(devices.begin(), devices.end());

  // Configures object dictionary entries.
  // First sets all known Ds301 & Ds402 & custom entries to 0.
  // Then sets all predefined values based on kSdoDefaultValues.
  // Finally, sets the serial number based on device.serial_number.
  object_dictionary_.clear();
  for (EthercatDeviceInfo& device : devices_on_bus_) {
    for (const CanObjectInfoVariant& object_entry : kDs301Objects) {
      auto object_address = CanObjectAddressFromCanObjectInfo(object_entry);
      // Sets known entries to a default value.
      object_dictionary_.insert(
          {{device.bus_position, object_address},
           std::visit(
               [](auto& entry) -> ObjectDictionaryValueType {
                 return entry.Cast(0);
               },
               object_entry)});
    }
    for (const CanObjectInfoVariant& object_entry : kDs402Objects) {
      auto object_address = CanObjectAddressFromCanObjectInfo(object_entry);
      // Sets known entries to a default value.
      object_dictionary_.insert(
          {{device.bus_position, object_address},
           std::visit(
               [](auto& entry) -> ObjectDictionaryValueType {
                 return entry.Cast(0);
               },
               object_entry)});
    }
    for (const CanObjectInfoVariant& object_entry : kCustomCanObjects) {
      auto object_address = CanObjectAddressFromCanObjectInfo(object_entry);
      // Sets known entries to a default value.
      object_dictionary_.insert(
          {{device.bus_position, object_address},
           std::visit(
               [](auto& entry) -> ObjectDictionaryValueType {
                 return entry.Cast(0);
               },
               object_entry)});
    }
    for (const auto& object_entry_and_value : kSdoDefaultValues) {
      // Can't use structured binding here as they are not capturable in C++17.
      // https://en.cppreference.com/w/cpp/language/structured_binding
      auto object_address =
          CanObjectAddressFromCanObjectInfo(object_entry_and_value.first);
      auto& value = object_entry_and_value.second;
      absl::StatusOr<ObjectDictionaryValueType> object_dict_entry = std::visit(
          [&value](auto& entry) -> absl::StatusOr<ObjectDictionaryValueType> {
            auto internal_value = entry.ValueFromVariant(value);
            if (!internal_value.ok()) {
              return internal_value.status();
            }
            return *internal_value;
          },
          object_entry_and_value.first);
      if (!object_dict_entry.ok()) {
        return object_dict_entry.status();
      }
      object_dictionary_.insert_or_assign(
          {device.bus_position, object_address}, *object_dict_entry);
    }
    // Serial number is a special case and set dynamically.
    object_dictionary_.insert_or_assign(
        {device.bus_position, kSerialNumberObjectInfo.address},
        kSerialNumberObjectInfo.Cast(device.serial_number));
    // Blob is a special case type-wise.
    object_dictionary_.insert_or_assign(
        {device.bus_position, kTxPdoBlobAddress},
        absl::MakeSpan(tx_pdo_binary_blob_));
    object_dictionary_.insert_or_assign(
        {device.bus_position, kRxPdoBlobAddress},
        absl::MakeSpan(rx_pdo_binary_blob_));
  }

  return absl::OkStatus();
}

bool FakeEthercatHost::HasDeviceBeenRegistered(
    const EthercatDeviceAddress& device_address) const {
  return std::find(registered_devices_.begin(), registered_devices_.end(),
                   device_address) != registered_devices_.end();
}

void FakeEthercatHost::SetDeviceApplicationLayerState(
    const EthercatDeviceAddress& device_address,
    const EthercatApplicationLayerState& state) {
  application_layer_states_[device_address] = state;
}

bool FakeEthercatHost::HasPdoBeenRegistered(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address) const {
  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (device_info.ok()) {
    return object_dictionary_.contains(
        {device_info->bus_position, object_address});
  } else {
    return false;
  }
}

absl::Status FakeEthercatHost::SetPdoBytes(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, ByteSpan pdo_bytes) {
  absl::StatusOr<EthercatDeviceInfo> device_info =
      EthercatDeviceAddressToDeviceInfo(device_address, devices_on_bus_);
  if (!device_info.ok()) {
    return device_info.status();
  }
  auto it =
      object_dictionary_.find({device_info->bus_position, object_address});
  if (it == object_dictionary_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "No entry found in the PDO object dictionary for object address: %s.",
        object_address.ToString()));
  }

  it->second = pdo_bytes;
  return absl::OkStatus();
}

void FakeEthercatHost::SetWorkingCounter(int working_counter) {
  working_counter_ = working_counter;
}

absl::Status FakeEthercatHost::CheckStarted() const {
  if (started_) return absl::OkStatus();

  return absl::FailedPreconditionError(
      "FakeEthercatHost has not entered cyclic operation.");
}

}  // namespace barkour
