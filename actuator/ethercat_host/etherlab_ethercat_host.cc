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

#include "etherlab_ethercat_host.h"

#include <algorithm>
#include <chrono>  // NOLINT(build/c++11)
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "canopen.h"
#include "constants.h"
#include "ethercat_host.h"
#include "etherlab_device_data.h"
#include "types.h"
#include "absl/cleanup/cleanup.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "etherlab/include/ecrt.h"

namespace barkour {
namespace {

constexpr uint32_t kProductCode = 0x1000;
constexpr int kMaxPdoSizeBits = 8 * 256;

struct DevicePositionAndAliasIntegers {
  uint16_t position;
  uint16_t alias;
};

// Helper method to get integers for the device bus position and alias to pass
// to EtherLab, from a device address.
//
// If an address containing a bus position is passed, the returned alias will
// be zero, and if an address containing a station alias is passed, the returned
// bus position will be zero.
DevicePositionAndAliasIntegers GetPositionAndAliasFromDeviceAddress(
    const EthercatDeviceAddress& device_address) {
  DevicePositionAndAliasIntegers output{0, 0};

  struct OutputVisitor {
    void operator()(EthercatBusPosition bus_position) {
      output_struct.position = static_cast<uint16_t>(bus_position);
    }
    void operator()(EthercatStationAlias station_alias) {
      output_struct.alias = static_cast<uint16_t>(station_alias);
    }
    DevicePositionAndAliasIntegers& output_struct;
  };

  std::visit(OutputVisitor{output}, device_address);
  return output;
}

// Returns the bus position as an uint16_t, assuming that device_address
// contains a bus position. Do not use this with a station alias.
inline uint16_t GetPositionIntegerFromDeviceAddress(
    const EthercatDeviceAddress& device_address) {
  return static_cast<uint16_t>(std::get<EthercatBusPosition>(device_address));
}

EthercatApplicationLayerState GetApplicationLayerStateFromEtherLabFormat(
    uint8_t etherlab_al_state) {
  bool error = etherlab_al_state & 0x10;

  switch (etherlab_al_state & 0x0f) {
    case 1:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kInit, error};
    case 2:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kPreop, error};
    case 3:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kBoot, error};
    case 4:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kSafeop, error};
    case 8:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kOp, error};
    default:
      return EthercatApplicationLayerState{
          BaseEthercatApplicationLayerState::kUnknown, error};
  }
}

absl::StatusOr<EthercatBusPosition> GetPositionFromDeviceAddress(
    const EthercatDeviceAddress& device_address,
    const absl::flat_hash_map<EthercatStationAlias, EthercatBusPosition>&
        alias_to_bus_position) {
  struct AddressVisitor {
    absl::StatusOr<EthercatBusPosition> operator()(
        EthercatBusPosition bus_position) {
      return bus_position;
    }

    absl::StatusOr<EthercatBusPosition> operator()(
        EthercatStationAlias device_alias) {
      auto it = alias_to_bus_position.find(device_alias);
      if (it != alias_to_bus_position.end()) return it->second;
      return absl::NotFoundError(absl::StrCat(
          "Device lookup failed for station alias: ", device_alias));
    }
    const absl::flat_hash_map<EthercatStationAlias, EthercatBusPosition>&
        alias_to_bus_position;
  };
  return std::visit(AddressVisitor{alias_to_bus_position}, device_address);
}

absl::Status CheckPdoDataSize(
    const DevicePositionAndAliasIntegers& position_and_alias) {
  for (const ec_sync_info_t& device_sync : kEtherlabDeviceSyncs) {
    if (device_sync.pdos != nullptr && device_sync.n_pdos > 0) {
      int pdo_size_bits = 0;
      for (size_t i = 0; i < device_sync.n_pdos; ++i) {
        const ec_pdo_info_t& pdo_info = device_sync.pdos[i];
        for (size_t j = 0; j < pdo_info.n_entries; ++j) {
          const ec_pdo_entry_info_t& pdo_entry = pdo_info.entries[j];
          pdo_size_bits += pdo_entry.bit_length;
        }
      }
      std::string pdo_size_info_message = absl::StrCat(
          "Device (bus_position=", position_and_alias.position,
          ", alias=", position_and_alias.alias, ") has a PDO size of ",
          pdo_size_bits, " bits for sync manager ",
          static_cast<int>(device_sync.index), ".");
      VLOG(4) << pdo_size_info_message;
      if (pdo_size_bits > kMaxPdoSizeBits) {
        return absl::OutOfRangeError(absl::StrCat(
            pdo_size_info_message, " This exceeds the limit of ",
            kMaxPdoSizeBits, " bits for RX or TX PDO transmission."));
      }
    }
  }
  return absl::OkStatus();
}

}  // namespace

EtherlabEthercatHost::EtherlabEthercatHost() : EtherlabEthercatHost(0) {}

EtherlabEthercatHost::EtherlabEthercatHost(int host_index)
    : host_index_(0),
      etherlab_host_(nullptr, &ecrt_release_master),
      domain_(nullptr),
      domain_pd_(nullptr) {}

absl::Status EtherlabEthercatHost::Connect() {
  if (etherlab_host_ != nullptr) {
    return absl::FailedPreconditionError(
        "EtherlabEthercatHost has already been connected.");
  }

  etherlab_host_ = std::unique_ptr<ec_master_t, void (*)(ec_master_t*)>(
      ecrt_request_master(host_index_), ecrt_release_master);

  if (etherlab_host_ == nullptr) {
    return absl::NotFoundError(
        "Request for an EtherLab host failed. Likely the EtherCAT host is not "
        "configured correctly or hasn't been started.");
  }

  // Query the bus for the device information at each bus position starting from
  // zero, until this fails.
  VLOG(1) << "Querying the EtherCAT bus for devices.";

  int bus_position = 0;
  while (true) {
    ec_slave_info_t etherlab_device_info;
    if (ecrt_master_get_slave(etherlab_host_.get(), bus_position,
                              &etherlab_device_info) == 0) {
      // Success
      devices_.push_back(EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(etherlab_device_info.position),
          .station_alias = EthercatStationAlias(etherlab_device_info.alias),
          .vendor_id = etherlab_device_info.vendor_id,
          .product_code = etherlab_device_info.product_code,
          .serial_number = etherlab_device_info.serial_number,
          .name = std::string(etherlab_device_info.name),
      });

      alias_to_bus_position_.insert(
          {EthercatStationAlias(etherlab_device_info.alias),
           EthercatBusPosition(etherlab_device_info.position)});

      ++bus_position;
    } else {
      VLOG(1) << absl::StreamFormat(
          "Failed to find an EtherCAT device at bus position: %d. Ending "
          "device lookup.",
          bus_position);
      return absl::OkStatus();
    }
  }
}

absl::Status EtherlabEthercatHost::Disconnect() {
  if (etherlab_host_ == nullptr) return absl::OkStatus();

  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "Cannot disconnect from host while in cyclic operation.");
  }

  alias_to_bus_position_.clear();
  devices_.clear();
  etherlab_host_ = nullptr;

  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::ConfigurePdoTransfer() {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "EtherlabEthercatHost has not been connected.");
  }

  if (domain_ != nullptr) {
    return absl::FailedPreconditionError(
        "PDO transfer has already been configured.");
  }

  // If there is an error somewhere between here and the end of the method, we
  // want to deactivate `etherlab_host_` (and clean up associated objects) so
  // that the state of the class was as if `ConfigurePdoTransfer` was never
  // called.
  absl::Cleanup cleanup = [this] {
    // Deactivate should be able to be called successfully even if the host has
    // not yet been activated, with the effect of freeing all the resources
    // acquired for PDO transfer.
    ecrt_master_deactivate(etherlab_host_.get());
    domain_ = nullptr;
  };

  // Create a new process data domain. A domain can be used for registering
  // PDOs and exchanging them in cyclic operation.
  //
  // Note this memory will be freed by EtherLab on a call to
  // `ecrt_master_deactivate`.
  domain_ = ecrt_master_create_domain(etherlab_host_.get());
  if (domain_ == nullptr) {
    return absl::InternalError(
        "Failed to create EtherLab process data domain.");
  }

  for (const EthercatDeviceAddress& registered_device_address :
       registered_devices_) {
    // Obtain a device configuration object for the device at the given address.
    DevicePositionAndAliasIntegers position_and_alias =
        GetPositionAndAliasFromDeviceAddress(registered_device_address);

    // The device is assumed to be a motor controller.
    ec_slave_config_t* device_config = ecrt_master_slave_config(
        etherlab_host_.get(), position_and_alias.alias,
        position_and_alias.position, kGoogleEthercatVendorId, kProductCode);

    if (device_config == nullptr) {
      return absl::InternalError(
          "Failed to get EtherLab device configuration.");
    }

    {
      auto status = CheckPdoDataSize(position_and_alias);
      if (!status.ok()) return status;
    }

    // Configure the PDOs.
    if (ecrt_slave_config_pdos(device_config, EC_END,
                               kEtherlabDeviceSyncs.data())) {
      return absl::InternalError("Failed to configure PDOs.");
    }
  }

  // Add a final empty entry to the list of PDOs, required by
  // ecrt_domain_reg_pdo_entry_list().
  domain_registration_entries_.push_back({});

  // Register PDO entries for process data exchange in a domain.
  //
  // From the Etherlab PDF:
  // The process data image can be easily managed by creating so-called
  // “domains”, which allow grouped PDO exchange. They also take care of
  // managing the datagram structures needed to exchange the PDOs. Domains are
  // mandatory for process data exchange, so there has to be at least one.
  if (ecrt_domain_reg_pdo_entry_list(domain_,
                                     &domain_registration_entries_[0])) {
    return absl::InternalError("PDO entry registration failed.");
  }

  // If we reach here then `ConfigurePdoTransfer` is guaranteed to succeed, so
  // we cancel the cleanup.
  std::move(cleanup).Cancel();

  return absl::OkStatus();
}

void EtherlabEthercatHost::CyclicOperationCleanup() {
  domain_state_ = std::nullopt;
  domain_pd_ = nullptr;

  // Clear values of PDO memory map, but not the entries themselves, to reverse
  // the setting of the entries in `RegisterPdoMemoryLocations`.
  for (auto& it : pdo_memory_map_) {
    it.second = {};
  }
}

absl::Status EtherlabEthercatHost::StartCyclicOperation() {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "EtherlabEthercatHost has not been initialized.");
  }

  if (domain_ == nullptr) {
    return absl::FailedPreconditionError(
        "PDO transfer has not been configured, cannot start cyclic operation.");
  }

  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "EtherlabEthercatHost has already entered cyclic operation.");
  }

  // Cleanup to leave a valid internal state if something fails.
  absl::Cleanup cleanup = [this] {
    ecrt_master_deactivate(etherlab_host_.get());
    this->CyclicOperationCleanup();
  };

  if (int activate_status = ecrt_master_activate(etherlab_host_.get());
      activate_status != 0) {
    // `ecrt_master_activate` returns `-errno` if it fails.
    return absl::ErrnoToStatus(-activate_status,
                               "Failed to activate EtherLab host: ");
  }

  // Getting the PDO data pointer from the domain must be done after the host is
  // activated.
  domain_pd_ = ecrt_domain_data(domain_);
  if (domain_pd_ == nullptr) {
    return absl::InternalError("Failed to get handle to process data memory.");
  }

  {
    auto status = RegisterPdoMemoryLocations();
    if (!status.ok()) return status;
  }

  domain_state_.emplace(ec_domain_state_t{});

  std::move(cleanup).Cancel();

  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::StopCyclicOperation() {
  if (!domain_state_.has_value()) return absl::OkStatus();
  ecrt_master_deactivate(etherlab_host_.get());
  CyclicOperationCleanup();

  return absl::OkStatus();
}

absl::StatusOr<absl::Span<const EthercatDeviceInfo>>
EtherlabEthercatHost::GetDevicesOnBus() {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot query for devices on bus before the EtherlabEthercatHost is "
        "initialized.");
  }
  return devices_;
}

absl::Status EtherlabEthercatHost::RegisterDeviceForPdoTransfer(
    const EthercatDeviceAddress& device_address) {
  if (domain_ != nullptr) {
    return absl::FailedPreconditionError(
        "Cannot register device after PDOs have been configured.");
  }

  if (std::find(registered_devices_.begin(), registered_devices_.end(),
                device_address) != registered_devices_.end()) {
    return absl::AlreadyExistsError(absl::StrFormat(
        "EtherCAT device with address: %s has already been registered.",
        EthercatDeviceAddressToString(device_address)));
  }

  registered_devices_.push_back(device_address);
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::ClearRegisteredDevices() {
  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "Cannot clear registered devices during cyclic operation.");
  }

  registered_devices_.clear();
  return absl::OkStatus();
}

int EtherlabEthercatHost::GetNumberOfRegisteredDevices() const {
  return registered_devices_.size();
}

absl::StatusOr<EthercatApplicationLayerState>
EtherlabEthercatHost::GetDeviceApplicationLayerState(
    const EthercatDeviceAddress& device_address) {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot get application layer states until connected.");
  }

  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();

  ec_slave_info_t etherlab_device_info;
  if (ecrt_master_get_slave(etherlab_host_.get(),
                            static_cast<uint16_t>(bus_position),
                            &etherlab_device_info) == 0) {
    return GetApplicationLayerStateFromEtherLabFormat(
        etherlab_device_info.al_state);
  }
  return absl::NotFoundError(
      absl::StrFormat("Could not find device at address %s to query the "
                      "application layer state.",
                      EthercatDeviceAddressToString(device_address)));
}

absl::Status EtherlabEthercatHost::RegisterPdoEntry(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, int pdo_data_size_bytes,
    const PdoDirection direction) {
  if (domain_ != nullptr) {
    return absl::FailedPreconditionError(
        "Cannot register PDO after PDOs have been configured.");
  }

  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();

  if (pdo_memory_map_.contains({bus_position, object_address})) {
    VLOG(1) << absl::StreamFormat(
        "PDO entry for address [%s] already registered to "
        "EtherlabEthercatHost. RegisterPdoEntry request will have no effect.",
        object_address.ToString());
    return absl::OkStatus();
  }

  // Allocate an entry in the PDO memory map, whose value is an empty span which
  // will later be pointed to a chunk of mapped memory. This avoids allocating
  // memory after activating the bus, when running `StartCyclicOperation`.
  pdo_memory_map_[{bus_position, object_address}] = {};

  // The EtherLab library requires the use of an unsigned int type here.
  pdo_offset_map_[{bus_position, object_address}] =
      std::make_unique<unsigned int>(0);

  DevicePositionAndAliasIntegers position_and_alias =
      GetPositionAndAliasFromDeviceAddress(device_address);

  domain_registration_entries_.push_back({
      .alias = position_and_alias.alias,
      .position = position_and_alias.position,
      .vendor_id = kGoogleEthercatVendorId,
      .product_code = kProductCode,
      .index = object_address.index,
      .subindex = object_address.subindex,
      .offset = pdo_offset_map_[{bus_position, object_address}].get(),
      // This will cause an error to be thrown if the PDO entry does not
      // byte-align, but that shouldn't be a problem for us.
      .bit_position = nullptr,
  });

  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::ClearRegisteredPdos() {
  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "Cannot clear registered PDOs during cyclic operation.");
  }

  pdo_offset_map_.clear();
  pdo_memory_map_.clear();
  domain_registration_entries_.clear();
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::PreCycle() {
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }

  // Write application time to master.
  // Because the master does not update the application time, we must do this
  // every cycle. This should be done at the beginning of the cycle, to ensure
  // consistency. This allows for time synchronization between the DC
  // reference clock and the application time.
  ecrt_master_application_time(
      etherlab_host_.get(),
      std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now())
          .time_since_epoch()
          .count());

  // Receive process data.
  ecrt_master_receive(etherlab_host_.get());

  // Evaluates the working counters of the received datagrams and outputs
  // statistics, if necessary.
  // This must be called after ecrt_master_receive().
  ecrt_domain_process(domain_);

  // Check process data state.
  ecrt_domain_state(domain_, &(domain_state_.value()));

  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::PostCycle() {
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }

  ecrt_master_sync_reference_clock_to(
      etherlab_host_.get(),
      std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now())
          .time_since_epoch()
          .count());
  ecrt_master_sync_slave_clocks(etherlab_host_.get());

  // Send process data.
  ecrt_domain_queue(domain_);
  ecrt_master_send(etherlab_host_.get());
  return absl::OkStatus();
}

absl::StatusOr<EthercatHost::ByteSpan> EtherlabEthercatHost::GetPdoBytes(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address) {
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }

  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();

  if (object_address.index == 0x0) {
    return absl::InvalidArgumentError(
        "Cannot get PDO bytes for an object address with index 0x0.");
  }
  {
    auto status = CheckForPdoMapEntry(bus_position, object_address);
    if (!status.ok()) return status;
  }
  return pdo_memory_map_[{bus_position, object_address}];
}

absl::StatusOr<const EthercatHost::PdoMemoryMap* const>
EtherlabEthercatHost::GetPdoMemoryMap() const {
  return &pdo_memory_map_;
}

absl::StatusOr<int> EtherlabEthercatHost::GetWorkingCounter() const {
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }

  return domain_state_->working_counter;
}

absl::Status EtherlabEthercatHost::WritePdoFromVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant value) {
  // Consider using std::endian if C++20 is available.
#ifndef ABSL_IS_LITTLE_ENDIAN
  return absl::UnimplementedError(
      "EtherCAT requires data to be little endian, but the host stack is "
      "running on a big endian system. This is very uncommon. If you require "
      "big endian support, use std::byteswap or equivalent to reorder the "
      "bytes when writing object dictionary data.");
#endif
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }
  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();
  {
    auto status = CheckForPdoMapEntry(bus_position, object_address);
    if (!status.ok()) return status;
  }
  std::visit(
      [&pdo_entry_memory = pdo_memory_map_[{bus_position, object_address}]](
          auto& value) -> void {
        memcpy(pdo_entry_memory.data(), &value, sizeof(value));
      },
      value);
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::ReadPdoToVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant& value) {
#ifndef ABSL_IS_LITTLE_ENDIAN
  return absl::UnimplementedError(
      "EtherCAT requires data to be little endian, but the host stack is "
      "running on a big endian system. This is very uncommon. If you require "
      "big endian support, use std::byteswap or equivalent to reorder the "
      "bytes when reading object dictionary data.");
#endif
  {
    auto status = CheckStarted();
    if (!status.ok()) return status;
  }
  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();
  {
    auto status = CheckForPdoMapEntry(bus_position, object_address);
    if (!status.ok()) return status;
  }
  std::visit(
      [&pdo_entry_memory = pdo_memory_map_[{bus_position, object_address}]](
          auto& value) -> void {
        memcpy(&value, pdo_entry_memory.data(), sizeof(value));
      },
      value);
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::WriteSdoFromVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant value) {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot download SDOs (write to device) before connection.");
  }

  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "Cannot download SDOs (write to device) after entering cyclic "
        "operation.");
  }

  // This ensures we use a bus position rather than an alias.
  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();
  auto [success, abort_code] = std::visit(
      [etherlab_host = etherlab_host_.get(),
       position = GetPositionIntegerFromDeviceAddress(bus_position),
       &object_address](auto& value) -> std::tuple<int, int> {
        uint32_t abort_code;
        int success = ecrt_master_sdo_download(
            etherlab_host, position, object_address.index,
            object_address.subindex, (uint8_t*)&value, sizeof(value),
            &abort_code);
        return {success, abort_code};
      },
      value);
  if (success == 0) {
    return absl::OkStatus();
  } else {
    return absl::AbortedError(absl::StrCat(
        "SDO downloadload (host -> device) failed. Return value: ", success,
        " abort code: ", abort_code,
        ". Device address: ", EthercatDeviceAddressToString(device_address),
        " Object address: ", object_address.ToString()));
  }
}

absl::Status EtherlabEthercatHost::ReadSdoToVariant(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address, CanObjectValueVariant& value) {
  if (etherlab_host_ == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot upload SDOs (read from device) before connection.");
  }

  if (domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "Cannot upload SDOs (read from device) after entering cyclic "
        "operation.");
  }

  // This looks unnecessary, but ensures we are using a bus position rather than
  // an alias.
  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();
  auto [success, abort_code] = std::visit(
      [etherlab_host = etherlab_host_.get(),
       position = GetPositionIntegerFromDeviceAddress(bus_position),
       &object_address](auto& value) -> std::tuple<int, int> {
        uint32_t abort_code;
        size_t result_size;
        int success = ecrt_master_sdo_upload(
            etherlab_host, position, object_address.index,
            object_address.subindex, (uint8_t*)&value, sizeof(value),
            &result_size, &abort_code);
        if (success == 0 && sizeof(value) != result_size) {
          LOG(ERROR) << "SDO upload error: size of received data: "
                     << result_size
                     << "(bytes) does not match requested data size: "
                     << sizeof(value);
          success = 666;
        }
        return {success, abort_code};
      },
      value);
  if (success == 0) {
    return absl::OkStatus();
  } else {
    return absl::AbortedError(absl::StrCat(
        "SDO upload (device -> host) failed. Return value: ", success,
        " abort code: ", abort_code,
        ". Device address: ", EthercatDeviceAddressToString(device_address),
        " Object address: ", object_address.ToString()));
  }
}

absl::Status EtherlabEthercatHost::RegisterPdoMemoryLocations() {
  // Exclude the last element of domain_registration_entries_, which is empty.
  for (size_t i = 0; i < domain_registration_entries_.size() - 1; ++i) {
    const ec_pdo_entry_reg_t& pdo_entry = domain_registration_entries_[i];

    CanObjectAddress object_address(pdo_entry.index, pdo_entry.subindex);
    EthercatDeviceAddress device_address;

    if (pdo_entry.alias != 0 && pdo_entry.position != 0) {
      // This should never happen as we only register entries with nonzero alias
      // or position.
      return absl::InternalError(
          "Encountered PDO with both position- and alias-based address. This "
          "shouldn't happen.");
    } else if (pdo_entry.alias != 0) {
      device_address = EthercatStationAlias(pdo_entry.alias);
    } else {
      device_address = EthercatBusPosition(pdo_entry.position);
    }

    absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
        GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
    if (!maybe_bus_position.ok()) {
      return maybe_bus_position.status();
    }
    const EthercatBusPosition bus_position = maybe_bus_position.value();

    absl::StatusOr<int> maybe_pdo_byte_length =
        GetEtherlabPdoByteLength(object_address);
    if (!maybe_pdo_byte_length.ok()) {
      return maybe_pdo_byte_length.status();
    }
    int pdo_byte_length = maybe_pdo_byte_length.value();

    if (!pdo_memory_map_.contains({bus_position, object_address})) {
      // Shouldn't happen, but check anyway.
      return absl::InternalError(absl::StrFormat(
          "Pdo memory map entry not allocated for bus position [%d], device "
          "address %s.",
          static_cast<int>(bus_position), object_address.ToString()));
    }

    CHECK_NE(domain_pd_, nullptr);
    pdo_memory_map_[{bus_position, object_address}] =
        ByteSpan(reinterpret_cast<std::byte*>(domain_pd_ + *pdo_entry.offset),
                 pdo_byte_length);
  }
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::CheckForPdoMapEntry(
    const EthercatDeviceAddress& device_address,
    const CanObjectAddress& object_address) const {
  absl::StatusOr<const EthercatBusPosition> maybe_bus_position =
      GetPositionFromDeviceAddress(device_address, alias_to_bus_position_);
  if (!maybe_bus_position.ok()) {
    return maybe_bus_position.status();
  }
  const EthercatBusPosition bus_position = maybe_bus_position.value();
  auto it = pdo_memory_map_.find({bus_position, object_address});
  if (it == pdo_memory_map_.end()) {
    return absl::InternalError(
        absl::StrFormat("No PDO mapping for index: %#x, subindex: %#x",
                        object_address.index, object_address.subindex));
  } else if (it->second.empty()) {
    return absl::InternalError(
        absl::StrFormat("PDO mapping for index: %#x, subindex: %#x contains an "
                        "empty memory span.",
                        object_address.index, object_address.subindex));
  } else if (it->second.data() == nullptr) {
    return absl::InternalError(
        absl::StrFormat("PDO mapping for index: %#x, subindex: %#x contains an "
                        "memory span pointing to NULL.",
                        object_address.index, object_address.subindex));
  }
  return absl::OkStatus();
}

absl::Status EtherlabEthercatHost::CheckStarted() const {
  if (!domain_state_.has_value()) {
    return absl::FailedPreconditionError(
        "EtherlabEthercatHost has not started cyclic operation.");
  }
  return absl::OkStatus();
}

}  // namespace barkour
