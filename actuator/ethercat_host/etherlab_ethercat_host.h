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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_ETHERCAT_HOST_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_ETHERCAT_HOST_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "canopen.h"
#include "ethercat_host.h"
#include "types.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "etherlab/include/ecrt.h"

namespace barkour {

// EtherLab-based implementation of an EtherCAT host.
//
// This is not designed to be thread-safe.
//
// While EtherLab supports a hybrid addressing scheme, where each device on the
// bus can be addressed by its relative position to the first device with a
// given alias, this implementation only supports addressing by absolute
// position or alias (without relative position).
//
// This class takes no responsibility for handling errors or misconfigurations
// caused by duplicate addresses.
class EtherlabEthercatHost : public EthercatHost {
 public:
  // Default constructor. Uses the first available EtherLab host active on the
  // system when connecting.
  EtherlabEthercatHost();

  // Alternative constructor. Uses the EtherLab host with an explicit index when
  // connecting.
  explicit EtherlabEthercatHost(int host_index);

  // Requests an EtherLab host instance.
  //
  // This will also query the bus for available devices. If the devices on the
  // bus have changed, you will need to disconnect then connect again in order
  // for this to be recognised properly.
  absl::Status Connect() override;

  // Frees the EtherLab host instance, if one has been allocated.
  absl::Status Disconnect() override;

  // Allocates memory and acquires resources for the registered PDOs, for the
  // registered devices.
  //
  // This will fail and return an internal error if the registered devices do
  // not exist on the bus, or if registered PDOs do not correspond to devices or
  // PDOs available on the bus, or if the PDO size exceeds the maximum PDO size
  // of 256 bytes.
  absl::Status ConfigurePdoTransfer() override;

  // Activates the EtherLab host, setting it up for cyclic operation.
  absl::Status StartCyclicOperation() override;

  // Deactivates the EtherCAT host, exiting cyclic operation.
  //
  // Also de-allocates memory and frees resources for registered PDOs.
  absl::Status StopCyclicOperation() override;

  absl::Status PreCycle() override;
  absl::Status PostCycle() override;

  // Returns a list of devices on the bus, or a failed precondition error if the
  // host has already been initialized.
  //
  // Note that this will query EtherLab for devices on the bus at each call, so
  // the return value may change between calls if devices are connected or
  // disconnected.
  absl::StatusOr<absl::Span<const EthercatDeviceInfo>> GetDevicesOnBus()
      override;

  // Registers a device to be available for PDOs. This can only be called when
  // the host is *not* initialized.
  //
  // Note that this does not check that the device is available on the bus.
  //
  // Returns:
  // - Failed precondition error: If the host is initialized.
  // - Already exists error: If the device address has already been registered.
  absl::Status RegisterDeviceForPdoTransfer(
      const EthercatDeviceAddress& device_address) override;

  // Clears all registered devices. This can only be called when the host is
  // *not* initialized.
  //
  // Returns:
  // - Failed precondition error: If the host is initialized.
  absl::Status ClearRegisteredDevices() override;

  // Returns the number of registered devices.
  int GetNumberOfRegisteredDevices() const override;

  // Returns the application layer state of the given device, if available.
  //
  // Possible errors:
  // - Not found error: if the device was not found on the bus.
  absl::StatusOr<EthercatApplicationLayerState> GetDeviceApplicationLayerState(
      const EthercatDeviceAddress& device_address) override;

  // Registers a PDO entry with the EtherLab host. This can only be called when
  // the host is *not* initialized.
  //
  // pdo_data_size_bytes and direction are currently unused. They will be needed
  // to implement PDOs in a dynamic way (configured via SDO). Right now this
  // info is hard-coded in etherlab_device_data.cc.
  //
  // Returns:
  // - Failed precondition error: If the device has been initialized.
  // - Not found error: If the device was not found on the bus.
  // - Already exists error: if the given object address has already been
  //   registered.
  absl::Status RegisterPdoEntry(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                int pdo_data_size_bytes,
                                PdoDirection direction) override;

  // Clears all registered PDO entries. This can only be called when the host is
  // *not* initialized.
  //
  // Returns:
  // - Failed precondition error: If the device has been initialized.
  absl::Status ClearRegisteredPdos() override;

  // Returns a failed precondition error if the driver is not initialized.
  // Returns not found error if the device was not found on the bus.
  absl::StatusOr<ByteSpan> GetPdoBytes(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address) override;

  // Low-overhead read access to mapped PDOs via the internal PdoMemoryMap.
  //
  // Advanced use only! This method is targeted at low-level, low-overhead
  // access (e.g. for logging purposes). Use ReadPdo or GetPdoBytes for general
  // PDO data access.
  //
  // PdoMemoryMap data is only valid between calls to PreCycle and PostCycle.
  // This method should only be called after ConfigurePdoTransfer().
  //
  // Returns a pointer to the PdoMemory map containing the raw data exchanged by
  // the EtherCAT host with all devices. The returned pointer is only valid
  // until Finalize is called.
  absl::StatusOr<const PdoMemoryMap* const> GetPdoMemoryMap() const override;

  // Returns the working counter for the EtherCAT datagram received in the last
  // call to `PreCycle`.
  //
  // Returns a failed precondition error if the driver is not initialized, or if
  // `PreCycle` hasn't been called yet.
  absl::StatusOr<int> GetWorkingCounter() const override;

 private:
  // Methods which read and write PDO data. Will return a failed precondition
  // error if the driver is not started and a not found error if the device
  // does not exist on the bus.
  absl::Status WritePdoFromVariant(const EthercatDeviceAddress& device_address,
                                   const CanObjectAddress& object_address,
                                   CanObjectValueVariant value) override;

  absl::Status ReadPdoToVariant(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                CanObjectValueVariant& value) override;

  // Methods which read and write SDO data. SDOs can only be transferred after
  // initialization and before starting. Will return a failed precondition
  // error if the driver if this condition does not hold, and a not found error
  // if the device does not exist on the bus.
  absl::Status WriteSdoFromVariant(const EthercatDeviceAddress& device_address,
                                   const CanObjectAddress& object_address,
                                   CanObjectValueVariant value) override;

  absl::Status ReadSdoToVariant(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                CanObjectValueVariant& value) override;

  // Registers the memory locations of each of the PDOs in the domain. Should be
  // called after the PDOs have been registered on `domain_` (using the EtherLab
  // function ecrt_domain_reg_pdo_entry_list).
  absl::Status RegisterPdoMemoryLocations();

  // Returns OK if an object exists in pdo_memory_map_ at the given address,
  // with a nonempty span which does not start at 0x0. Otherwise, returns an
  // internal error.
  absl::Status CheckForPdoMapEntry(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address) const;

  absl::Status CheckStarted() const;

  // Cleans up all the resources acquired during `StartCyclicOperation` which
  // are not freed by the call to `ecrt_master_deactivate`.
  void CyclicOperationCleanup();

  int host_index_;

  std::unique_ptr<std::chrono::steady_clock> const clock_;

  // Null until connection.
  std::unique_ptr<ec_master_t, void (*)(ec_master_t*)> etherlab_host_;

  // Raw pointers to a memory "domain" which is used for PDO mappings. These are
  // owned by EtherLab, and remain valid until `ecrt_master_deactivate` is
  // called.
  ec_domain_t* domain_;
  uint8_t* domain_pd_;

  // Devices on the bus. Fixed during cyclic operation.
  std::vector<EthercatDeviceInfo> devices_;

  // Lookup table for alias to bus position for efficient SDO/PDO access.
  absl::flat_hash_map<EthercatStationAlias, EthercatBusPosition>
      alias_to_bus_position_;

  // nullopt until StartCyclicOperation is called.
  std::optional<ec_domain_state_t> domain_state_;

  std::vector<EthercatDeviceAddress> registered_devices_;
  std::vector<ec_pdo_entry_reg_t> domain_registration_entries_;

  // Tracks the mapped memory for a PDO object.
  //
  // Populated by RegisterPdoEntry, with the start of the memory block (as an
  // offset from domain_pd_) determined by ecrt_domain_reg_pdo_entry_list().
  PdoMemoryMap pdo_memory_map_;

  // In order to organise the PDO memory, EtherLab requires a series of unsigned
  // int pointers which it will associate with the offsets in the PDO memory
  // block. This map is used for persistent storage of these ints with pointer
  // stability of values.
  absl::flat_hash_map<std::pair<EthercatBusPosition, CanObjectAddress>,
                      std::unique_ptr<uint32_t>>
      pdo_offset_map_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_ETHERLAB_ETHERCAT_HOST_H_
