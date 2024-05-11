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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_FAKE_ETHERCAT_HOST_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_FAKE_ETHERCAT_HOST_H_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

#include "custom_objects.h"
#include "canopen.h"
#include "ethercat_host.h"
#include "types.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

namespace barkour {

class FakeEthercatHost : public EthercatHost {
 public:
  FakeEthercatHost();

  // Returns:
  // - Failed precondition error: if connection already made.
  absl::Status Connect() override;

  // Returns:
  // - Failed precondition error: if cyclic operation has started.
  absl::Status Disconnect() override;

  // Returns:
  // - Failed precondition error: if PDO transfer already configured.
  // - Internal error: If any of the registered PDOs are registered with device
  //   addresses which have not been registered with `RegisterDevice`.
  absl::Status ConfigurePdoTransfer() override;

  // Returns:
  // - Failed precondition error: if not initialized or already started.
  absl::Status StartCyclicOperation() override;

  // Always succeeds.
  absl::Status StopCyclicOperation() override;

  // Returns the devices set by `SetDevicesOnBus`, an empty vector if this has
  // not been called yet, or a failed precondition error if the host is
  // initialized.
  absl::StatusOr<absl::Span<const EthercatDeviceInfo>> GetDevicesOnBus()
      override;

  // Registers a device to be available for PDO communications.
  //
  // Returns:
  // - Failed precondition error: if cyclic operation has started.
  // - Already exists error: If the given device address has already been
  //   registered.
  absl::Status RegisterDeviceForPdoTransfer(
      const EthercatDeviceAddress& device_address) override;

  // Clears registered devices.
  //
  // Returns a failed precondition error if the host is initialized.
  absl::Status ClearRegisteredDevices() override;

  // Returns the number of registered devices.
  int GetNumberOfRegisteredDevices() const override;

  // Returns the simulated application layer state of a device. The return value
  // will be:
  // - Failed precondition error, if `Connect` hasn't been called yet.
  // - Not found, if the device address does not match a device which has been
  //   passed to `SetDevicesOnBus`, else
  // - The value set by SetApplicationLayerState, if it has been called with the
  //   given device address, else
  // - A default value, which is the INIT state with no error.
  absl::StatusOr<EthercatApplicationLayerState> GetDeviceApplicationLayerState(
      const EthercatDeviceAddress& device_address) override;

  // Registers a PDO entry to be available for reading and writing. This adds
  // an entry to pdo_object_dictionary_ containing an std::monostate object -
  // note this method knows nothing about the type of the entry, and so nothing
  // can be read from the PDO until it is written to.
  //
  // Returns a failed precondition error if cyclic operation has started.
  absl::Status RegisterPdoEntry(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                int pdo_data_size_bytes,
                                PdoDirection direction) override;

  // Clears registered PDOs. Returns a failed precondition error if cyclic
  // operation has started.
  absl::Status ClearRegisteredPdos() override;

  // Gets the PDO byte span at a given address. This span can be written to,
  // which will mutate the underlying storage backing the span.
  //
  // Returns a failed precondition error if the cyclic operation has not yet
  // started.
  //
  // Note this will only succeed if `SetPdoBytes` has been used to explicitly
  // set a PDO byte span at the address. In particular, if e.g. a int16_t has
  // been written at `object_address`, this will return a not found error rather
  // than returning the span of bytes associated to the int16_t object.
  absl::StatusOr<ByteSpan> GetPdoBytes(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address) override;

  // Returns a simulated working counter for the fake host.
  //
  // By default, the working counter will be the number of registered devices on
  // the bus, multiplied by the per-device working counter increment
  // corresponding to a successful read and write operation (this increment is
  // 3). If an alternative working counter has been set with
  // `SetWorkingCounter`, this will be returned instead.
  //
  // Returns a failed precondition error if the cyclic operation has not yet
  // started.
  absl::StatusOr<int> GetWorkingCounter() const override;

  // Pre- and post- cycle methods are no-ops.
  absl::Status PreCycle() override;
  absl::Status PostCycle() override;

  // Test-specific methods.

  // Checks if the host has been connected yet.
  bool IsConnected() const;

  // Checks if PDO transfers have been configured yet.
  bool ArePdosConfigured() const;

  // Checks if the host has had cyclic operation started yet.
  bool HasCyclicOperationStarted() const;

  // Sets the return value of `GetDevicesOnBus`.
  // Fails with FailedPrecondition if cyclic operation has started.
  absl::Status SetDevicesOnBus(absl::Span<const EthercatDeviceInfo> devices);

  // Sets the application layer state of a specific device.
  void SetDeviceApplicationLayerState(
      const EthercatDeviceAddress& device_address,
      const EthercatApplicationLayerState& state);

  // Returns true if a device has been registered with the given device address.
  bool HasDeviceBeenRegistered(
      const EthercatDeviceAddress& device_address) const;

  // Returns true if a PDO has been registered with the given device and object
  // address.
  bool HasPdoBeenRegistered(const EthercatDeviceAddress& device_address,
                            const CanObjectAddress& object_address) const;

  // Sets the byte span for a given object address. Note that this will not take
  // ownership or a copy of the spanned data, so care should be taken that the
  // underlying data remains valid while the data is read using `GetPdoBytes`.
  //
  // This will overwrite any data which is currently at the given object
  // address.
  //
  // Returns a not found error if the PDO has not been registered.
  absl::Status SetPdoBytes(const EthercatDeviceAddress& device_address,
                           const CanObjectAddress& object_address,
                           ByteSpan pdo_bytes);

  // Sets the return value of `GetWorkingCounter`.
  void SetWorkingCounter(int working_counter);

 private:
  using ObjectDictionaryValueType =
      std::variant<std::monostate, int8_t, uint8_t, int16_t, uint16_t, int32_t,
                   uint32_t, uint64_t, int64_t, float, ByteSpan>;
  using ObjectDictionaryType =
      absl::flat_hash_map<std::pair<EthercatBusPosition, CanObjectAddress>,
                          ObjectDictionaryValueType>;

  // Write PDO data.
  //
  // Returns:
  // - Failed precondition error: if cyclic operation has not started.
  // - Not found error: If no PDO has been registered with the given device and
  //   object address.
  absl::Status WritePdoFromVariant(const EthercatDeviceAddress& device_address,
                                   const CanObjectAddress& object_address,
                                   CanObjectValueVariant value) override;

  // Read PDO data.
  //
  // Returns:
  // - Failed precondition error: if cyclic operation has not started.
  // - Not found error: If no PDO has been registered with the given device and
  //   object address, if the PDO has been registered but no data, or data of
  //   the wrong type, has been written to it.
  absl::Status ReadPdoToVariant(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                CanObjectValueVariant& output_variant) override;

  // Write SDO data.
  //
  // This should only be called before cyclic operation has started, i.e.
  // SDO exchanges should happen before a call to ConfigurePdoTransfer().
  //
  // Returns:
  // - OkStatus: If the device is on the (fake) bus and object_address matches
  //   an entry in kDs301Objects or kDs402Objects.
  // - Failed precondition error: If cyclic operation has started.
  // - AbortedError: If the device_address is not on the bus or object_address
  //   is not in kDs301Objects or kDs402Objects.
  absl::Status WriteSdoFromVariant(const EthercatDeviceAddress& device_address,
                                   const CanObjectAddress& object_address,
                                   CanObjectValueVariant value) override;

  // Reads data via SDO.
  //
  // This should only be called before cyclic operation has started, i.e.
  // SDO exchanges should happen before a call to ConfigurePdoTransfer().
  //
  // If successful, output_variant will be set to:
  // - Serial number: Device serial number as passed via SetDevicesOnBus.
  // - Default values for rated current, rated torque, encoder and gear entries.
  // - 0 for all other known object dictionary entries (DS301/DS402).
  //
  // All entries, except for the serial number, can be overwritten by calls to
  // WriteSdoFromVariant.
  //
  // Returns:
  // - OkStatus: If the device is on the (fake) bus and object_address matches
  //   an entry in kDs301Objects or kDs402Objects.
  // - Failed precondition error: If cyclic operation has started.
  // - AbortedError: If the device_address is not on the bus or object_address
  //   is not in kDs301Objects or kDs402Objects.
  absl::Status ReadSdoToVariant(const EthercatDeviceAddress& device_address,
                                const CanObjectAddress& object_address,
                                CanObjectValueVariant& output_variant) override;

  // Returns a failed precondition error if the host is not started.
  absl::Status CheckStarted() const;

  bool connected_;
  bool pdos_configured_;
  bool started_;

  std::vector<EthercatDeviceInfo> devices_on_bus_;
  std::vector<EthercatDeviceAddress> registered_devices_;
  std::array<std::byte, kBinaryBlobNumBytes> rx_pdo_binary_blob_;
  std::array<std::byte, kBinaryBlobNumBytes> tx_pdo_binary_blob_;
  ObjectDictionaryType object_dictionary_;

  absl::flat_hash_map<EthercatDeviceAddress, EthercatApplicationLayerState>
      application_layer_states_;

  int working_counter_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_HOST_IMPLEMENTATIONS_FAKE_ETHERCAT_HOST_H_
