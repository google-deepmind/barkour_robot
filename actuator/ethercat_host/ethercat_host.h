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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_ETHERCAT_HOST_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_ETHERCAT_HOST_H_

#include <cstddef>
#include <type_traits>
#include <utility>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_cat.h"
#include "canopen.h"
#include "types.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

namespace barkour {

namespace internal {

// Helper struct to work out statically if a given type is one of the
// alternatives in a variant.
template <typename T, typename VariantTypes>
struct IsTypeInVariant;

template <typename T, typename... Ts>
struct IsTypeInVariant<T, std::variant<Ts...>>
    : std::disjunction<std::is_same<T, Ts>...> {};

}  // namespace internal

// Provides a generic EtherCAT host interface.
//
// Currently this only supports CAN-over-EtherCAT communications, but may be
// expanded with further EtherCAT-based functionality in future (e.g.
// file-over-EtherCAT).
//
// The typical usage pattern for an EthercatHost looks like the following:
//
// ```
// // Create the class instance.
// EthercatHost* host = ...;
//
// // Connect to the bus.
// host->Connect();
//
// // If needed, query for devices on the bus, and send and receive some SDOs
// // for device configuration.
// host->GetDevicesOnBus();
// host->WriteSdo(device_address, object_address, value);
// host->ReadSdo(another_device_address, another_object_address)
// ...
//
// // Register required devices and PDOs. Note that this can be done either
// // before or after `Connect` is called, as sometimes the list of PDOs and
// // devices will be known statically, but at other times SDO or bus
// // information may be needed before this step can be carried out.
// host->RegisterDeviceForPdoTransfer(device_address);
// host->RegisterDeviceForPdoTransfer(another_device_address);
// ...
//
// host->RegisterPdoEntry(device_address, object_address, direction);
// host->RegisterPdoEntry(device_address, another_object_address, direction);
// ...
//
// // Configure the PDOs. This call will allocate mapped memory for the PDOs and
// // generally acquire resources.
// host->ConfigurePdoTransfer();
//
//
// // Start cyclic operation. As soon as this is called the calling thread
// // should enter real-time context, where `PreCycle` and `PostCycle` are
// // called at regular intervals, ideally with bounded jitter.
// host->StartCyclicOperation();
//
//
// // Read and write PDO data in the run loop.
// while (!finished) {
//   host->PreCycle()
//
//   host->ReadPdo(device_address, object_address);
//   ...
//
//   host->WritePdo(another_device_address, another_object_address, value);
//   ...
//
//   host->PostCycle();
//
//   // Sleep for a suitable time
//   ...
// }
//
// // Close down the host, in reverse order. Note the stopping cyclic operation
// // also de-configures the PDOs, so there is no standalone method to undo the
// // `ConfigurePdoTransfer` setup call.
// host->StopCyclicOperation();
// host->ClearRegisteredPdos();
// host->ClearRegisteredDevices();
// host->Disconnect();
// ```
class EthercatHost {
 public:
  // Represents a continuous block of allocated binary memory, such as that used
  // for mapping PDOs into memory.
  using ByteSpan = absl::Span<std::byte>;
  // Pointers to PDO data.
  using PdoMemoryMap =
      absl::flat_hash_map<std::pair<EthercatBusPosition, CanObjectAddress>,
                          ByteSpan>;

  virtual ~EthercatHost() = default;

  // Connects to the EtherCAT fieldbus. Default implementation is a no-op.
  virtual absl::Status Connect();

  // Removes the connection to the EtherCAT fieldbus, and deconfigures the PDOs
  // if required. Default implementation is a no-op.
  virtual absl::Status Disconnect();

  // Sets up bus for cyclic PDO data transfer.
  // Default implementation is a no-op.
  virtual absl::Status ConfigurePdoTransfer();

  // Requests for devices to enter OP state and start sending and receiving
  // PDOs. Between calls to `StartCyclicOperation` and `StopCyclicOperation`,
  // the `PreCycle` and `PostCycle` methods must be called at regular intervals
  // in a real-time context. Default implementation is a no-op.
  virtual absl::Status StartCyclicOperation();

  // Requests for devices to enter PREOP state, stops sending and receiving
  // PDOs, and clears the PDO configuration.
  //
  // Note, this will undo the effects of *both* `ConfigurePdoTransfer` and
  // `StartCyclicOperation`.
  //
  // Default implementation is a no-op.
  virtual absl::Status StopCyclicOperation();

  // Returns a list of devices on the bus. This will allocate memory and should
  // fail if called before `Connect` or after `StartCyclicOperation`.
  //
  // Note that this gives a list of *all* the devices on the bus, and is
  // independent of which devices have been registered with `RegisterDevice`.
  //
  // The returned span will be valid until Finalize() is called.
  virtual absl::StatusOr<absl::Span<const EthercatDeviceInfo>>
  GetDevicesOnBus() = 0;

  // Registers an EtherCAT device with the host to be configured for cyclic
  // communications.
  //
  // This should fail if called before `Connect` or after
  // `StartCyclicOperation`.
  virtual absl::Status RegisterDeviceForPdoTransfer(
      const EthercatDeviceAddress& device_address) = 0;

  // Clears all registered devices, if this is possible in the current state.
  virtual absl::Status ClearRegisteredDevices() = 0;

  // Returns the number of registered devices.
  virtual int GetNumberOfRegisteredDevices() const = 0;

  // Returns the application layer state of the device with the given address.
  virtual absl::StatusOr<EthercatApplicationLayerState>
  GetDeviceApplicationLayerState(
      const EthercatDeviceAddress& device_address) = 0;

  // Registers a PDO entry with the host for cyclic transfer.
  //
  // This should fail if called before `Connect` or after
  // `ConfigurePdoTransfer`.
  //
  // Different EtherCAT libraries have very different methods of configuring
  // the available PDOs and their properties, so it is left up to the
  // implementation to define the available PDOs, and map the registered
  // PDOs to the available PDOs using the CAN object address.
  absl::Status RegisterPdoEntry(const EthercatDeviceAddress& device_address,
                                const CanObjectInfoVariant& object_info,
                                PdoDirection direction);
  virtual absl::Status RegisterPdoEntry(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address, int pdo_data_size_bytes,
      PdoDirection direction) = 0;

  // Clears all registered PDOs, if this is possible in the current state.
  virtual absl::Status ClearRegisteredPdos() = 0;

  // Writes PDO data to mapped memory.
  //
  // Only signed and unsigned integers of length 8, 16, 32 and 64 bits or float
  // (32 bit) can be used as the template parameter.
  //
  // Depending on the implementation, this may fail if cyclic operation has not
  // begun, or if it is not called between `PreCycle` and `PostCycle`.
  template <typename T>
  absl::Status WritePdo(const EthercatDeviceAddress& device_address,
                        const CanObjectAddress& object_address, T value) {
    static_assert(internal::IsTypeInVariant<T, CanObjectValueVariant>::value,
                  "Invalid value type parameter for EthercatHost::WritePdo");
    return WritePdoFromVariant(device_address, object_address,
                               CanObjectValueVariant(value));
  }

  // Reads PDO data from mapped memory.
  //
  // Only signed and unsigned integers of length 8, 16, 32 and 64 bits or float
  // (32 bit) can be used as the template parameter.
  //
  // Depending on the implementation, this may fail if cyclic operation has not
  // begun, or if it is not called between `PreCycle` and `PostCycle`.
  template <typename T>
  absl::StatusOr<T> ReadPdo(const EthercatDeviceAddress& device_address,
                            const CanObjectAddress& object_address) {
    static_assert(internal::IsTypeInVariant<T, CanObjectValueVariant>::value,
                  "Invalid value type parameter for EthercatHost::ReadPdo");

    // Use a variant to signal which type we want to read to.
    CanObjectValueVariant output_variant(std::in_place_type<T>, 0);
    absl::Status status =
        ReadPdoToVariant(device_address, object_address, output_variant);
    if (!status.ok()) return status;

    // Ensure that the type of the variant hasn't changed. This is unlikely but
    // possible.
    if (std::holds_alternative<T>(output_variant)) {
      return std::get<T>(output_variant);
    } else {
      return absl::InternalError(absl::StrCat(
          "Invalid variant alternative after call to ReadPdoToVariant. Please "
          "check the concrete implementation of this method. Device address: ",
          EthercatDeviceAddressToString(device_address),
          " object address: ", object_address.ToString()));
    }
  }

  // Writes data via SDO.
  //
  // Only signed and unsigned integers of length 8, 16, 32 and 64 bits or float
  // (32 bit) can be used as the template parameter.
  //
  // This should only be called before cyclic operation has started.
  //
  // Returns:
  // - Failed precondition error: If there is no connection to the bus or if
  //   cyclic operation has begun.
  // - NotFoundError: If the device wasn't found on the bus (incorrect alias or
  //   bus position).
  // - AbortedError: If the SDO download (write) failed.
  template <typename T>
  absl::Status WriteSdo(const EthercatDeviceAddress& device_address,
                        const CanObjectAddress& object_address, T value) {
    static_assert(internal::IsTypeInVariant<T, CanObjectValueVariant>::value,
                  "Invalid value type parameter for EthercatHost::WriteSdo");
    return WriteSdoFromVariant(device_address, object_address,
                               CanObjectValueVariant(value));
  }

  // Reads data via SDO.
  //
  // This should only be called before cyclic operation has started, i.e.
  // SDO exchanges should happen before a call to Initialize().
  //
  // Returns:
  // - Failed precondition error: If there is no connection to the bus or if
  //   cyclic operation has begun.
  // - NotFoundError: If the device wasn't found on the bus (incorrect alias or
  //   bus position).
  // - AbortedError: If the SDO upload (read) failed.
  template <typename T>
  absl::StatusOr<T> ReadSdo(const EthercatDeviceAddress& device_address,
                            const CanObjectAddress& object_address) {
    static_assert(internal::IsTypeInVariant<T, CanObjectValueVariant>::value,
                  "Invalid value type parameter for EthercatHost::ReadSdo");

    // Use a variant to signal which type we want to read to.
    CanObjectValueVariant output_variant(std::in_place_type<T>, 0);
    absl::Status status =
        ReadSdoToVariant(device_address, object_address, output_variant);
    if (!status.ok()) return status;

    // Ensure that the type of the variant hasn't changed. This is unlikely but
    // possible.
    if (std::holds_alternative<T>(output_variant)) {
      return std::get<T>(output_variant);
    } else {
      return absl::InternalError(absl::StrCat(
          "Invalid variant alternative after call to ReadSdoToVariant. Please "
          "check the concrete implementation of this method. Device address: ",
          EthercatDeviceAddressToString(device_address),
          " object address: ", object_address.ToString()));
    }
  }

  // Returns a span to the PDO memory for an object at the given address.
  // This can be used for data which is not integer-valued, e.g. the binary
  // blob.
  //
  // This memory may be written to as well as read from, depending on the
  // implementation.
  virtual absl::StatusOr<ByteSpan> GetPdoBytes(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address) = 0;

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
  // until Finalize is called. Returns a null pointer in
  // case the EtherCAT host implementation does not support direct PDO memory
  // map access.
  virtual absl::StatusOr<const PdoMemoryMap* const> GetPdoMemoryMap() const;

  // Returns the working counter for the last-received EtherCAT datagram.
  //
  // Each EtherCAT datagram has an associated working counter, which starts at
  // zero when the datagram is sent out from the host, and incremented by a
  // certain amount by each device encountered on the bus, depending on the
  // success of the device's EtherCAT operations. For example, if the device was
  // set up to read and write PDO data from the datagram and it succeeds, it
  // will increment the working counter by 3, but if it only manages to read and
  // not write the data, it will increment the working counter by 1. This means
  // that when the datagram is received by the host at the end of the round
  // trip, the working counter can be inspected to understand if there were any
  // errors in the device EtherCAT operations.
  //
  // For more details, see the Beckhoff EtherCAT Slave Controller Hardware Data
  // Sheet, Section I - Technology, section 2.4 Working Counter.
  //
  // Depending on the implementation, this may fail if cyclic operation has not
  // begun, or if it is not called between `PreCycle` and `PostCycle`.
  virtual absl::StatusOr<int> GetWorkingCounter() const = 0;

  // Pre-cycle method. This should fetch new packets from the bus, filling their
  // contents into PDO memory so that they can be read using the `ReadPdo`
  // methods.
  virtual absl::Status PreCycle() = 0;

  // Post-cycle method. This should write the values in PDO memory into data
  // packets and queue / send them on the bus.
  virtual absl::Status PostCycle() = 0;

 private:
  // Methods to read and write PDOs using a variant to carry the information on
  // which type of data should be read or written. These must be specified by
  // each implementation.
  //
  // Implementations may choose to return errors if cyclic operation has not
  // begun, or if it is not called between `PreCycle` and `PostCycle`.
  virtual absl::Status WritePdoFromVariant(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address, CanObjectValueVariant value) = 0;

  // The alternative held by the passed variant will specify which type of data
  // to read. The variant should be modified in place such that it holds the
  // same alternative, but the value of the alternative is set to the data read
  // from the PDO.
  //
  // Implementations may choose to return errors if cyclic operation has not
  // begun, or if it is not called between `PreCycle` and `PostCycle`.
  virtual absl::Status ReadPdoToVariant(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address,
      CanObjectValueVariant& output) = 0;

  // The alternative held by the passed variant will specify which type of data
  // to read. The variant should be modified in place such that it holds the
  // same alternative, but the value of the alternative is set to the data read
  // from the SDO.
  virtual absl::Status ReadSdoToVariant(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address,
      CanObjectValueVariant& output) = 0;

  virtual absl::Status WriteSdoFromVariant(
      const EthercatDeviceAddress& device_address,
      const CanObjectAddress& object_address, CanObjectValueVariant value) = 0;
};

// Helper functions to read / write SDOs and PDOs based on a given CAN object
// info struct.
//
// These use deduction to make it easier to select the template parameter based
// on a CanObjectInfo instance, avoiding the need to do verbose things like:
// `host.ReadPdo<decltype(kStatuswordObjectInfo)::ObjectType>(
//    kStatuswordObjectInfo.address)`.
template <typename T>
absl::Status WriteCanObjectSdoToEthercatDevice(
    EthercatHost& ethercat_host, const EthercatDeviceAddress& device_address,
    const CanObjectInfo<T>& object_info, T value) {
  return ethercat_host.WriteSdo<T>(device_address, object_info.address, value);
}

template <typename T>
absl::StatusOr<T> ReadCanObjectSdoFromEthercatDevice(
    EthercatHost& ethercat_host, const EthercatDeviceAddress& device_address,
    const CanObjectInfo<T>& object_info) {
  return ethercat_host.ReadSdo<T>(device_address, object_info.address);
}

template <typename T>
absl::Status WriteCanObjectPdoToEthercatDevice(
    EthercatHost& ethercat_host, const EthercatDeviceAddress& device_address,
    const CanObjectInfo<T>& object_info, T value) {
  return ethercat_host.WritePdo<T>(device_address, object_info.address, value);
}

template <typename T>
absl::StatusOr<T> ReadCanObjectPdoFromEthercatDevice(
    EthercatHost& ethercat_host, const EthercatDeviceAddress& device_address,
    const CanObjectInfo<T>& object_info) {
  return ethercat_host.ReadPdo<T>(device_address, object_info.address);
}

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_ETHERCAT_HOST_H_
