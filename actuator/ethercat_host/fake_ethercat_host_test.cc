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

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "gmock/gmock.h"
#include "absl/status/statusor.h"
#include "canopen.h"
#include "ds301_objects.h"
#include "ds402_objects.h"
#include "constants.h"
#include "ethercat_host.h"
#include "types.h"
#include "gtest/gtest.h"
#include "absl/status/status.h"
#include "absl/types/span.h"

namespace barkour {
namespace {

#ifndef ASSERT_OK
#define ASSERT_OK(expression) \
  ASSERT_EQ(expression.code(), absl::StatusCode::kOk)
#endif

#ifndef EXPECT_OK
#define EXPECT_OK(expression) \
  EXPECT_EQ(expression.code(), absl::StatusCode::kOk)
#endif

EthercatDeviceInfo CreateDeviceInfoForAlias(uint16_t alias) {
  return EthercatDeviceInfo{
      .bus_position = EthercatBusPosition(0),
      .station_alias = EthercatStationAlias(alias),
      .vendor_id = 8665,
      .product_code = 105689,
      .serial_number = 2,
      .name = "device_0",
  };
}

TEST(FakeEthercatHostTest, CannotConnectTwice) {
  FakeEthercatHost host;
  EXPECT_OK(host.Connect());
  EXPECT_EQ(host.Connect().code(), absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CanAlwaysDisconnect) {
  FakeEthercatHost host;

  // Check we can disconnect before connection.
  EXPECT_OK(host.Disconnect());

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.Disconnect());

  // Check we can finalize twice.
  EXPECT_OK(host.Disconnect());
}

TEST(FakeEthercatHostTest, CannotConfigurePdosTwice) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  EXPECT_EQ(host.ConfigurePdoTransfer().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CannotStartCyclicOperationBeforeConfiguringPdos) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());

  EXPECT_EQ(host.StartCyclicOperation().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CanAlwaysStopCyclicOperation) {
  FakeEthercatHost host;

  // Check we can stop before connection.
  EXPECT_OK(host.StopCyclicOperation());

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());

  ASSERT_OK(host.StartCyclicOperation());

  ASSERT_OK(host.StopCyclicOperation());

  // Check we can stop twice.
  EXPECT_OK(host.StopCyclicOperation());
}

TEST(FakeEthercatHostTest, IsConnectedReportsCorrectValues) {
  FakeEthercatHost host;
  EXPECT_FALSE(host.IsConnected());

  ASSERT_OK(host.Connect());
  EXPECT_TRUE(host.IsConnected());

  ASSERT_OK(host.Disconnect());
  EXPECT_FALSE(host.IsConnected());
}

TEST(FakeEthercatHostTest, ArePdosConfiguredReportsCorrectValues) {
  FakeEthercatHost host;
  EXPECT_FALSE(host.ArePdosConfigured());

  ASSERT_OK(host.Connect());
  EXPECT_FALSE(host.ArePdosConfigured());

  ASSERT_OK(host.ConfigurePdoTransfer());
  EXPECT_TRUE(host.ArePdosConfigured());

  ASSERT_OK(host.StartCyclicOperation());
  EXPECT_TRUE(host.ArePdosConfigured());

  ASSERT_OK(host.StopCyclicOperation());
  EXPECT_FALSE(host.ArePdosConfigured());

  ASSERT_OK(host.Disconnect());
  EXPECT_FALSE(host.ArePdosConfigured());
}

TEST(FakeEthercatHostTest, HasCyclicOperationStartedReportsCorrectValues) {
  FakeEthercatHost host;
  EXPECT_FALSE(host.HasCyclicOperationStarted());

  ASSERT_OK(host.Connect());
  EXPECT_FALSE(host.HasCyclicOperationStarted());

  ASSERT_OK(host.ConfigurePdoTransfer());
  EXPECT_FALSE(host.HasCyclicOperationStarted());

  ASSERT_OK(host.StartCyclicOperation());
  EXPECT_TRUE(host.HasCyclicOperationStarted());

  ASSERT_OK(host.StopCyclicOperation());
  EXPECT_FALSE(host.HasCyclicOperationStarted());

  ASSERT_OK(host.Disconnect());
  EXPECT_FALSE(host.HasCyclicOperationStarted());
}

TEST(FakeEthercatHostTest,
     CannotRegisterDeviceForPdoTransferAfterConfiguringPdos) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());

  EXPECT_EQ(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(32)).code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CannotRegisterDeviceForPdoTransferTwice) {
  FakeEthercatHost host;
  EthercatDeviceAddress address(EthercatStationAlias(1));

  ASSERT_OK(host.RegisterDeviceForPdoTransfer(address));
  EXPECT_EQ(host.RegisterDeviceForPdoTransfer(address).code(),
                absl::StatusCode::kAlreadyExists);
}

TEST(FakeEthercatHostTest,
     CannotClearRegisteredDevicesAfterStartingCyclicOperation) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  EXPECT_EQ(host.ClearRegisteredDevices().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, HasDeviceBeenRegisteredWorksAsExpected) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);

  EXPECT_FALSE(host.HasDeviceBeenRegistered(device_address));

  ASSERT_OK(host.RegisterDeviceForPdoTransfer(device_address));
  EXPECT_TRUE(host.HasDeviceBeenRegistered(device_address));

  ASSERT_OK(host.ClearRegisteredDevices());
  EXPECT_FALSE(host.HasDeviceBeenRegistered(device_address));
}

TEST(FakeEthercatHostTest, GetNumberOfRegisteredDevicesReturnsZeroByDefault) {
  FakeEthercatHost host;
  EXPECT_EQ(host.GetNumberOfRegisteredDevices(), 0);
}

TEST(FakeEthercatHostTest, GetNumberOfRegisteredDevicesWorksAsExpected) {
  FakeEthercatHost host;
  EXPECT_OK(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(32)));
  EXPECT_OK(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(33)));

  EXPECT_EQ(host.GetNumberOfRegisteredDevices(), 2);
}

TEST(FakeEthercatHostTest, GetDeviceApplicationLayerStateWorksAsExpected) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());

  // Set some devices on the bus.
  std::vector<EthercatDeviceInfo> devices = {
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(0),
          .station_alias = EthercatStationAlias(245),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 2,
          .name = "device_0",
      },
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(300),
          .station_alias = EthercatStationAlias(0),
          .vendor_id = 64,
          .product_code = 8432,
          .serial_number = 3423567,
          .name = "device_1",
      },
  };
  ASSERT_OK(host.SetDevicesOnBus(devices));

  // Set an AL state for one of them.
  EthercatApplicationLayerState al_state = {
      .base_state = BaseEthercatApplicationLayerState::kSafeop,
      .error = true,
  };
  host.SetDeviceApplicationLayerState(devices[0].bus_position, al_state);

  // Device not on bus -> returns not found.
  EXPECT_EQ(host.GetDeviceApplicationLayerState(EthercatBusPosition(30))
                .status()
                .code(),
            absl::StatusCode::kNotFound);

  // Device on bus, no value set, cyclic operation not started -> PREOP default.
  absl::StatusOr<EthercatApplicationLayerState> reported_al_state =
      host.GetDeviceApplicationLayerState(devices[1].station_alias);
  EXPECT_OK(reported_al_state.status());
  EXPECT_EQ(reported_al_state->base_state,
            BaseEthercatApplicationLayerState::kPreop);
  EXPECT_FALSE(reported_al_state->error);

  // Value set should be reported.
  reported_al_state =
      host.GetDeviceApplicationLayerState(devices[0].bus_position);
  EXPECT_OK(reported_al_state.status());
  EXPECT_EQ(reported_al_state->base_state, al_state.base_state);
  EXPECT_EQ(reported_al_state->error, al_state.error);

  // Default should change after cyclic operation has started.
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  // Device on bus, no value set, cyclic operation started -> OP default.
  reported_al_state =
      host.GetDeviceApplicationLayerState(devices[1].station_alias);
  EXPECT_OK(reported_al_state.status());
  EXPECT_EQ(reported_al_state->base_state,
            BaseEthercatApplicationLayerState::kOp);
  EXPECT_FALSE(reported_al_state->error);

  // Value set explicitly should still be reported.
  reported_al_state =
      host.GetDeviceApplicationLayerState(devices[0].bus_position);
  EXPECT_OK(reported_al_state.status());
  EXPECT_EQ(reported_al_state->base_state, al_state.base_state);
  EXPECT_EQ(reported_al_state->error, al_state.error);
}

TEST(FakeEthercatHostTest, CannotRegisterPdoAfterConfiguringPdos) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());

  EXPECT_EQ(host.RegisterPdoEntry(EthercatBusPosition(32),
                                  CanObjectAddress(0x1234, 0x5), 4,
                                  PdoDirection::kTxDeviceToHost)
                .code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest,
     CannotClearRegisteredPdosAfterStartingCyclicOperation) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  EXPECT_EQ(host.ClearRegisteredPdos().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CanRegisterSamePdoEntryTwice) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);

  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
}

TEST(FakeEthercatHostTest, HasPdoBeenRegisteredWorksAsExpected) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));

  EXPECT_FALSE(host.HasPdoBeenRegistered(device_address, object_address));

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
  EXPECT_TRUE(host.HasPdoBeenRegistered(device_address, object_address));

  ASSERT_OK(host.ClearRegisteredPdos());
  EXPECT_FALSE(host.HasPdoBeenRegistered(device_address, object_address));
}

TEST(FakeEthercatHostTest, CanSetAndGetDevicesOnBus) {
  FakeEthercatHost host;

  std::vector<EthercatDeviceInfo> devices = {
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(0),
          .station_alias = EthercatStationAlias(245),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 2,
          .name = "device_0",
      },
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(300),
          .station_alias = EthercatStationAlias(0),
          .vendor_id = 64,
          .product_code = 8432,
          .serial_number = 3423567,
          .name = "device_1",
      },
  };

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.SetDevicesOnBus(devices));
  auto devices_on_bus = host.GetDevicesOnBus();
  ASSERT_OK(devices_on_bus.status());
  ASSERT_EQ(devices_on_bus->size(), devices.size());
  for (size_t i = 0; i < devices_on_bus->size(); ++i) {
    EXPECT_EQ((*devices_on_bus)[i], devices[i]);
  }
}

template <typename T>
class ReadAndWritePdoTest : public testing::Test {};

using SupportedIntegerTypes =
    ::testing::Types<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t,
                     int64_t, uint64_t, float>;

TYPED_TEST_SUITE(ReadAndWritePdoTest, SupportedIntegerTypes);

TYPED_TEST(ReadAndWritePdoTest, CannotWriteBeforeCyclicOperation) {
  FakeEthercatHost host;
  TypeParam written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  CanObjectAddress object_address(0x1234, 0x5);

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  EXPECT_EQ(host.WritePdo(device_address, object_address, written_value).code(),
            absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotWriteAfterCyclicOperation) {
  FakeEthercatHost host;
  TypeParam written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.StopCyclicOperation());

  EXPECT_EQ(host.WritePdo(device_address, object_address, written_value).code(),
            absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotWriteWithoutRegistering) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  TypeParam written_value = 19;
  EXPECT_EQ(host.WritePdo(device_address, object_address, written_value).code(),
            absl::StatusCode::kNotFound);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadBeforeConnection) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, object_address).status().code(),
      absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadBeforeCyclicOperation) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, object_address).status().code(),
      absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadAfterDisconnection) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.Disconnect());

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, object_address).status().code(),
      absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadAfterStoppingCyclicOperation) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.StopCyclicOperation());

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, object_address).status().code(),
      absl::StatusCode::kFailedPrecondition);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadWithoutRegistering) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, object_address).status().code(),
      absl::StatusCode::kNotFound);
}

TYPED_TEST(ReadAndWritePdoTest, CanWriteAndReadSameType) {
  FakeEthercatHost host;
  TypeParam written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.WritePdo(device_address, object_address, written_value));

  auto read_value = host.ReadPdo<TypeParam>(device_address, object_address);
  ASSERT_OK(read_value.status());
  EXPECT_EQ(*read_value, written_value);
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadWrongType) {
  FakeEthercatHost host;
  TypeParam written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.WritePdo(device_address, object_address, written_value));

  // Find a different type to use for the read function.
  if (std::is_signed<TypeParam>::value) {
    EXPECT_EQ(
        host.ReadPdo<uint8_t>(device_address, object_address).status().code(),
        absl::StatusCode::kNotFound);
  } else {
    EXPECT_EQ(
        host.ReadPdo<int8_t>(device_address, object_address).status().code(),
        absl::StatusCode::kNotFound);
  }
}

TYPED_TEST(ReadAndWritePdoTest, CannotReadFromWrongAddress) {
  FakeEthercatHost host;
  TypeParam written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);

  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.WritePdo(device_address, object_address, written_value));

  EXPECT_EQ(
      host.ReadPdo<TypeParam>(device_address, CanObjectAddress(0x2345, 0x6))
          .status()
          .code(),
      absl::StatusCode::kNotFound);
  EXPECT_EQ(host.ReadPdo<TypeParam>(EthercatBusPosition(34), object_address)
                .status()
                .code(),
            absl::StatusCode::kNotFound);
}

TEST(ReadSdoTest, ReadSdo) {
  FakeEthercatHost host;
  std::vector<EthercatDeviceInfo> devices = {
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(0),
          .station_alias = EthercatStationAlias(245),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 2,
          .name = "device_0",
      },
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(300),
          .station_alias = EthercatStationAlias(0),
          .vendor_id = 64,
          .product_code = 8432,
          .serial_number = 3423567,
          .name = "device_1",
      },
  };

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.Connect());

  EXPECT_OK(host.ReadSdo<uint32_t>(
      EthercatBusPosition(300),
      CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo)).status());
  EXPECT_EQ(*host.ReadSdo<uint32_t>(
                EthercatBusPosition(300),
                CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo)),
            (uint32_t)3423567);
  EXPECT_EQ(*host.ReadSdo<uint32_t>(EthercatBusPosition(300),
                                    CanObjectAddressFromCanObjectInfo(
                                        kGearMotorRevolutionsObjectInfo)),
            (uint32_t)5);
  EXPECT_EQ(*host.ReadSdo<int8_t>(
                EthercatBusPosition(300),
                CanObjectAddressFromCanObjectInfo(kModesObjectInfo)),
            (int8_t)0);
  EXPECT_EQ(host.ReadSdo<uint32_t>(
                    EthercatBusPosition(301),
                    CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo))
                .status()
                .code(),
            absl::StatusCode::kNotFound);
  EXPECT_EQ(host.ReadSdo<int32_t>(EthercatBusPosition(300),
                                  CanObjectAddress(0xFFFF, 123))
                .status()
                .code(),
            absl::StatusCode::kAborted);
}

TEST(ReadSdoTest, ReadSdoFailIfNotConnected) {
  FakeEthercatHost host;

  EXPECT_EQ(host.ReadSdo<uint32_t>(
                    EthercatBusPosition(301),
                    CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo))
                .status()
                .code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(ReadSdoTest, ReadSdoFailIfCyclicOperationStarted) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  EXPECT_EQ(host.ReadSdo<uint32_t>(
                    EthercatBusPosition(301),
                    CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo))
                .status()
                .code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(ReadSdoTest, WriteSdoFailIfNotConnected) {
  FakeEthercatHost host;

  EXPECT_EQ(
      host.WriteSdo<uint32_t>(
              EthercatBusPosition(301),
              CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo), 32)
          .code(),
      absl::StatusCode::kFailedPrecondition);
}

TEST(ReadSdoTest, WriteSdoFailIfCyclicOperationStarted) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  EXPECT_EQ(
      host.WriteSdo<uint32_t>(
              EthercatBusPosition(301),
              CanObjectAddressFromCanObjectInfo(kSerialNumberObjectInfo), 32)
          .code(),
      absl::StatusCode::kFailedPrecondition);
}

TEST(ReadSdoTest, WriteSdo) {
  FakeEthercatHost host;
  std::vector<EthercatDeviceInfo> devices = {
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(0),
          .station_alias = EthercatStationAlias(245),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 2,
          .name = "device_0",
      },
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(300),
          .station_alias = EthercatStationAlias(0),
          .vendor_id = 64,
          .product_code = 8432,
          .serial_number = 3423567,
          .name = "device_1",
      },
  };

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.Connect());

  EXPECT_EQ(host.WriteSdo<uint32_t>(EthercatBusPosition(300),
                                    CanObjectAddressFromCanObjectInfo(
                                        kTorqueSetpointObjectInfo),
                                    -32199)
                .code(),
            absl::StatusCode::kAborted);
  EXPECT_OK(host.WriteSdo<int16_t>(
      EthercatBusPosition(300),
      CanObjectAddressFromCanObjectInfo(kTorqueSetpointObjectInfo), -32199));
  auto result = host.ReadSdo<int16_t>(
      EthercatBusPosition(300),
      CanObjectAddressFromCanObjectInfo(kTorqueSetpointObjectInfo));
  ASSERT_OK(result.status());
  EXPECT_EQ(*result, -32199);
}

TEST(FakeEthercatHostTest, WritingPdoDataOverwritesOldType) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));

  int8_t old_value = 8;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.WritePdo(device_address, object_address, old_value));

  uint32_t new_value = 10004;
  ASSERT_OK(host.WritePdo(device_address, object_address, new_value));

  auto read_value = host.ReadPdo<uint32_t>(device_address, object_address);
  ASSERT_OK(read_value.status());
  EXPECT_EQ(*read_value, new_value);
  }

TEST(FakeEthercatHostTest, WritingPdoDataToUnknownDeviceFails) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {
      EthercatDeviceInfo{
          .bus_position = EthercatBusPosition(0),
          .station_alias = EthercatStationAlias(102),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 2,
          .name = "device_0",
      },
  };

  ASSERT_OK(host.SetDevicesOnBus(devices));
  EXPECT_EQ(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost)
                .code(),
            absl::StatusCode::kNotFound);
}

TEST(FakeEthercatHostTest, CannotGetPdoBytesBeforeCyclicOperationStarts) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  EXPECT_EQ(
      host.GetPdoBytes(EthercatBusPosition(1), CanObjectAddress(0x1234, 0x5))
          .status()
          .code(),
      absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CanWriteAndReadPdoBytes) {
  FakeEthercatHost host;
  std::array<std::byte, 5> pdo_data = {
      std::byte(0), std::byte(2), std::byte(34), std::byte(6), std::byte(205)};
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.SetPdoBytes(device_address, object_address,
                             absl::MakeSpan(pdo_data)));

  auto returned_span =
      host.GetPdoBytes(device_address, object_address);
  ASSERT_OK(returned_span.status());
  EXPECT_EQ(returned_span->size(), pdo_data.size());
  for (size_t i = 0; i < pdo_data.size(); ++i) {
    EXPECT_EQ((*returned_span)[i], pdo_data[i]);
  }
}

TEST(FakeEthercatHostTest, CannotGetPdoBytesWhenObjectHasAnotherType) {
  FakeEthercatHost host;
  int16_t written_value = 19;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.WritePdo(device_address, object_address, written_value));

  EXPECT_EQ(host.GetPdoBytes(device_address, object_address).status().code(),
            absl::StatusCode::kNotFound);
}

TEST(FakeEthercatHostTest, CannotGetPdoBytesWrongAddress) {
  FakeEthercatHost host;
  EthercatDeviceAddress device_address = EthercatStationAlias(101);
  CanObjectAddress object_address(0x1234, 0x5);
  std::vector<EthercatDeviceInfo> devices = {CreateDeviceInfoForAlias(101)};

  ASSERT_OK(host.SetDevicesOnBus(devices));
  std::array<std::byte, 5> pdo_data = {
      std::byte(0), std::byte(2), std::byte(34), std::byte(6), std::byte(205)};

  ASSERT_OK(host.RegisterPdoEntry(device_address, object_address, 4,
                                  PdoDirection::kTxDeviceToHost));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.SetPdoBytes(device_address, object_address,
                             absl::MakeSpan(pdo_data)));

  EXPECT_EQ(host.GetPdoBytes(device_address, CanObjectAddress(0x2345, 0x6))
                .status()
                .code(),
            absl::StatusCode::kNotFound);
  EXPECT_EQ(host.GetPdoBytes(EthercatBusPosition(456), object_address)
                .status()
                .code(),
            absl::StatusCode::kNotFound);
}

TEST(FakeEthercatHostTest, CannotGetWorkingCounterBeforeCyclicOperationStarts) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());

  EXPECT_EQ(host.GetWorkingCounter().status().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, CannotGetWorkingCounterAfterCyclicOperationStops) {
  FakeEthercatHost host;
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());
  ASSERT_OK(host.StopCyclicOperation());

  EXPECT_EQ(host.GetWorkingCounter().status().code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST(FakeEthercatHostTest, DefaultWorkingCounterValue) {
  FakeEthercatHost host;
  ASSERT_OK(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(32)));
  ASSERT_OK(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(33)));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  auto working_counter = host.GetWorkingCounter();
  ASSERT_OK(working_counter.status());
  EXPECT_EQ(*working_counter,
            2 * kEthercatWorkingCounterIncrementReadWriteCommandSuccess);
}

TEST(FakeEthercatHostTest, CanSetWorkingCounterValue) {
  FakeEthercatHost host;
  ASSERT_OK(host.RegisterDeviceForPdoTransfer(EthercatBusPosition(33)));
  ASSERT_OK(host.Connect());
  ASSERT_OK(host.ConfigurePdoTransfer());
  ASSERT_OK(host.StartCyclicOperation());

  int new_working_counter = 6343;
  host.SetWorkingCounter(new_working_counter);

  auto working_counter = host.GetWorkingCounter();
  ASSERT_OK(working_counter.status());
  EXPECT_EQ(*working_counter, new_working_counter);
}

}  // namespace
}  // namespace barkour
