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

#include <cstdint>

#include "absl/types/span.h"
#include "canopen.h"
#include "types.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

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

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgReferee;
using ::testing::Types;
using ::testing::VariantWith;

class MockEthercatHost : public EthercatHost {
 public:
  MOCK_METHOD(absl::Status, WritePdoFromVariant,
              (const EthercatDeviceAddress& device_address,
               const CanObjectAddress& object_addres,
               CanObjectValueVariant value),
              (override));
  MOCK_METHOD(absl::Status, ReadPdoToVariant,
              (const EthercatDeviceAddress& device_address,
               const CanObjectAddress& object_addres,
               CanObjectValueVariant& value),
              (override));
  MOCK_METHOD(absl::Status, ReadSdoToVariant,
              (const EthercatDeviceAddress& device_address,
               const CanObjectAddress& object_addres,
               CanObjectValueVariant& value),
              (override));
  MOCK_METHOD(absl::Status, WriteSdoFromVariant,
              (const EthercatDeviceAddress& device_address,
               const CanObjectAddress& object_addres,
               CanObjectValueVariant value),
              (override));

  // Unused mocks, required to override pure virtual interface methods.
  MOCK_METHOD(absl::StatusOr<absl::Span<const EthercatDeviceInfo>>,
              GetDevicesOnBus, (), (override));
  MOCK_METHOD(absl::Status, RegisterDeviceForPdoTransfer,
              (const EthercatDeviceAddress&), (override));
  MOCK_METHOD(absl::Status, ClearRegisteredDevices, (), (override));
  MOCK_METHOD(int, GetNumberOfRegisteredDevices, (), (const, override));
  MOCK_METHOD(absl::StatusOr<EthercatApplicationLayerState>,
              GetDeviceApplicationLayerState, (const EthercatDeviceAddress&),
              (override));
  MOCK_METHOD(absl::Status, RegisterPdoEntry,
              (const EthercatDeviceAddress&, const CanObjectAddress&, int,
               const PdoDirection direction),
              (override));
  MOCK_METHOD(absl::Status, ClearRegisteredPdos, (), (override));
  MOCK_METHOD(absl::StatusOr<ByteSpan>, GetPdoBytes,
              (const EthercatDeviceAddress&, const CanObjectAddress&),
              (override));
  MOCK_METHOD(absl::StatusOr<int>, GetWorkingCounter, (), (const, override));
  MOCK_METHOD(absl::Status, PreCycle, (), (override));
  MOCK_METHOD(absl::Status, PostCycle, (), (override));
};

TEST(EthercatHostTest, ConnectDefaultImplReturnsOkStatus) {
  MockEthercatHost host;
  EXPECT_OK(host.Connect());
}

TEST(EthercatHostTest, DisconnectDefaultImplReturnsOkStatus) {
  MockEthercatHost host;
  EXPECT_OK(host.Disconnect());
}

TEST(EthercatHostTest, ConfigurePdoTransferDefaultImplReturnsOkStatus) {
  MockEthercatHost host;
  EXPECT_OK(host.ConfigurePdoTransfer());
}

TEST(EthercatHostTest, StartCyclicOperationDefaultImplReturnsOkStatus) {
  MockEthercatHost host;
  EXPECT_OK(host.StartCyclicOperation());
}

TEST(EthercatHostTest, StopCyclicOperationDefaultImplReturnsOkStatus) {
  MockEthercatHost host;
  EXPECT_OK(host.StopCyclicOperation());
}

template <typename T>
class CanObjectValueTest : public testing::Test {};

using CanObjectValueTypes = Types<int8_t, uint8_t, int16_t, uint16_t, int32_t,
                                  uint32_t, int64_t, uint64_t, float>;
TYPED_TEST_SUITE(CanObjectValueTest, CanObjectValueTypes);

TYPED_TEST(CanObjectValueTest, CanReadValues) {
  MockEthercatHost host;

  TypeParam value_to_read = 7;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);

  EXPECT_CALL(host, ReadPdoToVariant(device_address, object_address, _))
      .WillOnce(
          DoAll(SetArgReferee<2>(value_to_read), Return(absl::OkStatus())));

  auto value_from_pdo = host.ReadPdo<TypeParam>(device_address, object_address);
  ASSERT_OK(value_from_pdo.status());
  EXPECT_EQ(*value_from_pdo, value_to_read);
}

TYPED_TEST(CanObjectValueTest, CanReadSdoValues) {
  MockEthercatHost host;

  TypeParam value_to_read = 7;
  EthercatDeviceAddress bus_position = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);

  EXPECT_CALL(host, ReadSdoToVariant(bus_position, object_address, _))
      .WillOnce(
          DoAll(SetArgReferee<2>(value_to_read), Return(absl::OkStatus())));

  auto value_from_sdo = host.ReadSdo<TypeParam>(bus_position, object_address);
  ASSERT_OK(value_from_sdo.status());
  EXPECT_EQ(*value_from_sdo, value_to_read);
}

TYPED_TEST(CanObjectValueTest, ReadPdoErrorsArePropagated) {
  MockEthercatHost host;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);
  absl::Status error = absl::FailedPreconditionError("an error");

  EXPECT_CALL(host, ReadPdoToVariant(device_address, object_address, _))
      .WillOnce(Return(error));

  EXPECT_EQ(host.ReadPdo<TypeParam>(device_address, object_address).status(),
            error);
}

TYPED_TEST(CanObjectValueTest, CanWriteValues) {
  MockEthercatHost host;

  TypeParam written_value = 7;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);

  EXPECT_CALL(host, WritePdoFromVariant(device_address, object_address,
                                        VariantWith<TypeParam>(written_value)))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(
      host.WritePdo<TypeParam>(device_address, object_address, written_value));
}

TYPED_TEST(CanObjectValueTest, CanWriteSdoValues) {
  MockEthercatHost host;

  TypeParam written_value = 7;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);

  EXPECT_CALL(host, WriteSdoFromVariant(device_address, object_address,
                                        VariantWith<TypeParam>(written_value)))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(
      host.WriteSdo<TypeParam>(device_address, object_address, written_value));
}

TYPED_TEST(CanObjectValueTest, WritePdoErrorsArePropagated) {
  MockEthercatHost host;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectAddress object_address(0x1020, 0x30);
  absl::Status error = absl::FailedPreconditionError("an error");

  TypeParam value_to_write = 0;

  EXPECT_CALL(host, WritePdoFromVariant(device_address, object_address,
                                        VariantWith<TypeParam>(value_to_write)))
      .WillOnce(Return(error));

  EXPECT_EQ(
      host.WritePdo<TypeParam>(device_address, object_address, value_to_write),
      error);
}

TEST(WriteCanObjectSdoToEthercatDeviceTest, ValueIsWrittenCorrectly) {
  MockEthercatHost host;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectInfo<int16_t> object_info(0x1020, 0x30);
  int16_t written_value = 5453;

  EXPECT_CALL(host, WriteSdoFromVariant(device_address, object_info.address,
                                        VariantWith<int16_t>(written_value)))
      .WillOnce(Return(absl::OkStatus()));

  ASSERT_OK(WriteCanObjectSdoToEthercatDevice(host, device_address, object_info,
                                              written_value));
}

TEST(ReadCanObjectSdoFromEthercatDeviceTest, ValueIsReadCorrectly) {
  MockEthercatHost host;

  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectInfo<int16_t> object_info(0x1020, 0x30);
  int16_t value_to_read = 5453;

  EXPECT_CALL(host, ReadSdoToVariant(device_address, object_info.address, _))
      .WillOnce(
          DoAll(SetArgReferee<2>(value_to_read), Return(absl::OkStatus())));

  auto value_from_sdo =
      host.ReadSdo<int16_t>(device_address, object_info.address);
  ASSERT_OK(value_from_sdo.status());
  EXPECT_EQ(*value_from_sdo, value_to_read);
}

TEST(WriteCanObjectPdoToEthercatDeviceTest, ValueIsWrittenCorrectly) {
  MockEthercatHost host;
  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectInfo<int16_t> object_info(0x1020, 0x30);
  int16_t written_value = 5453;

  EXPECT_CALL(host, WritePdoFromVariant(device_address, object_info.address,
                                        VariantWith<int16_t>(written_value)))
      .WillOnce(Return(absl::OkStatus()));

  ASSERT_OK(WriteCanObjectPdoToEthercatDevice(host, device_address, object_info,
                                              written_value));
}

TEST(ReadCanObjectPdoFromEthercatDeviceTest, ValueIsReadCorrectly) {
  MockEthercatHost host;

  EthercatDeviceAddress device_address = EthercatBusPosition(56);
  CanObjectInfo<int16_t> object_info(0x1020, 0x30);
  int16_t value_to_read = 5453;

  EXPECT_CALL(host, ReadPdoToVariant(device_address, object_info.address, _))
      .WillOnce(
          DoAll(SetArgReferee<2>(value_to_read), Return(absl::OkStatus())));

  auto value_from_pdo =
      ReadCanObjectPdoFromEthercatDevice(host, device_address, object_info);
  ASSERT_OK(value_from_pdo.status());
  EXPECT_EQ(*value_from_pdo, value_to_read);
}

}  // namespace
}  // namespace barkour
