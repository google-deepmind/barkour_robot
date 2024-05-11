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

#include "gtest/gtest.h"

namespace barkour {
namespace {

TEST(EthercatDeviceAddressTest, ToStringWorksAsExpected) {
  EXPECT_EQ(EthercatDeviceAddressToString(EthercatBusPosition(10)),
            "EthercatDeviceAddress[bus_position = 10]");
  EXPECT_EQ(EthercatDeviceAddressToString(EthercatStationAlias(56)),
            "EthercatDeviceAddress[station_alias = 56]");
}

TEST(EthercatDeviceInfoTest, EqualityOperatorWorksAsExpected) {
  const EthercatDeviceInfo device_info = {
      .bus_position = EthercatBusPosition(0),
      .station_alias = EthercatStationAlias(1),
      .vendor_id = 2,
      .product_code = 3,
      .serial_number = 4,
      .name = "a_device",
  };

  EXPECT_EQ(device_info, device_info);
}

TEST(EthercatDeviceInfoTest, InequalityOperatorWorksAsExpected) {
  const EthercatDeviceInfo device_info = {
      .bus_position = EthercatBusPosition(0),
      .station_alias = EthercatStationAlias(1),
      .vendor_id = 2,
      .product_code = 3,
      .serial_number = 4,
      .name = "a_device",
  };

  EthercatDeviceInfo other = device_info;
  other.bus_position = EthercatBusPosition(100);
  EXPECT_NE(device_info, other);

  other = device_info;
  other.station_alias = EthercatStationAlias(200);
  EXPECT_NE(device_info, other);

  other = device_info;
  other.vendor_id = 234;
  EXPECT_NE(device_info, other);

  other = device_info;
  other.product_code = 952;
  EXPECT_NE(device_info, other);

  other = device_info;
  other.serial_number = 7425;
  EXPECT_NE(device_info, other);

  other = device_info;
  other.name = "other_device";
  EXPECT_NE(device_info, other);
}

TEST(EthercatDeviceInfoTest, ToStringWorksAsExpected) {
  const EthercatDeviceInfo device_info = {
      .bus_position = EthercatBusPosition(12),
      .station_alias = EthercatStationAlias(1),
      .vendor_id = 2,
      .product_code = 3,
      .serial_number = 4,
      .name = "🦿_device",
  };
  EXPECT_EQ(device_info.ToString(),
            "EthercatDeviceInfo[bus_position=12, station_alias=1, vendor_id=2, "
            "product_code=3, serial_number=4, name=🦿_device]");
}

struct EthercatApplicationLayerStateTestParams {
  EthercatApplicationLayerState state;
  std::string expected_string;
};

using EthercatApplicationLayerStateTest =
    ::testing::TestWithParam<EthercatApplicationLayerStateTestParams>;

INSTANTIATE_TEST_SUITE_P(
    ToStringTestPairs, EthercatApplicationLayerStateTest,
    testing::Values(
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kBoot, false}, "BOOT"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kInit, false}, "INIT"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kPreop, false}, "PREOP"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kSafeop, false}, "SAFEOP"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kOp, false}, "OP"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kUnknown, false}, "UNKNOWN"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState(100), false}, "UNKNOWN"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kBoot, true}, "BOOT + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kInit, true}, "INIT + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kPreop, true}, "PREOP + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kSafeop, true},
            "SAFEOP + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kOp, true}, "OP + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState::kUnknown, true},
            "UNKNOWN + ERROR"},
        EthercatApplicationLayerStateTestParams{
            {BaseEthercatApplicationLayerState(100), true},
            "UNKNOWN + ERROR"}));

TEST_P(EthercatApplicationLayerStateTest, ToStringWorksAsExpected) {
  EXPECT_EQ(GetParam().state.ToString(), GetParam().expected_string);
}

}  // namespace
}  // namespace barkour
