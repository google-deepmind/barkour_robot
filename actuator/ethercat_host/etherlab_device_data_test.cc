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

#include "etherlab_device_data.h"

#include "canopen.h"
#include "ds402_objects.h"
#include "custom_objects.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/status/status.h"

namespace barkour {
namespace {

#ifndef EXPECT_OK
#define EXPECT_OK(expression) \
  EXPECT_EQ(expression.code(), absl::StatusCode::kOk)
#endif

TEST(GetEtherlabPdoByteLengthTest, KnownValues) {
  auto length = GetEtherlabPdoByteLength(kStatuswordObjectInfo.address);
  EXPECT_OK(length.status());
  EXPECT_EQ(*length, 2);
  length = GetEtherlabPdoByteLength(kPositionObjectInfo.address);
  EXPECT_OK(length.status());
  EXPECT_EQ(*length, 4);
}

TEST(GetEtherlabPdoByteLengthTest, BinaryBlobValues) {
  auto length = GetEtherlabPdoByteLength(kRxPdoBlobAddress);
  EXPECT_OK(length.status());
  EXPECT_EQ(*length, kBinaryBlobNumBytes);
  length = GetEtherlabPdoByteLength(kTxPdoBlobAddress);
  EXPECT_OK(length.status());
  EXPECT_EQ(*length, kBinaryBlobNumBytes);
}

TEST(GetEtherlabPdoByteLengthTest, PassingUnknownAddressReturnsNotFound) {
  EXPECT_EQ(
      GetEtherlabPdoByteLength(CanObjectAddress(0x9999, 0x00)).status().code(),
      absl::StatusCode::kNotFound);
}

TEST(GetEtherlabPdoByteLengthTest, PassingZeroAddressReturnsNotFound) {
  EXPECT_EQ(GetEtherlabPdoByteLength(CanObjectAddress(0, 0)).status().code(),
            absl::StatusCode::kInvalidArgument);
}

}  // namespace
}  // namespace barkour
