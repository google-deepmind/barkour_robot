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

#include "canopen.h"

#include <cstdint>
#include <limits>
#include <variant>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/status/status.h"

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

TEST(TestCanObjectAddress, EqualityOperatorsWorkAsExpected) {
  EXPECT_EQ(CanObjectAddress(0x320f, 0x02), CanObjectAddress(0x320f, 0x02));
  EXPECT_NE(CanObjectAddress(0x1a04, 0x00), CanObjectAddress(0x1a04, 0x01));
  EXPECT_NE(CanObjectAddress(0x1a04, 0x00), CanObjectAddress(0x1a02, 0x00));
}

TEST(TestCanObjectAddress, ToStringProducesCorrectOutput) {
  EXPECT_EQ(CanObjectAddress(0x50f, 0x02).ToString(),
            "Index: 0x050f, Subindex: 0x02.");
  EXPECT_EQ(CanObjectAddress(0x1000, 0).ToString(),
            "Index: 0x1000, Subindex: 0x00.");
}

TEST(TestCanObjectAddressWithId, EqualityOperatorsWorkAsExpected) {
  EXPECT_EQ(CanObjectAddressWithId(0x05, 0x320f, 0x02),
            CanObjectAddressWithId(0x05, 0x320f, 0x02));
  EXPECT_NE(CanObjectAddressWithId(0x1d, 0x1a04, 0x00),
            CanObjectAddressWithId(0x1d, 0x1a04, 0x01));
  EXPECT_NE(CanObjectAddressWithId(0x1d, 0x1a04, 0x00),
            CanObjectAddressWithId(0x1d, 0x1a02, 0x00));
  EXPECT_NE(CanObjectAddressWithId(0x1d, 0x1a04, 0x00),
            CanObjectAddressWithId(0x1a, 0x1a04, 0x01));
}

TEST(TestCanObjectAddressWithId, ToStringProducesCorrectOutput) {
  EXPECT_EQ(CanObjectAddressWithId(0xd, 0x50f, 0x02).ToString(),
            "ID: 0x0d, Index: 0x050f, Subindex: 0x02.");
  EXPECT_EQ(CanObjectAddressWithId(0, 0x1000, 0).ToString(),
            "ID: 0x00, Index: 0x1000, Subindex: 0x00.");
}

template <typename T>
class TestCanObjectInfo : public testing::Test {};

using SupportedIntegerTypes =
    ::testing::Types<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t,
                     int64_t, uint64_t, float>;

TYPED_TEST_SUITE(TestCanObjectInfo, SupportedIntegerTypes);

TYPED_TEST(TestCanObjectInfo, Cast) {
  EXPECT_EQ(
      (CanObjectInfo<TypeParam>::Cast(std::numeric_limits<TypeParam>::max())),
      std::numeric_limits<TypeParam>::max());
  EXPECT_EQ(
      (CanObjectInfo<TypeParam>::Cast(std::numeric_limits<TypeParam>::min())),
      std::numeric_limits<TypeParam>::min());

  EXPECT_EQ((CanObjectInfo<TypeParam>::Cast(0)), 0);
  EXPECT_EQ((CanObjectInfo<TypeParam>::Cast(1)), 1);
}

TYPED_TEST(TestCanObjectInfo, ValueFromVariant) {
  std::variant<TypeParam> input =
      TypeParam{std::numeric_limits<TypeParam>::max()};
  auto value = CanObjectInfo<TypeParam>::ValueFromVariant(input);
  ASSERT_OK(value.status());
  EXPECT_EQ(*value, std::numeric_limits<TypeParam>::max());
}

TEST(TestCanObjectInfo, ErrorIfValueFromIncorrectVariant) {
  std::variant<int32_t, uint64_t> input =
      int32_t{std::numeric_limits<int32_t>::min()};
  EXPECT_EQ(CanObjectInfo<uint64_t>::ValueFromVariant(input).status().code(),
            absl::StatusCode::kInvalidArgument);
}

TYPED_TEST(TestCanObjectInfo, GetSize) {
  EXPECT_EQ(CanObjectSizeFromCanObjectInfo(CanObjectInfo<TypeParam>(0x20, 0x9)),
            sizeof(TypeParam));
  CanObjectInfoVariant variant = CanObjectInfo<TypeParam>(0x20, 0x9);
  EXPECT_EQ(CanObjectSizeFromCanObjectInfo(variant), sizeof(TypeParam));
}

TYPED_TEST(TestCanObjectInfo, GetAddress) {
  auto object_info = CanObjectInfo<TypeParam>(0x20, 0x9);
  EXPECT_EQ(CanObjectAddressFromCanObjectInfo(object_info).index,
            CanObjectIndexType(0x20));
  EXPECT_EQ(CanObjectAddressFromCanObjectInfo(object_info).subindex,
            CanObjectSubindexType(0x9));
  CanObjectInfoVariant variant = object_info;
  EXPECT_EQ(CanObjectAddressFromCanObjectInfo(variant).index,
            CanObjectIndexType(0x20));
  EXPECT_EQ(CanObjectAddressFromCanObjectInfo(variant).subindex,
            CanObjectSubindexType(0x9));
}


}  // namespace
}  // namespace barkour
