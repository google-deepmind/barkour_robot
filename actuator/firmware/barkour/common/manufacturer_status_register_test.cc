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

#include "actuator/firmware/barkour/common/manufacturer_status_register.h"

#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {

namespace {

TEST(ManufacturerStatusRegisterErrorCodesTest, BitwiseOperators) {
  ManufacturerStatusRegisterErrorCodes a =
      ManufacturerStatusRegisterErrorCodes(0x1d5c0398u);
  ManufacturerStatusRegisterErrorCodes b =
      ManufacturerStatusRegisterErrorCodes(0xf58bba66u);

  EXPECT_EQ(static_cast<uint32_t>(a | b),
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
  EXPECT_EQ(static_cast<uint32_t>(a & b),
            static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
  EXPECT_EQ(static_cast<uint32_t>(a ^ b),
            static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
  EXPECT_EQ(static_cast<uint32_t>(~a), ~static_cast<uint32_t>(a));
}

TEST(ManufacturerStatusRegisterErrorCodesTest, InPlaceBitwiseOperators) {
  ManufacturerStatusRegisterErrorCodes a =
      ManufacturerStatusRegisterErrorCodes(0x1d5c0398u);
  ManufacturerStatusRegisterErrorCodes b =
      ManufacturerStatusRegisterErrorCodes(0xf58bba66u);

  ManufacturerStatusRegisterErrorCodes target = a;
  target |= b;
  EXPECT_EQ(static_cast<uint32_t>(target),
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b));

  target = a;
  target &= b;
  EXPECT_EQ(static_cast<uint32_t>(target),
            static_cast<uint32_t>(a) & static_cast<uint32_t>(b));

  target = a;
  target ^= b;
  EXPECT_EQ(static_cast<uint32_t>(target),
            static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

}  // namespace

}  // namespace barkour
