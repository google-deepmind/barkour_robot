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

#include "actuator/firmware/barkour/common/serial_number.h"

#include <cstdint>

#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(SerialNumberTest, TestSerialNumberNonZero) {
  EXPECT_NE(Get64BitSerialNumber(), static_cast<uint64_t>(0));
}

TEST(SerialNumberTest, TestGet64BitSerialNumberRepeatedlyReturnsSameValue) {
  uint64_t serial_first = Get64BitSerialNumber();
  uint64_t serial_second = Get64BitSerialNumber();
  EXPECT_EQ(serial_first, serial_second);
}

TEST(SerialNumberTest, TestGet32BitSerialNumberIsCorrectlyTruncated) {
  EXPECT_EQ(Get32BitSerialNumber(), Get64BitSerialNumber() >> 32);
}

}  // namespace
}  // namespace barkour
