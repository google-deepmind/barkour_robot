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

#include "ds301_objects.h"

#include <cstdint>
#include <variant>

#include "gtest/gtest.h"
#include "absl/log/check.h"
#include "canopen.h"

namespace barkour {
namespace {

TEST(TestObjectDict, CheckObjectSizes) {
  for (const auto& entry : kDs301Objects) {
    if (const auto* entry_ptr = std::get_if<CanObjectInfo<uint32_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 4);
    } else if (const auto* entry_ptr =
                   std::get_if<CanObjectInfo<int32_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 4);
    } else if (const auto* entry_ptr =
                   std::get_if<CanObjectInfo<uint16_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 2);
    } else if (const auto* entry_ptr =
                   std::get_if<CanObjectInfo<int16_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 2);
    } else if (const auto* entry_ptr =
                   std::get_if<CanObjectInfo<int8_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 1);
    } else if (const auto* entry_ptr =
                   std::get_if<CanObjectInfo<uint8_t>>(&entry)) {
      CHECK_EQ(entry_ptr->size(), 1);
    }
  }
}

}  // namespace
}  // namespace barkour
