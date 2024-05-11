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

#include <string>
#include <variant>

#include "absl/strings/str_format.h"

namespace barkour {

bool CanObjectAddress::operator==(const CanObjectAddress& other) const {
  return index == other.index && subindex == other.subindex;
}

bool CanObjectAddress::operator!=(const CanObjectAddress& other) const {
  return !operator==(other);
}

std::string CanObjectAddress::ToString() const {
  return absl::StrFormat("Index: 0x%04x, Subindex: 0x%02x.", index, subindex);
}

bool CanObjectAddressWithId::operator==(
    const CanObjectAddressWithId& other) const {
  return id == other.id && index == other.index && subindex == other.subindex;
}

bool CanObjectAddressWithId::operator!=(
    const CanObjectAddressWithId& other) const {
  return !operator==(other);
}

std::string CanObjectAddressWithId::ToString() const {
  return absl::StrFormat("ID: 0x%02x, Index: 0x%04x, Subindex: 0x%02x.", id,
                         index, subindex);
}

const CanObjectAddress& CanObjectAddressFromCanObjectInfo(
    const CanObjectInfoVariant& object_info) {
  return std::visit([](const auto& object_info) -> const CanObjectAddress& {
        return CanObjectAddressFromCanObjectInfo(object_info);
      },
      object_info);
}

int CanObjectSizeFromCanObjectInfo(const CanObjectInfoVariant& object_info) {
  return std::visit([](const auto& object_info) -> int {
        return CanObjectSizeFromCanObjectInfo(object_info);
      },
      object_info);
}

}  // namespace barkour
