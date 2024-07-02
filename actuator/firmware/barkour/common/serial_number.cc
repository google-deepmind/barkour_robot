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

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/shared_memory.h"
#include "pw_bytes/span.h"
#include "pw_crypto/sha256.h"

namespace barkour {

namespace {

constexpr uint8_t kStm32UniqueIdNumWords = 3;

uint64_t Get64BitSerialNumberImpl() {
  std::array<std::byte, pw::crypto::sha256::kDigestSizeBytes> output_buffer;
  pw::ByteSpan output_span(output_buffer);

  std::array<std::byte, sizeof(uint32_t) * kStm32UniqueIdNumWords> input_buffer;
  memcpy(input_buffer.data(),
         reinterpret_cast<std::byte*>(kUidWord0Address),
         sizeof(uint32_t));
  memcpy(input_buffer.data() + sizeof(uint32_t),
         reinterpret_cast<std::byte*>(kUidWord1Address),
         sizeof(uint32_t));
  memcpy(input_buffer.data() + 2 * sizeof(uint32_t),
         reinterpret_cast<std::byte*>(kUidWord2Address),
         sizeof(uint32_t));

  pw::ConstByteSpan input_span(input_buffer);

  pw::crypto::sha256::Hash(input_span, output_span);
  return *(reinterpret_cast<uint64_t*>(output_buffer.data()));
}

}  // namespace

uint64_t Get64BitSerialNumber() {
  static uint64_t serial = Get64BitSerialNumberImpl();
  return serial;
}

uint32_t Get32BitSerialNumber() { return Get64BitSerialNumber() >> 32; }

}  // namespace barkour
