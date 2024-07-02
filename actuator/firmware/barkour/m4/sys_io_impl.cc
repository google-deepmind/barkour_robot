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

// Implementation of low-level methods for pw::sys_io.
//
// Many of the following functions are stubbed out, returning Unimplemented
// status. These functions are never called.
//
// ReadBytes is called, however, support for efficient reading from the
// UART is presently not implemented.

#include <cstddef>
#include <cstdio>
#include <string_view>

#include "pw_bytes/span.h"
#include "pw_status/status.h"
#include "pw_status/status_with_size.h"
#include "pw_sys_io/sys_io.h"

namespace pw::sys_io {

pw::Status ReadByte(std::byte* dest) { return Status::Unimplemented(); }

pw::Status WriteByte(std::byte b) { return Status::Unimplemented(); }

pw::StatusWithSize WriteLine(std::string_view s) {
  return StatusWithSize(Status::Unimplemented(), 0);
}

StatusWithSize ReadBytes(ByteSpan dest) {
  return StatusWithSize(Status::Unimplemented(), 0);
}

pw::StatusWithSize WriteBytes(ConstByteSpan src) {
  return StatusWithSize(fwrite(src.data(), 1, src.size_bytes(), stdout));
}

}  // namespace pw::sys_io
