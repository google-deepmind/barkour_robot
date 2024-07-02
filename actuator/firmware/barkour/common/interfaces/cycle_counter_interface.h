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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_CYCLE_COUNTER_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_CYCLE_COUNTER_INTERFACE_H_

#include <cstdint>

namespace barkour {

// A generic interface exposing CPU cycle count.
class CycleCounterInterface {
 public:
  virtual ~CycleCounterInterface() = default;

  // Enables the cycle counter.
  //
  // The cycle count will only be incremented while the counter is enabled.
  virtual void Enable() = 0;

  // Disables the cycle counter.
  //
  // The cycle count will not be incremented while the counter is disabled.
  virtual void Disable() = 0;

  // Returns true if the cycle counter is enabled.
  virtual bool IsEnabled() const = 0;

  // Returns the cycle count.
  virtual uint32_t GetCount() const = 0;

  // Resets the CPU cycle count.
  virtual void Reset() = 0;

  // Blocks execution by polling for us microseconds.
  virtual void BlockForMicroseconds(uint32_t us) const = 0;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_CYCLE_COUNTER_INTERFACE_H_
