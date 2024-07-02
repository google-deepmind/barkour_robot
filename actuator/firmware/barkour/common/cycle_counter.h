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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CYCLE_COUNTER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CYCLE_COUNTER_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/cycle_counter_interface.h"

namespace barkour {

// ARM Cortex-M Implementation of the CycleCounter interface.
//
// This implementation is implemented using  DWT register.
class CycleCounter : public CycleCounterInterface {
 public:
  // Singleton access to CycleCounter.
  //
  // On Cortex-M cores this CycleCounterInterface implementation depends upon
  // the memory-mapped DWT (Data Watchpoint and Trace) unit. Becuase only one
  // DWT unit exists, CycleCounter uses the singleton pattern.
  static CycleCounter& Get();

  // Enables the cycle counter.
  //
  // The cycle count will only be incremented while the counter is enabled.
  void Enable() override;

  // Disables the cycle counter.
  //
  // The cycle count will not be incremented while the counter is disabled.
  void Disable() override;

  // Returns true if the cycle counter is enabled.
  //
  // Warning: Because the the underlying DWT unit registers may be modified
  // directly by debugger tooling, it is not safe to assume that the
  // CycleCounter will continue to remained enabled/disabled after calls to
  // Enable()/Disable().
  bool IsEnabled() const override;

  // Returns the CPU cycle count.
  uint32_t GetCount() const override;

  // Resets the cycle counter.
  void Reset() override;

  // Blocks execution by polling for us microseconds.
  void BlockForMicroseconds(uint32_t us) const override;

 private:
  CycleCounter();
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_CYCLE_COUNTER_H_
