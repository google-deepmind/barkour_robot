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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_SAMPLE_SELECTION_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_SAMPLE_SELECTION_H_

namespace barkour {

// Defines which two of three phase currents are sampled on any sampling cycle.
//
// Only two of the three currents are sampled each cycle depending on the
// PWM duty cycle of each phase.
//
// The phase with the minimum ON time (maximum OFF time) is ignored
// as the on time sampling window may be two small to get a clean measurement.
// The value of the ignored phase is reconstructed from the other two
// assuming that a+b+c=0.
enum class PhaseSampleSelection {
  kAB,  // ignore C - sample A & B
  kAC,  // ignore B - sample A & C
  kBC,  // ignore A - sample B & C
  kABC  // Sample all
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_SAMPLE_SELECTION_H_
