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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_ORDER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_ORDER_H_

namespace barkour {

// Describes the ordering of the phases.
//
// ABC ordering means that as the motor rotates in a positive direction
// (according to the encoder), the phases will align with the magnetic axis of
// the stator in the order .. -> A -> B -> C -> A -> B -> C -> ..., and similar
// for ACB ordering.
//
// Note that any other orderings (e.g. BAC) are equivalent to ABC or ACB by a
// shift of the starting point of the sequence.
enum class PhaseOrder {
  kAbc,
  kAcb,
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PHASE_ORDER_H_
