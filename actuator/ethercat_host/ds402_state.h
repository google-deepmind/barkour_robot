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

#ifndef BARKOUR_ROBOT_HARDWARE_COMMON_DS402_STATE_H_
#define BARKOUR_ROBOT_HARDWARE_COMMON_DS402_STATE_H_

// Utility functions for managing a DS402 finite state machine.

#include <cstdint>

#include "absl/strings/string_view.h"

namespace barkour {

enum class Ds402State : uint8_t {
  kNotReady = 0,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStop,
  kFaultReaction,
  kFault,
};

// Returns a string representation of the given state.
absl::string_view Ds402StateAsString(Ds402State);

// Attempts to convert a status word to a DS402 state. If the conversion is
// successful, returns true. Otherwise (i.e. if status_word does not correspond
// to a recognised state), false will be returned and state will be unmodified.
bool Ds402StateFromStatusWord(uint16_t status_word, Ds402State& state);

// Returns the status word associated to the given state.
uint16_t Ds402StatusWordFromState(Ds402State state);

// Calculates a control word to move the DS402 finite state machine towards
// target_state from current_state, and sets control_word to this value.
//
// If current_state is not the fault state then the fault bit of control_word
// will always be set to zero to ensure that the fault reset command (changing
// the fault bit from zero to one) will always be available.
//
// Returns true if there is an unambiguous transition and associated controlword
// to move the state from current_state in the direction of target_state.
// Otherwise (including the case where target_state == current_state),
// control_word will be unmodified (other than the fault bit being set to zero)
// and false will be returned.
//
// In the case that there is an automatic transition out of current_state (e.g.
// in the fault reaction or not ready states), and target_state !=
// current_state, then control_word will be set to zero.
//
// If the current state is the fault state and target_state != current_state,
// then either control_word will be set to the fault reset code and true will be
// returned (if auto_reset_from_fault is true) or it will be unmodified except
// for the fault bit being set to zero and false will be returned (otherwise).
bool Ds402ControlWordForTargetState(Ds402State target_state,
                                     Ds402State current_state,
                                     uint16_t& control_word,
                                     bool auto_reset_from_fault = true);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_COMMON_DS402_STATE_H_
