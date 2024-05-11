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

#include "ds402_state.h"

#include <cstdint>
#include <ios>
#include <type_traits>

#include "absl/log/log.h"
#include "absl/strings/string_view.h"

namespace barkour {

namespace {

template <typename Enumeration>
constexpr std::enable_if_t<std::is_enum<Enumeration>::value,
                           std::underlying_type_t<Enumeration>>
as_number(Enumeration value) {
  return static_cast<std::underlying_type_t<Enumeration>>(value);
}

enum class ControlwordBitMasks : uint16_t {
  kSwitchOn = 1,
  kEnableVoltage = 1 << 1,
  kQuickStop = 1 << 2,
  kEnableOperation = 1 << 3,
  kFaultReset = 1 << 7,
};
enum class StatuswordBitMasks : uint16_t {
  kReadyToSwitchOn = 1,
  kSwitchedOn = 1 << 1,
  kOperationEnabled = 1 << 2,
  kFault = 1 << 3,
  kVoltageEnabled = 1 << 4,
  kQuickStop = 1 << 5,
  kSwitchOnDisabled = 1 << 6,
  kWarning = 1 << 7,
};

}  // namespace

absl::string_view Ds402StateAsString(Ds402State state) {
  switch (state) {
    case Ds402State::kNotReady:
      return "NOT_READY";
    case Ds402State::kSwitchedOn:
      return "SWITCHED_ON";
    case Ds402State::kOperationEnabled:
      return "OPERATION_ENABLED";
    case Ds402State::kQuickStop:
      return "QUICK_STOP";
    case Ds402State::kSwitchOnDisabled:
      return "SWITCH_ON_DISABLED";
    case Ds402State::kReadyToSwitchOn:
      return "READY_TO_SWITCH_ON";
    case Ds402State::kFaultReaction:
      return "FAULT_REACTION";
    case Ds402State::kFault:
      return "FAULT";
    default:
      return "UNKNOWN_STATE";
  }
}

bool Ds402ControlWordForTargetState(Ds402State target_state,
                                     Ds402State current_state,
                                     uint16_t& control_word,
                                     bool auto_reset_from_fault) {
  // Reset the fault bit if we are not in fault.
  // This will ensure that the fault bit will always be zero.
  if (current_state != Ds402State::kFault) {
    control_word &= ~as_number(ControlwordBitMasks::kFaultReset);
    auto_reset_from_fault = false;
  }

  if (target_state == current_state) {
    // The target state has already been reached.
    return false;
  }
  // This bit should always remain 1 in all the commands.
  // In comments, the transition number as per the DS402 standard.
  constexpr uint16_t f = as_number(ControlwordBitMasks::kFaultReset);  // bit 7
  constexpr uint16_t e =
      as_number(ControlwordBitMasks::kEnableOperation);               // bit 3
  constexpr uint16_t q = as_number(ControlwordBitMasks::kQuickStop);  // bit 2
  constexpr uint16_t v =
      as_number(ControlwordBitMasks::kEnableVoltage);                // bit 1
  constexpr uint16_t s = as_number(ControlwordBitMasks::kSwitchOn);  // bit 0

  constexpr uint16_t shutdown = (0 | 0 | q | v | 0);
  constexpr uint16_t switch_on = (0 | 0 | q | v | s);
  constexpr uint16_t disable_voltage = (0 | 0 | 0 | 0 | 0);
  constexpr uint16_t quick_stop = (0 | 0 | 0 | v | 0);
  constexpr uint16_t disable_operation = (0 | 0 | q | v | s);
  constexpr uint16_t enable_operation = (0 | e | q | v | s);
  constexpr uint16_t fault_reset = (f | 0 | 0 | 0 | 0);
  switch (current_state) {
    case Ds402State::kFault: {
      // Transition 15. Fault reset
      if (auto_reset_from_fault) {
        VLOG(1) << "Reset from fault.";
        control_word = fault_reset;
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kSwitchOnDisabled: {
      // If we want to go towards the "enable state" (or to "quick stop" via the
      // enabled state):
      if (target_state == Ds402State::kReadyToSwitchOn ||
          target_state == Ds402State::kSwitchedOn ||
          target_state == Ds402State::kOperationEnabled ||
          target_state == Ds402State::kQuickStop) {
        // Transition 2. Shutdown command.
        control_word = shutdown;
        VLOG(1) << "kSwitchOnDisabled => Shutdown" << control_word;
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kReadyToSwitchOn: {
      if (target_state == Ds402State::kSwitchedOn) {
        // Transition 3. Switch On command.
        control_word = switch_on;
        VLOG(1) << "kReadyToSwitchOn => switch_on" << control_word << '\n';
        return true;
      } else if (target_state == Ds402State::kOperationEnabled ||
                 target_state == Ds402State::kQuickStop) {
        // Transition 3 + 4. Switch On + Enable Operation commands.
        control_word = enable_operation;
        VLOG(1) << "kReadyToSwitchOn => enable_operation" << control_word
                << '\n';
        return true;
      } else if (target_state == Ds402State::kSwitchOnDisabled) {
        // Transition 7. Quick stop & Disable voltage command.
        control_word = disable_voltage;
        VLOG(1) << "kReadyToSwitchOn => disable_voltage" << control_word
                << '\n';
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kSwitchedOn: {
      if (target_state == Ds402State::kOperationEnabled ||
          target_state == Ds402State::kQuickStop) {
        // Transition 4. Enable operation command.
        control_word = enable_operation;
        VLOG(1) << "kSwitchedOn => enable_operation" << control_word << '\n';
        return true;
      } else if (target_state == Ds402State::kReadyToSwitchOn) {
        // Transition 6. Shutdown command.
        control_word = shutdown;
        VLOG(1) << "kSwitchedOn => shutdown" << control_word << '\n';
        return true;
      } else if (target_state == Ds402State::kSwitchOnDisabled) {
        // Transition 10. Disable voltage or quick stop command.
        control_word = disable_voltage;
        VLOG(1) << "kSwitchedOn => disable_voltage" << control_word << '\n';
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kOperationEnabled: {
      if (target_state == Ds402State::kSwitchedOn) {
        // Transition 5. Disable operation command.
        control_word = disable_operation;
        VLOG(1) << "kOperationEnabled => disable_operation" << control_word
                << '\n';
        return true;
      } else if (target_state == Ds402State::kReadyToSwitchOn) {
        // Transition 8. Shutdown command.
        control_word = shutdown;
        VLOG(1) << "kOperationEnabled => shutdown" << control_word << '\n';
        return true;
      } else if (target_state == Ds402State::kSwitchOnDisabled) {
        // Transition 9. Disable voltage or quick stop command.
        control_word = disable_voltage;
        VLOG(1) << "kOperationEnabled => disable_voltage" << control_word
                << '\n';
        return true;
      } else if (target_state == Ds402State::kQuickStop) {
        // Transition 11. Quick stop.
        control_word = quick_stop;
        VLOG(1) << "kOperationEnabled => quick_stop" << control_word << '\n';
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kQuickStop: {
      if (target_state == Ds402State::kOperationEnabled) {
        // Transition 16. Enable operation command.
        control_word = enable_operation;
        VLOG(1) << "kQuickStop => enable_operation" << control_word << '\n';
        return true;
      } else if (target_state == Ds402State::kSwitchOnDisabled) {
        // Transition 12. Disable voltage command.
        control_word = disable_voltage;
        VLOG(1) << "kQuickStop => disable_voltage" << control_word << '\n';
        return true;
      } else {
        return false;
      }
    }
    case Ds402State::kFaultReaction:
    case Ds402State::kNotReady:
      // These states will be moved out of automatically, so the control word is
      // ignored.
      control_word = 0;
      return true;
  }
  return true;
}

bool Ds402StateFromStatusWord(uint16_t status_word, Ds402State& state) {
  // Shortcuts for the bits (0 to 6)
  const uint16_t r = as_number(StatuswordBitMasks::kReadyToSwitchOn);   // bit 0
  const uint16_t s = as_number(StatuswordBitMasks::kSwitchedOn);        // bit 1
  const uint16_t e = as_number(StatuswordBitMasks::kOperationEnabled);  // bit 2
  const uint16_t f = as_number(StatuswordBitMasks::kFault);             // bit 3
  // bit 4 ignored.
  const uint16_t q = as_number(StatuswordBitMasks::kQuickStop);         // bit 5
  const uint16_t d = as_number(StatuswordBitMasks::kSwitchOnDisabled);  // bit 6
  // The above bits (and their combination) is the only thing we check.
  // We then mask on those bits
  uint16_t word = status_word & (d | q | f | e | s | r);
  Ds402State next_state = Ds402State::kNotReady;
  // We now switch on all the possible states (and their multiple
  // representations).
  switch (word) {
    //    d   q   f   e   s   r
    case (0 | 0 | 0 | 0 | 0 | 0):
    case (0 | q | 0 | 0 | 0 | 0):
      next_state = Ds402State::kNotReady;
      VLOG(1) << std::hex << status_word << "Ds402State::kNotReady"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (d | 0 | 0 | 0 | 0 | 0):
    case (d | q | 0 | 0 | 0 | 0):
      next_state = Ds402State::kSwitchOnDisabled;
      VLOG(1) << std::hex << status_word << "Ds402State::kSwitchOnDisabled"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | q | 0 | 0 | 0 | r):
      next_state = Ds402State::kReadyToSwitchOn;
      VLOG(1) << std::hex << status_word << "Ds402State::kReadyToSwitchOn"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | q | 0 | 0 | s | r):
      next_state = Ds402State::kSwitchedOn;
      VLOG(1) << std::hex << status_word << "Ds402State::kSwitchedOn"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | q | 0 | e | s | r):
      next_state = Ds402State::kOperationEnabled;
      VLOG(1) << std::hex << status_word << "Ds402State::kOperationEnabled"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | 0 | 0 | e | s | r):
      next_state = Ds402State::kQuickStop;
      VLOG(1) << std::hex << status_word << "Ds402State::kQuickStop"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | 0 | f | e | s | r):
    case (0 | q | f | e | s | r):
      next_state = Ds402State::kFaultReaction;
      VLOG(1) << std::hex << status_word << "Ds402State::kFaultReaction"
              << std::dec;
      break;
    //    d   q   f   e   s   r
    case (0 | 0 | f | 0 | 0 | 0):
    case (0 | q | f | 0 | 0 | 0):
    case (d | q | f | 0 | 0 | 0):  // For beckhoff motor
    case (d | 0 | f | 0 | 0 | 0):  // For beckhoff motor
      next_state = Ds402State::kFault;
      VLOG(1) << std::hex << status_word << "Ds402State::kFault" << std::dec;
      break;
    default:
      return false;
  }
  state = next_state;
  return true;
}

uint16_t Ds402StatusWordFromState(Ds402State state) {
  switch (state) {
    case Ds402State::kSwitchedOn:
      return 0b0010'0011;
    case Ds402State::kOperationEnabled:
      return 0b0010'0111;
    case Ds402State::kQuickStop:
      return 0b0000'0111;
    case Ds402State::kSwitchOnDisabled:
      return 0b0100'0000;
    case Ds402State::kReadyToSwitchOn:
      return 0b0010'0001;
    case Ds402State::kFaultReaction:
      return 0b0000'1111;
    case Ds402State::kFault:
      return 0b0000'1000;
    case Ds402State::kNotReady:
    default:
      return 0b0000'0000;
  }
}

}  // namespace barkour
