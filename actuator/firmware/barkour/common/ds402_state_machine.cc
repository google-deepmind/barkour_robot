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

#include "actuator/firmware/barkour/common/ds402_state_machine.h"

#include <bitset>
#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_string/string_builder.h"

namespace barkour {

namespace {

constexpr uint16_t kStatusWordVoltageEnabledMask = 0x0010;

bool IsFinished(Ds402DriveInterface::TransitionStatus transition_status) {
  return transition_status == Ds402DriveInterface::TransitionStatus::kFinished;
}

}  // namespace

pw::StringBuilder& operator<<(pw::StringBuilder& sb, const Ds402State& value) {
  return sb << "Ds402State(" << value.str() << ')';
}

Ds402StateMachine::Ds402StateMachine(Ds402DriveInterface& drive)
    : state_(Ds402State::NotReadyToSwitchOn()),
      target_state_(Ds402State::SwitchOnDisabled()),
      previous_control_word_(0),
      control_word_(0),
      drive_(drive) {}

const Ds402State& Ds402StateMachine::GetState() { return state_; }

void Ds402StateMachine::SetControlWord(uint16_t control_word) {
  std::bitset<16> control_word_bits(control_word);
  bool fault_reset = control_word_bits[7];
  bool enable_operation = control_word_bits[3];
  bool quick_stop = control_word_bits[2];
  bool enable_voltage = control_word_bits[1];
  bool switch_on = control_word_bits[0];

  std::bitset<16> previous_control_word_bits(previous_control_word_);
  bool previous_fault_reset = previous_control_word_bits[7];

  // Determine which command is associated with the control_word.
  // There are 8 possible commands.
  // Details in ETG.6010.5.2 (Implementation Directive for CiA402)
  Ds402Command command = Ds402Command::None();

  if (!previous_fault_reset && fault_reset) {
    // Fault reset command should only be sent once, upon the rising edge of the
    // fault reset bit of the control word.
    command = Ds402Command::FaultReset();
  } else if (!enable_voltage) {
    command = Ds402Command::DisableVoltage();
  } else if (!quick_stop) {
    // Note: the QuickStop signal is in the control word is active-low.
    command = Ds402Command::QuickStop();
  } else if (!switch_on) {
    command = Ds402Command::Shutdown();
  } else if (switch_on && enable_voltage && enable_operation) {
    command = Ds402Command::EnableOperation();
  } else {
    command = Ds402Command::SwitchOn();
  }

  // SwitchOn command is the same as DisableOperation command, so we will
  // interpret the control word depending on our current state.
  if ((state_ == Ds402State::OperationEnabled()) &&
      (command == Ds402Command::SwitchOn())) {
    command = Ds402Command::DisableOperation();
  }

  if (control_word_ != previous_control_word_) {
    previous_control_word_ = control_word_;
    PW_LOG_INFO(
        "Drive received new command: %s (%02X)", command.str(), control_word_);
  }

  new_command_ = command;
}

uint16_t Ds402StateMachine::GetStatusWord() {
  uint16_t status_word = static_cast<uint16_t>(state_);

  // Set Voltage enabled bit appropriately.
  switch (static_cast<Ds402State::Code>(state_)) {
    case static_cast<Ds402State::Code>(Ds402State::SwitchedOn()):
    case static_cast<Ds402State::Code>(Ds402State::OperationEnabled()):
    case static_cast<Ds402State::Code>(Ds402State::QuickStopActive()):
    case static_cast<Ds402State::Code>(Ds402State::FaultReactionActive()): {
      status_word |= kStatusWordVoltageEnabledMask;
      break;
    }
    default: {
      break;
    }
  }

  return status_word;
}

Ds402State Ds402StateMachine::Update(
    ManufacturerStatusRegisterErrorCodes& manufacturer_status_register,
    GateDriverStatusRegisters& gate_driver_status_registers) {
  // If this is set to a non-nullopt value in the switch statement, a state
  // transition will occur.
  std::optional<Ds402State> next_state;
  bool command_invalid = false;

  // Perform transition action.
  switch (static_cast<Ds402State::Code>(state_)) {
    case static_cast<Ds402State::Code>(Ds402State::NotReadyToSwitchOn()): {
      // Transition 1 from CiA402.
      // Automatic transition to SwitchOnDisabled().
      if (IsFinished(CheckDriveCommandMaybeFault(drive_.Initialize()))) {
        next_state = Ds402State::SwitchOnDisabled();
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::SwitchOnDisabled()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(Ds402Command::Shutdown()): {
          // Transition 2 from CiA402.
          next_state = Ds402State::ReadyToSwitchOn();
          break;
        }
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::DisableVoltage()): {
          break;
        }
        default:
          command_invalid = true;
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::ReadyToSwitchOn()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(Ds402Command::SwitchOn()): {
          // Transition 3 from CiA402.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.EnablePower()))) {
            next_state = Ds402State::SwitchedOn();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(Ds402Command::QuickStop()):
          // Transition 7 from CiA402.
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::DisableVoltage()): {
          // Also transition 7 from CiA402.

          // Even though the drive does not usually have power enabled in this
          // state, if a transition from this state -> SwitchedOn was requested
          // but not completed then the drive power may be partially enabled.
          // Disabling it again explicitly defeats this risk.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
            next_state = Ds402State::SwitchOnDisabled();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::EnableOperation()): {
          // Note: Though the spec does not show this transition in the diagram,
          //  ETG.6010.5.1 text indicates that bits 0, 1 and 3 can be
          //  simultaneously set in order to pass immediately from
          //  ReadyToSwitchOn state to OperationEnabled state via a single
          //  command.
          //
          // Double-stacked transition 3, then 4.
          //
          // If only the first transition succeeds, we'll just transition to
          // Switched On, where the same command still requests a transition to
          // Operation Enabled.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.EnablePower()))) {
            next_state = Ds402State::SwitchedOn();
            if (IsFinished(CheckDriveCommandMaybeFault(drive_.EnableDrive()))) {
              next_state = Ds402State::OperationEnabled();
            }
          }
          break;
        }
        default: {
          command_invalid = true;
        }
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::SwitchedOn()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::EnableOperation()): {
          // Transition 4 from CiA402.
          // May need to enable power first if a previous transition which
          // requests disabling power has been requested but not completed.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.EnablePower())) &&
              IsFinished(CheckDriveCommandMaybeFault(drive_.EnableDrive()))) {
            next_state = Ds402State::OperationEnabled();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(Ds402Command::DisableVoltage()):
        case static_cast<Ds402Command::OpCode>(Ds402Command::QuickStop()): {
          // Transition 10 from CiA402.
          // May need to disable drive first if a previous transition which
          // requests enabling the drive has been requested but not completed.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive())) &&
              IsFinished(CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
            next_state = Ds402State::SwitchOnDisabled();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(Ds402Command::Shutdown()): {
          // Transition 6 from CiA402.
          // May need to disable drive first if a previous transition which
          // requests enabling the drive has been requested but not completed.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive())) &&
              IsFinished(CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
            next_state = Ds402State::ReadyToSwitchOn();
          }
          break;
        }
        default: {
          command_invalid = true;
        }
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::OperationEnabled()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::DisableOperation()): {
          // Transition 5 from CiA402.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive()))) {
            next_state = Ds402State::SwitchedOn();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(Ds402Command::Shutdown()): {
          // Transition 8 from CiA402.
          //
          // Double-stacked transition 5, then 6.
          //
          // If only the first transition succeeds, we'll just transition to
          // Switched On, where the same command still requests a transition to
          // Ready to Switch On .
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive()))) {
            next_state = Ds402State::SwitchedOn();
            if (IsFinished(
                    CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
              next_state = Ds402State::ReadyToSwitchOn();
            }
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::DisableVoltage()): {
          // Transition 9 from CiA402.
          //
          // Triple-stacked transition 5, then 6, then 7 (which is a no-op).
          //
          // If only the first transition succeeds, we'll just transition to
          // Switched On, where the same command still requests a transition to
          // Ready to Switch On .
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive()))) {
            next_state = Ds402State::SwitchedOn();
            if (IsFinished(
                    CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
              next_state = Ds402State::SwitchOnDisabled();
            }
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(Ds402Command::QuickStop()): {
          // Transition 11 from CiA402.
          if (IsFinished(
                  CheckDriveCommandMaybeFault(drive_.ActivateQuickStop()))) {
            next_state = Ds402State::QuickStopActive();
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::EnableOperation()): {
          next_state = Ds402State::OperationEnabled();
          break;
        }
        default: {
          command_invalid = true;
        }
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::QuickStopActive()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::DisableVoltage()): {
          // Transition 12 from CiA402.
          //
          // Double-stacked disable drive + disable power transition, with no
          // intermediate state. As the drive methods are idempotent, this
          // should work fine.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.DisableDrive()))) {
            if (IsFinished(
                    CheckDriveCommandMaybeFault(drive_.DisablePower()))) {
              next_state = Ds402State::SwitchOnDisabled();
            }
          }
          break;
        }
        case static_cast<Ds402Command::OpCode>(
            Ds402Command::EnableOperation()): {
          // Transition 16 from CiA402.
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.EnableDrive()))) {
            next_state = Ds402State::OperationEnabled();
          }
          break;
        }
        default: {
          command_invalid = true;
        }
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::FaultReactionActive()): {
      // Transition 14 from CiA402.
      // Automatic transition to Fault() state.
      if (IsFinished(drive_.HandleFault())) {
        next_state = Ds402State::Fault();
      }
      break;
    }

    case static_cast<Ds402State::Code>(Ds402State::Fault()): {
      switch (static_cast<Ds402Command::OpCode>(new_command_)) {
        case static_cast<Ds402Command::OpCode>(Ds402Command::FaultReset()): {
          if (IsFinished(CheckDriveCommandMaybeFault(drive_.Initialize()))) {
            // Reset appropriate error bits in status register.
            manufacturer_status_register &=
                ~kManufacturerStatusRegisterCia402FaultMask;

            next_state = Ds402State::SwitchOnDisabled();
          }
          break;
        }
        default: {
          command_invalid = true;
        }
      }
      break;
    }
  }

  if (command_invalid && new_command_ != Ds402Command::None()) {
    PW_LOG_DEBUG("Command %s is invalid while in state %s, ignoring.",
                 new_command_.str(),
                 state_.str());
  }

  new_command_ = Ds402Command::None();

  if (next_state.has_value() && next_state.value() != state_) {
    PW_LOG_INFO("DS402 State Machine switching state from %s to %s",
                state_.str(),
                next_state.value().str());
    state_ = next_state.value();
  }

  // Now do the state action. If there is an error, cause a Fault.
  pw::Status status = drive_.DoStateAction(
      state_, manufacturer_status_register, gate_driver_status_registers);
  if (!status.ok()) {
    PW_LOG_ERROR("Error in DoStateAction: %s.", status.str());
    Fault();
  }

  return state_;
}

void Ds402StateMachine::Fault() {
  PW_LOG_ERROR(
      "A Ds402 drive FAULT has occurred, immediately switching to %s state.",
      Ds402State::FaultReactionActive().str());
  state_ = Ds402State::FaultReactionActive();
  new_command_ = Ds402Command::None();
}

// Checks the result of the a drive command.
//
// If a failure status other than Unavailable is returned, a fault will be
// triggered.
//
// Returns drive_command_status.
Ds402DriveInterface::TransitionStatus
Ds402StateMachine::CheckDriveCommandMaybeFault(
    pw::Result<Ds402DriveInterface::TransitionStatus> drive_command_result) {
  if (!drive_command_result.ok()) {
    Fault();
    // denote the transition as ongoing, in order not to let the next state be
    // overwritten by the switching mechanism.
    return Ds402DriveInterface::TransitionStatus::kOngoing;
  }
  return drive_command_result.value();
}

}  // namespace barkour
