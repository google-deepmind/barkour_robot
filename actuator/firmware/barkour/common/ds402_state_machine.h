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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DS402_STATE_MACHINE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DS402_STATE_MACHINE_H_

#include <cstdint>
#include <string_view>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_string/string_builder.h"

namespace barkour {

class Ds402State {
 public:
  // These state code values can be cast in order to obtain the state
  // bit values of the CiA402 control word.
  enum Code : uint16_t {
    kNotReadyToSwitchOn = 0x0000,
    kSwitchOnDisabled = 0x0040,
    kReadyToSwitchOn = 0x0021,
    kSwitchedOn = 0x0023,
    kOperationEnabled = 0x0027,
    kQuickStopActive = 0x0007,
    kFaultReactionActive = 0x000F,
    kFault = 0x0008,
  };

  // Functions that create a Ds402State with the specified code.
  // clang-format off
  static constexpr Ds402State NotReadyToSwitchOn() {
    return Ds402State(Code::kNotReadyToSwitchOn, "NOT_READY_TO_SWITCH_ON");
  }

  static constexpr Ds402State SwitchOnDisabled() {
    return Ds402State(Code::kSwitchOnDisabled, "SWITCH_ON_DISABLED");
  }

  static constexpr Ds402State ReadyToSwitchOn() {
    return Ds402State(Code::kReadyToSwitchOn, "READY_TO_SWITCH_ON");
  }

  static constexpr Ds402State SwitchedOn() {
    return Ds402State(Code::kSwitchedOn, "SWITCHED_ON");
  }

  static constexpr Ds402State OperationEnabled() {
    return Ds402State(Code::kOperationEnabled, "OPERATION_ENABLED");
  }

  static constexpr Ds402State QuickStopActive() {
    return Ds402State(Code::kQuickStopActive, "QUICK_STOP_ACTIVE");
  }

  static constexpr Ds402State FaultReactionActive() {
    return Ds402State(Code::kFaultReactionActive, "FAULT_REACTION_ACTIVE");
  }

  static constexpr Ds402State Fault() {
    return Ds402State(Code::kFault, "FAULT");
  }

  constexpr explicit operator Code() const { return code_; }

  constexpr explicit operator uint16_t() const {
    return static_cast<uint16_t>(code_);
  }

  bool operator==(const Ds402State& other) const {
    return (code_ == other.code_);
  }

  bool operator!=(const Ds402State& other) const {
    return (code_ != other.code_);
  }

  bool operator==(const Ds402State::Code& other_code) const {
    return (code_ == other_code);
  }

  bool operator!=(const Ds402State::Code& other_code) const {
    return (code_ != other_code);
  }

  // Returns a null-terminated string representation of the Ds402State.
  const char* str() const { return name_.data(); }

 private:
  // Note: It is the caller's responsibility to ensure that the state does not
  //   outlive the underlying character array that the name value points to.
  constexpr Ds402State(Code code, const std::string_view name)
      : code_(code), name_(name) {}

  Code code_;
  std::string_view name_;
};

pw::StringBuilder& operator<<(pw::StringBuilder& sb, const Ds402State& value);

class Ds402Command {
 public:
  enum OpCode : uint16_t {
    // CiA402 commands.
    kShutdown = 0,
    kSwitchOn,
    kDisableVoltage,
    kQuickStop,
    kEnableOperation,
    kDisableOperation,
    kFaultReset,
    // Internal Commands.
    // These commands cannot be generated via a control word.
    kFault,
    kNone,
    kUnknown,
  };

  static constexpr Ds402Command Shutdown() {
    return Ds402Command(OpCode::kShutdown, "SHUTDOWN");
  }

  static constexpr Ds402Command SwitchOn() {
    return Ds402Command(OpCode::kSwitchOn, "SWITCH_ON");
  }

  static constexpr Ds402Command DisableVoltage() {
    return Ds402Command(OpCode::kDisableVoltage, "DISABLE_VOLTAGE");
  }

  static constexpr Ds402Command QuickStop() {
    return Ds402Command(OpCode::kQuickStop, "QUICK_STOP");
  }

  static constexpr Ds402Command EnableOperation() {
    return Ds402Command(OpCode::kEnableOperation, "ENABLE_OPERATION");
  }

  static constexpr Ds402Command DisableOperation() {
    return Ds402Command(OpCode::kDisableOperation, "DISABLE_OPERATION");
  }

  static constexpr Ds402Command FaultReset() {
    return Ds402Command(OpCode::kFaultReset, "FAULT_RESET");
  }

  static constexpr Ds402Command Fault() {
    return Ds402Command(OpCode::kFault, "FAULT");
  }

  static constexpr Ds402Command None() {
    return Ds402Command(OpCode::kNone, "NONE");
  }

  static constexpr Ds402Command Unknown() {
    return Ds402Command(OpCode::kUnknown, "UNKNOWN");
  }

  constexpr explicit operator OpCode() const { return code_; }

  bool operator==(const Ds402Command& other) const {
    return (code_ == other.code_);
  }

  bool operator!=(const Ds402Command& other) const {
    return (code_ != other.code_);
  }

  Ds402Command() : code_(kNone), name_("NONE") {}

  // Returns a null-terminated string representation of the Ds402Command.
  const char* str() const { return name_.data(); }

 private:
  // Note: It is the caller's responsibility to ensure that the state does not
  //   outlive the underlying character array that the name value points to.
  constexpr Ds402Command(OpCode code, std::string_view name)
      : code_(code), name_(name) {}

  OpCode code_;
  std::string_view name_;
};

// Drives must implement Ds402DriveInterface. The methods declared in this
// interface will be called by Ds402State machine.
class Ds402DriveInterface {
 public:
  enum class TransitionStatus : uint8_t {
    // The requested transition is still ongoing, and the DS402 state machine
    // should not change state.
    kOngoing = 0,
    // The requested transition is finished, and the DS402 state machine is free
    // to change to the next state.
    kFinished,
  };

  virtual ~Ds402DriveInterface() = default;

  // --- Transition Methods ---

  // The following virtual functions correspond to transitions or
  // sub-transitions between DS402 states. They should return:
  // - TransitionStatus::kFinished for a successful transition,
  // - TransitionStatus::kOngoing for an in-progress transition which hasn't
  //   completed yet,
  // - An error status for a transition failure, which will result in
  //   FaultHandler being called on the next update cycle.
  //
  // Each transition function should be idempotent, i.e., should be able to be
  // called repeatedly after the transition is complete, with no further
  // effects.

  // Initializes the drive.
  virtual pw::Result<TransitionStatus> Initialize() = 0;

  // Enables power to the drive gates.
  virtual pw::Result<TransitionStatus> EnablePower() = 0;

  // Disables power to the gates.
  virtual pw::Result<TransitionStatus> DisablePower() = 0;

  // Enables the drive. Motion commands will only be possible while the drive
  // is enabled.
  //
  // This is also used to re-enable the drive from the quick stop active state.
  virtual pw::Result<TransitionStatus> EnableDrive() = 0;

  // Disables the drive. Motion commands will not be possible while the drive
  // is disabled.
  virtual pw::Result<TransitionStatus> DisableDrive() = 0;

  // Activates the quick stop functionality.
  virtual pw::Result<TransitionStatus> ActivateQuickStop() = 0;

  // Fault handler, should completely disable the drive regardless of which
  // state it is currently in.
  //
  // This is only permitted to return a TransitionState and not a Status,
  // as handling a fault may not finish immediately but cannot fail.
  virtual TransitionStatus HandleFault() = 0;

  // --- State Action Methods ---

  // State action function. This will be called after each state machine update,
  // after any transitions have been handled, with the value of the current
  // state of the machine.
  //
  // `manufacturer_status_register` should be updated with any appropriate
  // errors that are observed.
  //
  // `gate_driver_status_registers` should be updated with the contents of the
  // gate driver fault registers in the case of any unexpected gate driver
  // faults.
  //
  // If this returns a non-OK status a DS402 fault will be triggered.
  virtual pw::Status DoStateAction(
      Ds402State current_state,
      ManufacturerStatusRegisterErrorCodes& manufacturer_status_register,
      GateDriverStatusRegisters& gate_driver_status_registers) = 0;
};

// The Ds402 state machine was initially described in CAN in Automation
// document 402, and later standardized as part of IEC 661800-7-201.
// Additionally, the EtherCAT Technology Group published ETC.6010
// "Implementation Directive for CiA402 Drive Profile", which further details
// clarifies how this state machine should be implemented for use in EtherCAT.
// The following class implements the state machine described in those
// documents.
class Ds402StateMachine {
 public:
  explicit Ds402StateMachine(Ds402DriveInterface& drive);

  // Returns the current state.
  const Ds402State& GetState();

  // The control word is the primary input to the state machine.
  // Calling this method will not cause the state machine to update. In order
  // for the state machine to be updated, the Update() function must be called.
  void SetControlWord(uint16_t control_word);

  // The status word provides details about the operating state of the drive.
  // For more details, refer to ETG.6010.
  uint16_t GetStatusWord();

  // Updates the state machine. This function is responsible for calling the
  // drive interface methods, and should called regularly.
  //
  // The passed `manufacturer_status_register` and
  // `gate_driver_status_registers` will be updated with any errors which occur
  // inside the Ds402Drive.
  //
  // Upon a recovery from the fault state, all error codes corresponding to DS
  // 402 errors (i.e. those which make up
  // `kManufacturerStatusRegisterCia402FaultCodes`) in the manufacturer status
  // register will be cleared.
  //
  // `gate_driver_status_registers` will only be updated when there is an
  // unexpected fault condition in the gate driver, and will not be cleared upon
  // any DS402 recovery.
  Ds402State Update(
      ManufacturerStatusRegisterErrorCodes& manufacturer_status_register,
      GateDriverStatusRegisters& gate_driver_status_registers);

  // Transition immediately to FaultReactionActive() state, disregarding any
  // existing command.
  void Fault();

 private:
  Ds402State state_;
  Ds402State target_state_;
  Ds402Command new_command_;
  uint16_t previous_control_word_;
  uint16_t control_word_;

  Ds402DriveInterface& drive_;

  Ds402DriveInterface::TransitionStatus CheckDriveCommandMaybeFault(
      pw::Result<Ds402DriveInterface::TransitionStatus> drive_command_result);
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_DS402_STATE_MACHINE_H_
