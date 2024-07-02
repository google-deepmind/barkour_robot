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

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr uint16_t kShutdownControlWord = 0b00000110;
constexpr uint16_t kSwitchOnControlWord = 0b00000111;
constexpr uint16_t kDisableVoltageControlWord = 0b00000000;
constexpr uint16_t kQuickStopControlWord = 0b00000010;
constexpr uint16_t kEnableOperationControlWord = 0b00001111;
constexpr uint16_t kDisableOperationControlWord = 0b00000111;
constexpr uint16_t kFaultResetControlWord = 0b10000000;

class TestDrive : public Ds402DriveInterface {
 public:
  enum class Command {
    kNone,
    kInitialize,
    kEnablePower,
    kDisablePower,
    kEnableDrive,
    kDisableDrive,
    kActivateQuickStop,
    kHandleFault,
    kReset,
  };

  pw::Result<TransitionStatus> Initialize() override {
    initialized_ = true;
    last_command_ = Command::kInitialize;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  pw::Result<TransitionStatus> EnablePower() override {
    power_enabled_ = true;
    last_command_ = Command::kEnablePower;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  pw::Result<TransitionStatus> DisablePower() override {
    power_enabled_ = false;
    last_command_ = Command::kDisablePower;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  pw::Result<TransitionStatus> EnableDrive() override {
    drive_enabled_ = true;
    last_command_ = Command::kEnableDrive;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  pw::Result<TransitionStatus> DisableDrive() override {
    drive_enabled_ = false;
    last_command_ = Command::kDisableDrive;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  pw::Result<TransitionStatus> ActivateQuickStop() override {
    drive_enabled_ = false;
    last_command_ = Command::kActivateQuickStop;
    return return_status_.ok()
               ? pw::Result<TransitionStatus>(return_transition_status_)
               : pw::Result<TransitionStatus>(return_status_);
  };

  TransitionStatus HandleFault() override {
    last_command_ = Command::kHandleFault;
    return return_transition_status_;
  };

  void Reset() {
    initialized_ = false;
    power_enabled_ = false;
    drive_enabled_ = false;
    last_command_ = Command::kReset;
  };

  pw::Status DoStateAction(
      Ds402State current_state,
      ManufacturerStatusRegisterErrorCodes& status_register,
      GateDriverStatusRegisters& gate_driver_status_registers) override {
    state_passed_to_state_action_ = current_state;

    status_register |= status_register_bits_to_set_;

    gate_driver_status_registers = gate_driver_status_registers_;
    return pw::OkStatus();
  }

  Command last_command_ = Command::kNone;
  Ds402State state_passed_to_state_action_ = Ds402State::NotReadyToSwitchOn();
  bool initialized_ = false;
  bool power_enabled_ = false;
  bool drive_enabled_ = false;
  pw::Status return_status_ = pw::OkStatus();
  TransitionStatus return_transition_status_ = TransitionStatus::kFinished;
  ManufacturerStatusRegisterErrorCodes status_register_bits_to_set_ =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers_;
};

TEST(Ds402StateMachineTest, InitialStateIsNotReadyToSwitchOn) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ASSERT_EQ(m.GetState(), Ds402State::NotReadyToSwitchOn());
}

// Testing the Automatic transitions to SwitchOnDisabled() state.
TEST(Ds402StateMachineTest, Transition1) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // After the first call to Update(), we should be in NotReadyToSwitchOn()
  // This is transition 1
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());

  // The TestDrive Initialization function should have been called.
  ASSERT_TRUE(drive.initialized_);
}

// Ensure that when Drive Initialize() fails during transition 1 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition1DriveInitializeFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kInitialize);
}

// Tests transitions between SwitchOnDisabled() and ReadyToSwitchOn()
TEST(Ds402StateMachineTest, Transitions2And7) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());

  // Transition 2 happens when we send a Shutdown() command from the
  // SwitchOnDisabled() state.
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  // Transition 7 occurs when QuickStop or DisableVoltage commands are
  // received.
  // So the QuickStop command will send us back to SwitchOnDisabled().
  m.SetControlWord(kQuickStopControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());

  // Now go back to the ReadyToSwitchOn() state.
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  // Finally, test the transition using the DisableVoltageControl command.
  m.SetControlWord(kDisableVoltageControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
}

// Tests the special transition not not specifically documented in the CiA-402
// FSA diagram, wherein the machine can transition directly from
// ReadyToSwitchOn() to OperationEnabled() upon receipt of an EnableOperation()
// command.
TEST(Ds402StateMachineTest, ReadyToSwitchOnToOperationEnabled) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, we need to get to the ReadyToSwitchOn state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  ASSERT_FALSE(drive.drive_enabled_);
  ASSERT_FALSE(drive.power_enabled_);

  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnableDrive);

  ASSERT_TRUE(drive.drive_enabled_);
  ASSERT_TRUE(drive.power_enabled_);
}

// Ensure that when Drive EnableDrive() fails during the special transition from
// ReadyToSwitchOn to OperationEnabled, that the state machine ends up in the
// FaultReactionActive() state.
TEST(Ds402StateMachineTest,
     ReadyToSwitchOnToOperationEnabledFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, we need to get to the ReadyToSwitchOn state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  m.SetControlWord(kEnableOperationControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnablePower);
}

// Tests transitions between SwitchedOn() and ReadyToSwitchOn()
TEST(Ds402StateMachineTest, Transitions3And6) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, we need to get to the ReadyToSwitchOn state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  // Transition 3 occurs when a SwitchOn command is received.
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  // This transition should have caused the drive's EnablePower()
  // method to be called.
  ASSERT_TRUE(drive.power_enabled_);
  drive.power_enabled_ = false;

  // Transition 6 occurs when Shutdown command is received from the host.
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
}

// Ensure that when drive EnablePower() fails during transition 1 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition3DriveEnablePowerFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, we need to get to the ReadyToSwitchOn state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  // A SwitchOn control word while in ReadyToSwitchOn will result in a call to
  // EnablePower().
  m.SetControlWord(kSwitchOnControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnablePower);
}

// Ensure that when drive DisablePower() fails during transition 6 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition6DriveDisablePowerFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into SwitchedOn.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  // A Shutdown control word while in SwitchedOn will result in a call to
  // DisablePower().
  m.SetControlWord(kShutdownControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());

  // The state machine will first disable the drive, in case it was partially
  // enabled by a request to enable operation.
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kDisableDrive);
}

// Tests transitions between SwitchedOn() and OperationEnabled().
TEST(Ds402StateMachineTest, Transitions4And5) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into SwitchedOn() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  // Transition 4 occurs when the EnableOperation() command is received.
  ASSERT_FALSE(drive.drive_enabled_);
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  // The EnableDrive() drive should have been called.
  ASSERT_TRUE(drive.drive_enabled_);

  // Transition 5 occurs when the DisableOperation() command is received.
  m.SetControlWord(kDisableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  ASSERT_FALSE(drive.drive_enabled_);
}

// Ensure that when drive EnableDrive() fails during transition 4 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition4DriveEnableDriveFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into SwitchedOn() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  m.SetControlWord(kEnableOperationControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());

  // The state machine will first disable power, in case it was partially
  // disabled by a request to disable voltage.
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnablePower);
}

// Ensure that when drive DisableDrive() fails during transition 5 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition5DriveDisableDriveFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  m.SetControlWord(kDisableOperationControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kDisableDrive);
}

// Tests transition between OperationEnabled() and ReadyToSwitchOn().
TEST(Ds402StateMachineTest, Transition8) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  // Transition 8: Shutdown() command while in OperationEnabled().
  // drive.DisableDrive() and drive.DisablePower() should be called.
  ASSERT_TRUE(drive.drive_enabled_);
  ASSERT_TRUE(drive.power_enabled_);

  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());

  ASSERT_FALSE(drive.drive_enabled_);
  ASSERT_FALSE(drive.power_enabled_);
}

// Ensure that when drive DisablePower() fails during transition 8 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition8DriveDisablePowerFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  m.SetControlWord(kShutdownControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kDisableDrive);
}

// Tests transition between OperationEnabled() and SwitchOnDisabled().
TEST(Ds402StateMachineTest, Transition9) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  // Transition 9: DisableVoltage() command while in OperationEnabled().
  // drive.DisableDrive() and drive.DisablePower() should be called.
  ASSERT_TRUE(drive.drive_enabled_);
  ASSERT_TRUE(drive.power_enabled_);

  m.SetControlWord(kDisableVoltageControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());

  ASSERT_FALSE(drive.drive_enabled_);
  ASSERT_FALSE(drive.power_enabled_);
}

// Ensure that when drive DisablePower() fails during transition 9 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition9DriveDisablePowerFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine intoOperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  m.SetControlWord(kDisableVoltageControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kDisableDrive);
}

// Tests transition between SwitchedOn() and SwitchOnDisabled().
TEST(Ds402StateMachineTest, Transition10) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into SwitchedOn() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  // DisableVoltage() should send the machine to SwitchOnDisabled().
  ASSERT_TRUE(drive.power_enabled_);
  m.SetControlWord(kDisableVoltageControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  ASSERT_FALSE(drive.power_enabled_);

  // QuickStop() should also send us to SwitchOnDisabled().
  // But first we must get back to the SwitchedOn() state.
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  ASSERT_TRUE(drive.power_enabled_);
  m.SetControlWord(kQuickStopControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  ASSERT_FALSE(drive.power_enabled_);
}

// Ensure that when drive DriveEnable() fails during transition 10 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition10DriveEnableFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into SwitchedOn() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());

  m.SetControlWord(kDisableVoltageControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());

  // The state machine will first disable the drive, in case it was partially
  // enabled by a request to enable operation.
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kDisableDrive);
}

// Tests QuickStop functionality, which consists of transitions between
// OperationEnabled(), QuickStopActive() and SwitchOnDisabled().
TEST(Ds402StateMachineTest, Transitions11And16) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  // Transition 11: QuickStop() command puts the machine in QuickStopActive().
  ASSERT_TRUE(drive.drive_enabled_);
  m.SetControlWord(kQuickStopControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::QuickStopActive());
  ASSERT_FALSE(drive.drive_enabled_);
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kActivateQuickStop);

  // Transition 16: EnableOperation() command puts the machine back into
  // the OperationEnabled() state.
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());
  ASSERT_TRUE(drive.drive_enabled_);
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnableDrive);
}

// Ensure that when drive DisableDrive() fails during transition 11 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition11DriveDisableDriveFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  m.SetControlWord(kQuickStopControlWord);
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kActivateQuickStop);
}

// Ensure that when drive EnableDrive() fails during transition 16 that the
// state machine ends up in FaultReactionActive() state.
TEST(Ds402StateMachineTest, Transition16DriveEnableDriveFailsToFaultReaction) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // First, put the state machine into OperationEnabled() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  m.SetControlWord(kShutdownControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::ReadyToSwitchOn());
  m.SetControlWord(kSwitchOnControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchedOn());
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::OperationEnabled());

  m.SetControlWord(kQuickStopControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::QuickStopActive());

  drive.return_status_ = pw::Status::Internal();
  m.SetControlWord(kEnableOperationControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kEnableDrive);
}

// These are the transitions associated with Fault and Fault Recovery.
TEST(Ds402StateMachineTest, Transitions13And14And15) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // Easiest way to fault is to encoutner a problem with drive initialization.
  // An update will cause drive Initialized to be called, which will return
  // an Internal() error. This error should cause a fault in the state machine.
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  drive.return_status_ = pw::OkStatus();
  drive.initialized_ = false;

  // After an update we should be in the Faulted() state, and fault handling
  // should have been called.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::Fault());
  ASSERT_EQ(drive.last_command_, TestDrive::Command::kHandleFault);

  // Update as we may, we should be stuck in this state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::Fault());
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::Fault());
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::Fault());

  // Send a FaultReset control word, this should drop us back into
  // SwitchOnDisabled(). drive Initialize() should be called again.
  m.SetControlWord(kFaultResetControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());
  ASSERT_TRUE(drive.initialized_);
}

TEST(Ds402StateMachineTest, FaultRecoveryResetsAppropriateStatusRegisterBits) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // Easiest way to fault is to encoutner a problem with drive initialization.
  // An update will cause drive Initialized to be called, which will return
  // an Internal() error. This error should cause a fault in the state machine.
  drive.return_status_ = pw::Status::Internal();
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::FaultReactionActive());
  drive.return_status_ = pw::OkStatus();
  drive.initialized_ = false;

  // After an update we should be in the Faulted() state.
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::Fault());

  // Set all bits
  status_register = ManufacturerStatusRegisterErrorCodes(0xffffffffu);

  // Send a FaultReset control word, this should drop us back into
  // SwitchOnDisabled(). drive Initialize() should be called again.
  m.SetControlWord(kFaultResetControlWord);
  ASSERT_EQ(m.Update(status_register, gate_driver_status_registers),
            Ds402State::SwitchOnDisabled());

  // Check the bits associated to CiA 402 faults in the status register have
  // been reset.
  ASSERT_EQ(status_register & kManufacturerStatusRegisterCia402FaultMask,
            ManufacturerStatusRegisterErrorCodes(0u));

  // Check other bits have not been reset
  ASSERT_EQ(status_register | ~kManufacturerStatusRegisterCia402FaultMask,
            status_register);
}

TEST(Ds402StateMachineTest, GateDriverStatusRegistersAreSet) {
  TestDrive drive;
  Ds402StateMachine m(drive);
  ManufacturerStatusRegisterErrorCodes status_register =
      ManufacturerStatusRegisterErrorCodes(0u);
  GateDriverStatusRegisters gate_driver_status_registers;

  // Set some bits.
  drive.gate_driver_status_registers_.fault_register_1 = 0x2450;
  drive.gate_driver_status_registers_.fault_register_2 = 0xf0fa;

  m.Update(status_register, gate_driver_status_registers);

  // Check the bits have been set.
  ASSERT_EQ(gate_driver_status_registers.fault_register_1,
            drive.gate_driver_status_registers_.fault_register_1);
  ASSERT_EQ(gate_driver_status_registers.fault_register_2,
            drive.gate_driver_status_registers_.fault_register_2);
}

}  // namespace
}  // namespace barkour
