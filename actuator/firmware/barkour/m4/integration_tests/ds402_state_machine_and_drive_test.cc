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
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "actuator/firmware/barkour/common/motor_control_modules/control_module_manager.h"
#include "actuator/firmware/barkour/common/motor_control_modules/current_control_module.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/motor_control_modules/zero_output_control_module.h"
#include "actuator/firmware/barkour/drivers/testing/fake_foc.h"
#include "actuator/firmware/barkour/drivers/testing/fake_gate_driver.h"
#include "actuator/firmware/barkour/m4/ds402_drive.h"
#include "pw_containers/flat_map.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(Ds402StateMachineAndDriveTest,
     UnfinishedSwitchingOnTransitionCanBeUndone) {
  FakeFoc fake_foc;
  // See b/244534575 for the use case that this is testing.

  // Set up a CiA 402 state machine and drive, with a fake gate driver.
  CurrentControlModule current_control_module =
      CurrentControlModule::Build(
          /* current_limit = */ 20.0f, fake_foc)
          .value();

  FakeGateDriver gate_driver;
  ZeroOutputControlModule zero_output_control_module(gate_driver);

  pw::containers::FlatMap<Ds402Drive::SupportedControlModes,
                          MotorControlModule*,
                          Ds402Drive::kNumSupportedControlModes>
      motor_control_modules({{
          {Ds402Drive::SupportedControlModes::kCurrentControl,
           &current_control_module},
          {Ds402Drive::SupportedControlModes::kZeroPwmOutput,
           &zero_output_control_module},
      }});

  ControlModuleManager<Ds402Drive::SupportedControlModes,
                       Ds402Drive::kNumSupportedControlModes>
      control_module_manager =
          ControlModuleManager<Ds402Drive::SupportedControlModes,
                               Ds402Drive::kNumSupportedControlModes>::
              Build(motor_control_modules)
                  .value();

  // Set up DS402 drive and state machine.
  Ds402Drive ds402_drive(gate_driver, control_module_manager);
  Ds402StateMachine ds402_state_machine(ds402_drive);

  ManufacturerStatusRegisterErrorCodes error_bitfield;
  GateDriverStatusRegisters gate_driver_status_registers;

  // Switch off STO lines
  gate_driver.voltage_on_sto_lines_ = false;

  // Update a few times so the state machine moves out of "not ready".
  for (int i = 0; i < 5; ++i) {
    ds402_state_machine.Update(error_bitfield, gate_driver_status_registers);
  }
  ASSERT_EQ((Ds402State::Code)ds402_state_machine.GetState(),
            Ds402State::kSwitchOnDisabled);

  // Request state transitions until the state is "ready to switch
  // on", and the requested state is "switched on".

  // Switch on disabled -> ready to switch on. 5 steps should be enough.
  ds402_state_machine.SetControlWord(0b110);
  for (int i = 0; i < 5; ++i) {
    ds402_state_machine.Update(error_bitfield, gate_driver_status_registers);
  }
  ASSERT_EQ((Ds402State::Code)ds402_state_machine.GetState(),
            Ds402State::kReadyToSwitchOn);

  // Ready to switch on -> operation enabled. This transition is requested but
  // should not complete as the STO lines are down on the gate driver.
  ds402_state_machine.SetControlWord(0b1111);
  for (int i = 0; i < 5; ++i) {
    ds402_state_machine.Update(error_bitfield, gate_driver_status_registers);
  }
  ASSERT_EQ((Ds402State::Code)ds402_state_machine.GetState(),
            Ds402State::kReadyToSwitchOn);

  // Now request a transition back down to "switch on disabled".
  ds402_state_machine.SetControlWord(0);
  for (int i = 0; i < 5; ++i) {
    ds402_state_machine.Update(error_bitfield, gate_driver_status_registers);
  }
  ASSERT_EQ((Ds402State::Code)ds402_state_machine.GetState(),
            Ds402State::kSwitchOnDisabled);

  // Switch on the STO lines and update a few more times. Check state hasn't
  // changed.
  gate_driver.voltage_on_sto_lines_ = true;
  for (int i = 0; i < 5; ++i) {
    ds402_state_machine.Update(error_bitfield, gate_driver_status_registers);
  }
  ASSERT_EQ((Ds402State::Code)ds402_state_machine.GetState(),
            Ds402State::kSwitchOnDisabled);

  // Gate driver should now be disabled.
  EXPECT_EQ(gate_driver.CurrentState(), GateDriverState::PowerOff());
}

}  // namespace
}  // namespace barkour
