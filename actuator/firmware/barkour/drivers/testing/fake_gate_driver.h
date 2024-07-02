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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_GATE_DRIVER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_GATE_DRIVER_H_

#include <array>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// Fake gate driver instance for use in tests.
class FakeGateDriver : public GateDriverInterface {
 public:
  // Constructor.
  //
  // Sets the current and target states to `PowerOff()`, duty cycles to zero and
  // `voltage_on_sto_lines_` to false.
  FakeGateDriver();

  // Returns `current_state_`.
  GateDriverState CurrentState() const override;

  // The following three methods all return the value of
  // `voltage_on_sto_lines_`.
  bool VoltageOnStoLines() const override;
  bool VoltageOnStoLine1() const override;
  bool VoltageOnStoLine2() const override;

  // After a call to Update, the current state is set according to the
  // combination of `target_state_` and `voltage_on_sto_lines_`, and this state
  // is returned.
  //
  // Target state = PowerOff
  // => current_state_ <- PowerOff
  //
  // Target state = PowerOnGateEnabled, voltage on STO lines = true
  // => current_state_ <- PowerOn
  //
  // Target state = PowerOnGateEnabled, voltage on STO lines = false
  // => current_state_ <- WaitingforPowerOn
  GateDriverState Update() override;

  // Sets the desired state of the gate driver.
  //
  // Args:
  // - state: Desired state of the gate driver. This should be either
  //   `PowerOff()` or `PowerOnGateEnabled()`.
  //
  // Returns:
  // - Ok: if the target state is valid.
  // - Invalid argument error: If state is not a permissible target state.
  pw::Status SetTargetState(const GateDriverState& target_state) override;

  // Controls the duty cycles for the PWM outputs.
  //
  // Args:
  // - duty_cycles: da,db,dc 3 floating point numbers between 0 and 1
  //   (inclusive) specifying the PWM duty cycle, one value for each phase.
  //   Values outside of [0, 1] will be clipped to this range.
  // - ignore_current_state: if true, this method does not verify that the gate
  //   driver is currently in `PowerOnGateEnabled()` mode.
  //
  // Returns:
  // - If the PWM duty cycles were updated successfully, returns code to
  // indicate
  //   which phase currents to sample next, else a failure status code:
  // - Invalid argument error: If pwm.size()!=3 or a duty cycle exceeded
  //   the range.
  // - Failed precondition error: If ignore_current_state is false and the
  //   gate driver is not in the PowerOnGateEnabled state.
  pw::Result<PhaseSampleSelection> SetDutyCycles(
      float da, float db, float dc, bool ignore_current_state) override;

  GateDriverStatusRegisters GetFaultStatusRegisters() override;

  // Attributes are all public for ease of testing.
  GateDriverState target_state_;
  GateDriverState current_state_;
  std::array<float, 3> duty_cycles_;

  // `voltage_on_sto_lines_` controls the behaviour of the gate driver states,
  // and should be set manually by the user in tests.
  bool voltage_on_sto_lines_;

  // Only used for reporting. Should be manually set by the user in tests.
  GateDriverStatusRegisters status_registers_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_GATE_DRIVER_H_
