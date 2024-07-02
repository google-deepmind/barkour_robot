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

#include "actuator/firmware/barkour/drivers/testing/fake_gate_driver.h"

#include <algorithm>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_assert/check.h"  // NOLINT
#include "pw_assert/internal/check_impl.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

FakeGateDriver::FakeGateDriver()
    : target_state_(GateDriverState::PowerOff()),
      current_state_(GateDriverState::PowerOff()),
      duty_cycles_{0, 0, 0},
      voltage_on_sto_lines_(false) {}

GateDriverState FakeGateDriver::CurrentState() const { return current_state_; }

bool FakeGateDriver::VoltageOnStoLines() const { return voltage_on_sto_lines_; }
bool FakeGateDriver::VoltageOnStoLine1() const { return voltage_on_sto_lines_; }
bool FakeGateDriver::VoltageOnStoLine2() const { return voltage_on_sto_lines_; }

GateDriverState FakeGateDriver::Update() {
  switch (target_state_) {
    case GateDriverState::PowerOff():
      current_state_ = GateDriverState::PowerOff();
      break;
    case GateDriverState::PowerOnGateEnabled():
      current_state_ = voltage_on_sto_lines_
                           ? GateDriverState::PowerOnGateEnabled()
                           : GateDriverState::WaitingForPowerOn();
      break;
    default:
      PW_CRASH("Invalid target state in FakeGateDriver: %s.",
               target_state_.str());
  }
  return current_state_;
}

pw::Status FakeGateDriver::SetTargetState(const GateDriverState& target_state) {
  switch (target_state) {
    case GateDriverState::PowerOff():
    case GateDriverState::PowerOnGateEnabled():
      target_state_ = target_state;
      return pw::OkStatus();
    default:
      return pw::Status::InvalidArgument();
  }
}

pw::Result<PhaseSampleSelection> FakeGateDriver::SetDutyCycles(
    float da, float db, float dc, bool ignore_current_state) {
  if (!ignore_current_state &&
      CurrentState() != GateDriverState::PowerOnGateEnabled()) {
    return pw::Status::FailedPrecondition();
  }

  da = std::clamp(da, 0.f, 1.f);
  db = std::clamp(db, 0.f, 1.f);
  dc = std::clamp(dc, 0.f, 1.f);

  duty_cycles_ = {da, db, dc};

  PhaseSampleSelection phase_select;

  if (da < db) {
    if (dc < da) {
      phase_select = PhaseSampleSelection::kAB;  // C min - ignored.
    } else {
      phase_select = PhaseSampleSelection::kBC;  // A min - ignored.
    }
    // db > da
  } else if (dc < db) {
    phase_select = PhaseSampleSelection::kAB;  // C min - ignored.
  } else {
    phase_select = PhaseSampleSelection::kAC;  // B min - ignored.
  }

  return phase_select;
}

GateDriverStatusRegisters FakeGateDriver::GetFaultStatusRegisters() {
  return status_registers_;
}

}  // namespace barkour
