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

#include "actuator/firmware/barkour/common/motor_control_modules/zero_output_control_module.h"

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

ZeroOutputControlModule::ZeroOutputControlModule(
    GateDriverInterface& gate_driver)
    : gate_driver_(gate_driver) {}

pw::Result<MotorControlModule::PossibleStatesFromStarting>
ZeroOutputControlModule::DoStart(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  // Immediate start as long as gate driver has started.
  return gate_driver_.CurrentState() == GateDriverState::PowerOnGateEnabled()
             ? pw::Result<PossibleStatesFromStarting>(
                   PossibleStatesFromStarting::kStarted)
             : pw::Result<PossibleStatesFromStarting>(
                   pw::Status::FailedPrecondition());
}

MotorControlModule::PossibleStatesFromStopping ZeroOutputControlModule::DoStop(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  // Immediate stop.
  return PossibleStatesFromStopping::kStopped;
}

void ZeroOutputControlModule::DoForceStop() {}

MotorControlModule::StateAndRotatingFrameVoltages<
    MotorControlModule::PossibleStatesFromStarted>
ZeroOutputControlModule::DoStep(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities,
    const ControlReferences& references) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  (void)references;
  gate_driver_.SetDutyCycles(0.5, 0.5, 0.5).IgnoreError();

  return {.state = PossibleStatesFromStarted::kStarted,
          .voltages = {.quadrature_voltage = 0, .direct_voltage = 0}};
}

}  // namespace barkour
