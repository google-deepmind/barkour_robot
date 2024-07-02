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

#include "actuator/firmware/barkour/common/motor_control_modules/fake_motor_control_module.h"

#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

FakeMotorControlModule::FakeMotorControlModule(int num_starting_calls,
                                               int num_stopping_calls)
    : num_starting_calls_(num_starting_calls),
      num_stopping_calls_(num_stopping_calls),
      actual_starting_calls_(0),
      actual_stopping_calls_(0) {}

pw::Result<MotorControlModule::PossibleStatesFromStarting>
FakeMotorControlModule::DoStart(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  // Reset call counts.
  actual_starting_calls_ = 0;
  actual_stopping_calls_ = 0;

  return num_starting_calls_ <= 0 ? PossibleStatesFromStarting::kStarted
                                  : PossibleStatesFromStarting::kStarting;
}

MotorControlModule::PossibleStatesFromStopping FakeMotorControlModule::DoStop(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  return num_stopping_calls_ <= 0 ? PossibleStatesFromStopping::kStopped
                                  : PossibleStatesFromStopping::kStopping;
}

void FakeMotorControlModule::DoForceStop() {}

MotorControlModule::PossibleStatesFromStarting
FakeMotorControlModule::DoStepWhileStarting(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  return (++actual_starting_calls_ >= num_starting_calls_)
             ? PossibleStatesFromStarting::kStarted
             : PossibleStatesFromStarting::kStarting;
}

MotorControlModule::StateAndRotatingFrameVoltages
<MotorControlModule::PossibleStatesFromStarted> FakeMotorControlModule::DoStep(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities,
    const ControlReferences& references) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  StateAndRotatingFrameVoltages<PossibleStatesFromStarted> output;

  output.voltages = voltages_to_return_;

  // If the control reference is less than zero, stop, simulating a fault
  // condition.
  if (references.reference_current < 0) {
    output.state = num_stopping_calls_ <= 0
                       ? PossibleStatesFromStarted::kStopped
                       : PossibleStatesFromStarted::kStopping;
    return output;
  } else {
    output.state = PossibleStatesFromStarted::kStarted;
  }

  return output;
}

MotorControlModule::StateAndRotatingFrameVoltages<
    MotorControlModule::PossibleStatesFromStopping>
FakeMotorControlModule::DoStepWhileStopping(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  StateAndRotatingFrameVoltages<PossibleStatesFromStopping> output;

  if (++actual_stopping_calls_ >= num_starting_calls_) {
    output.state = PossibleStatesFromStopping::kStopped;
    output.voltages = {0, 0};
  } else {
    output.state = PossibleStatesFromStopping::kStopping;
    output.voltages = voltages_to_return_;
  }
  return output;
}

void FakeMotorControlModule::SetVoltagesToReturn(
    const RotatingFrameVoltages& voltages) {
  voltages_to_return_ = voltages;
}

}  // namespace barkour
