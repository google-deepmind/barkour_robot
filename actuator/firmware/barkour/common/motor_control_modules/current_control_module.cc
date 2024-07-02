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

#include "actuator/firmware/barkour/common/motor_control_modules/current_control_module.h"

#include <algorithm>

#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

namespace {

pw::Status CheckNonNegative(float arg) {
  if (arg < 0) {
    return pw::Status::InvalidArgument();
  }
  return pw::OkStatus();
}

}  // namespace

CurrentControlModule::CurrentControlModule(float current_limit,
                                           FocInterface& foc)
    : current_limit_(current_limit), foc_(foc) {}

pw::Result<CurrentControlModule> CurrentControlModule::Build(
    float current_limit, FocInterface& foc) {
  // Verify.
  PW_TRY(CheckNonNegative(current_limit));

  return CurrentControlModule(current_limit, foc);
}

pw::Result<MotorControlModule::PossibleStatesFromStarting>
CurrentControlModule::DoStart(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_derived_quantities;
  // Measure bus voltage and set saturation accordingly.

  foc_.Start(sensor_readings.bus_voltage);
  // Immediate start.
  return PossibleStatesFromStarting::kStarted;
}

MotorControlModule::PossibleStatesFromStopping CurrentControlModule::DoStop(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  // Immediate stop.
  foc_.Stop();

  return PossibleStatesFromStopping::kStopped;
}

void CurrentControlModule::DoForceStop() {}

// Steps the control module while it is in the "started" phase, and produces
// output voltages to control the motor with.
MotorControlModule::StateAndRotatingFrameVoltages<
    MotorControlModule::PossibleStatesFromStarted>
CurrentControlModule::DoStep(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities,
    const ControlReferences& references) {
  (void)sensor_derived_quantities;
  float reference_current =
      std::clamp(references.reference_current, -current_limit_, current_limit_);

  StateAndRotatingFrameVoltages<PossibleStatesFromStarted> output;

  // Keep running.
  output.state = PossibleStatesFromStarted::kStarted;

  foc_.SetTargetQ(reference_current, sensor_readings.bus_voltage);

  FocInterface::FocState foc_state = foc_.GetState();

  output.voltages.direct_voltage = foc_state.v_d_;
  output.voltages.quadrature_voltage = foc_state.v_q_;

  return output;
}

}  // namespace barkour
