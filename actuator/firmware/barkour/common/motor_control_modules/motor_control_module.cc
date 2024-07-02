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

#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"

#include "pw_assert/check.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

namespace {

template <typename TransitionStateType>
MotorControlModule::State FromTransitionState(TransitionStateType next_state);

template <>
MotorControlModule::State
FromTransitionState<MotorControlModule::PossibleStatesFromStarting>(
    MotorControlModule::PossibleStatesFromStarting next_state) {
  switch (next_state) {
    case MotorControlModule::PossibleStatesFromStarting::kStarting:
      return MotorControlModule::State::kStarting;
    case MotorControlModule::PossibleStatesFromStarting::kStarted:
      return MotorControlModule::State::kStarted;
    case MotorControlModule::PossibleStatesFromStarting::kStopping:
      return MotorControlModule::State::kStopping;
    case MotorControlModule::PossibleStatesFromStarting::kStopped:
      return MotorControlModule::State::kStopped;
  }

  // Should never get here, unless someone starts passing round invalid enums.
  PW_CRASH("Invalid enum for PossibleStatesFromStarting.");
}

template <>
MotorControlModule::State
FromTransitionState<MotorControlModule::PossibleStatesFromStarted>(
    MotorControlModule::PossibleStatesFromStarted next_state) {
  switch (next_state) {
    case MotorControlModule::PossibleStatesFromStarted::kStarted:
      return MotorControlModule::State::kStarted;
    case MotorControlModule::PossibleStatesFromStarted::kStopping:
      return MotorControlModule::State::kStopping;
    case MotorControlModule::PossibleStatesFromStarted::kStopped:
      return MotorControlModule::State::kStopped;
  }

  // Should never get here, unless someone starts passing round invalid enums.
  PW_CRASH("Invalid enum for PossibleStatesFromStarted.");
}

template <>
MotorControlModule::State
FromTransitionState<MotorControlModule::PossibleStatesFromStopping>(
    MotorControlModule::PossibleStatesFromStopping next_state) {
  switch (next_state) {
    case MotorControlModule::PossibleStatesFromStopping::kStopping:
      return MotorControlModule::State::kStopping;
    case MotorControlModule::PossibleStatesFromStopping::kStopped:
      return MotorControlModule::State::kStopped;
  }

  // Should never get here, unless someone starts passing round invalid enums.
  PW_CRASH("Invalid enum for PossibleStatesFromStarted.");
}

}  // namespace

MotorControlModule::MotorControlModule() : current_state_(State::kStopped) {}

pw::Status MotorControlModule::Start(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  if (current_state_ != State::kStopped) {
    return pw::Status::FailedPrecondition();
  }

  PW_TRY_ASSIGN(PossibleStatesFromStarting next_state,
                DoStart(sensor_readings, sensor_derived_quantities));
  current_state_ = FromTransitionState(next_state);
  return pw::OkStatus();
}

pw::Status MotorControlModule::Stop(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  if (current_state_ != State::kStarting && current_state_ != State::kStarted) {
    return pw::Status::FailedPrecondition();
  }

  PossibleStatesFromStopping next_state =
      DoStop(sensor_readings, sensor_derived_quantities);

  current_state_ = FromTransitionState(next_state);
  return pw::OkStatus();
}

pw::Status MotorControlModule::ForceStop() {
  if (current_state_ != State::kStarting && current_state_ != State::kStarted &&
      current_state_ != State::kStopping) {
    return pw::Status::FailedPrecondition();
  }

  DoForceStop();
  current_state_ = State::kStopped;
  return pw::OkStatus();
}

pw::Result<RotatingFrameVoltages> MotorControlModule::Step(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities,
    const ControlReferences& references) {
  switch (current_state_) {
    case State::kStopped:
      return pw::Status::FailedPrecondition();

    case State::kStarting: {
      PossibleStatesFromStarting next_state =
          DoStepWhileStarting(sensor_readings, sensor_derived_quantities);
      current_state_ = FromTransitionState(next_state);
      return RotatingFrameVoltages{};  // Will be all zeros.
    }

    case State::kStarted: {
      StateAndRotatingFrameVoltages<PossibleStatesFromStarted>
          state_and_output =
              DoStep(sensor_readings, sensor_derived_quantities, references);
      current_state_ = FromTransitionState(state_and_output.state);
      return state_and_output.voltages;
    }

    case State::kStopping: {
      StateAndRotatingFrameVoltages<PossibleStatesFromStopping>
          state_and_output =
              DoStepWhileStopping(sensor_readings, sensor_derived_quantities);
      current_state_ = FromTransitionState(state_and_output.state);
      return state_and_output.voltages;
    }
  }

  // Should never get here, unless someone starts passing round invalid enums.
  PW_CRASH("Invalid enum for MotorControlModule::State.");
}

MotorControlModule::State MotorControlModule::GetState() const {
  return current_state_;
}

MotorControlModule::PossibleStatesFromStarting
MotorControlModule::DoStepWhileStarting(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  return PossibleStatesFromStarting::kStarted;
}

MotorControlModule::StateAndRotatingFrameVoltages<
    MotorControlModule::PossibleStatesFromStopping>
MotorControlModule::DoStepWhileStopping(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  (void)sensor_readings;
  (void)sensor_derived_quantities;
  return {.state = PossibleStatesFromStopping::kStopped,
          .voltages = RotatingFrameVoltages{0, 0}};
}

}  // namespace barkour
