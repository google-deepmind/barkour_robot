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

#include "actuator/firmware/barkour/m4/ds402_drive.h"

#include <optional>

#include "actuator/firmware/barkour/common/commutation.h"
#include "actuator/firmware/barkour/common/ds402_state_machine.h"
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

Ds402Drive::Ds402Drive(GateDriverInterface& gate_driver,
                       ControlModuleManagerType& control_module_manager)
    : gate_driver_(gate_driver),
      control_module_manager_(control_module_manager),
      external_error_(false) {}

pw::Result<Ds402DriveInterface::TransitionStatus> Ds402Drive::Initialize() {
  // Can't start if an external error.
  if (external_error_) {
    return pw::Status::Internal();
  }

  // If no errors, transition immediately.
  return TransitionStatus::kFinished;
}

pw::Result<Ds402DriveInterface::TransitionStatus> Ds402Drive::EnablePower() {
  if (gate_driver_.CurrentState() != GateDriverState::PowerOnGateEnabled()) {
    gate_driver_.SetTargetState(barkour::GateDriverState::PowerOnGateEnabled());
  }

  gate_driver_.Update();

  if (gate_driver_.CurrentState() ==
      barkour::GateDriverState::PowerOnGateEnabled()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for gate driver.
  return TransitionStatus::kOngoing;
}

pw::Result<Ds402DriveInterface::TransitionStatus> Ds402Drive::DisablePower() {
  if (gate_driver_.CurrentState() != GateDriverState::PowerOff()) {
    gate_driver_.SetTargetState(barkour::GateDriverState::PowerOff());
  }

  gate_driver_.Update();

  if (gate_driver_.CurrentState() == barkour::GateDriverState::PowerOff()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for gate driver.
  return TransitionStatus::kOngoing;
}

pw::Result<Ds402DriveInterface::TransitionStatus> Ds402Drive::EnableDrive() {
  if (!sensor_readings_.has_value() ||
      !sensor_derived_quantities_.has_value()) {
    return pw::Status::NotFound();
  }

  // Switch the desired control module to current control if needed.
  if (control_module_manager_.GetSelectedControlModuleLabel() !=
      SupportedControlModes::kCurrentControl) {
    PW_TRY(control_module_manager_.SetSelectedControlModule(
        SupportedControlModes::kCurrentControl));
  }

  // Start the control module manager if needed.
  if (!control_module_manager_.IsRunning() ||
      // IsRunning() returning true implies there is an active control mode.
      (control_module_manager_.GetActiveControlModuleState().value() ==
           MotorControlModule::State::kStopping ||
       control_module_manager_.GetActiveControlModuleState().value() ==
           MotorControlModule::State::kStopped)) {
    PW_TRY(control_module_manager_.Start(sensor_readings_.value(),
                                         sensor_derived_quantities_.value()));
  }

  if (control_module_manager_.IsSelectedControlModuleReady()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for controller.
  return TransitionStatus::kOngoing;
}

pw::Result<Ds402DriveInterface::TransitionStatus> Ds402Drive::DisableDrive() {
  if (!sensor_readings_.has_value() ||
      !sensor_derived_quantities_.has_value()) {
    return pw::Status::NotFound();
  }

  // Stop the control module.
  if (control_module_manager_.IsRunning() &&
      // IsRunning() returning true implies there is an active control mode.
      (control_module_manager_.GetActiveControlModuleState().value() ==
           MotorControlModule::State::kStarting ||
       control_module_manager_.GetActiveControlModuleState().value() ==
           MotorControlModule::State::kStarted)) {
    PW_TRY(control_module_manager_.Stop(sensor_readings_.value(),
                                        sensor_derived_quantities_.value()));
  }

  if (!control_module_manager_.IsRunning()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for controller.
  return TransitionStatus::kOngoing;
}

pw::Result<Ds402DriveInterface::TransitionStatus>
Ds402Drive::ActivateQuickStop() {
  // Switch the desired control module to braking control if needed.
  //
  // Note that the controller manager will handle stopping the current control
  // module and starting the braking control module. We can guarantee that the
  // controller manager has already been started, because the
  // `ActivateQuickStop` transition can only be triggered from the operation
  // enabled CiA 402 state.
  if (control_module_manager_.GetSelectedControlModuleLabel() !=
      SupportedControlModes::kZeroPwmOutput) {
    PW_TRY(control_module_manager_.SetSelectedControlModule(
        SupportedControlModes::kZeroPwmOutput));
  }

  if (control_module_manager_.IsSelectedControlModuleReady()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for controller.
  return TransitionStatus::kOngoing;
}

Ds402DriveInterface::TransitionStatus Ds402Drive::HandleFault() {
  // First, attempt to stop the controller, if this hasn't been done already.
  if (control_module_manager_.IsRunning()) {
    switch (control_module_manager_.GetActiveControlModuleState().value()) {
      case MotorControlModule::State::kStarted:
      case MotorControlModule::State::kStarting: {
        if (!sensor_readings_.has_value() ||
            !sensor_derived_quantities_.has_value()) {
          // This shouldn't happen, but if it does we can't return an error
          // since this is already the error handling pathway, so force a
          // controller stop.
          control_module_manager_.ForceStop();
        } else {
          control_module_manager_.Stop(*sensor_readings_,
                                       *sensor_derived_quantities_);
        }
        break;
      }
      case MotorControlModule::State::kStopping: {
        if (!sensor_readings_.has_value() ||
            !sensor_derived_quantities_.has_value()) {
          // Same as above.
          control_module_manager_.ForceStop();
        }
        break;
      }
      case MotorControlModule::State::kStopped: {
        // Do nothing.
        break;
      }
      default: {
        break;
      }
    }
    // Haven't disabled the controller yet.
    return TransitionStatus::kOngoing;
  }

  // If we reach here, we have disabled the controller, so now disable the
  // power.
  if (gate_driver_.CurrentState() != GateDriverState::PowerOff()) {
    gate_driver_.SetTargetState(barkour::GateDriverState::PowerOff());
  }

  gate_driver_.Update();

  if (gate_driver_.CurrentState() == barkour::GateDriverState::PowerOff()) {
    // Successful transition.
    return TransitionStatus::kFinished;
  }

  // Still waiting for gate driver.
  return TransitionStatus::kOngoing;
}

pw::Status Ds402Drive::DoStateAction(
    Ds402State current_state,
    ManufacturerStatusRegisterErrorCodes& manufacturer_status_register,
    GateDriverStatusRegisters& gate_driver_status_registers) {
  // Reset control outputs.
  rotating_frame_voltage_commands_ = std::nullopt;
  phase_duty_cycles_ = std::nullopt;

  // In all states, update the gate driver.
  GateDriverState gate_driver_state = gate_driver_.Update();

  // If there is an external fault state and we're not already in fault, cause
  // a fault now. The external fault will already be signalled in the status
  // register, so we don't need to do that here.
  if (current_state != Ds402State::kFault &&
      current_state != Ds402State::kFaultReactionActive &&
      current_state != Ds402State::kNotReadyToSwitchOn && external_error_) {
    PW_LOG_WARN("External error occurred, shutting down motor.");
    SafetyCleanup();
    return pw::Status::Internal();
  }

  // Check for STO safety condition.
  if (current_state == Ds402State::SwitchedOn() ||
      current_state == Ds402State::OperationEnabled() ||
      current_state == Ds402State::QuickStopActive()) {
    if (gate_driver_state != barkour::GateDriverState::PowerOnGateEnabled()) {
      PW_LOG_WARN(
          "Gate driver state machine state changed, shutting down motor.");
      manufacturer_status_register |= ManufacturerStatusRegisterErrorCodes::
          kGateDriverStateChangeWhileVoltageApplied;
      gate_driver_status_registers = gate_driver_.GetFaultStatusRegisters();

      if (!gate_driver_.VoltageOnStoLine1()) {
        PW_LOG_WARN(
            "FB_STO_1 line observed down while gate driver state changed");
        manufacturer_status_register |= ManufacturerStatusRegisterErrorCodes::
            kStoLine1DownWhileVoltageApplied;
      }
      if (!gate_driver_.VoltageOnStoLine2()) {
        PW_LOG_WARN(
            "FB_STO_2 line observed down while gate driver state changed");
        manufacturer_status_register |= ManufacturerStatusRegisterErrorCodes::
            kStoLine2DownWhileVoltageApplied;
      }
      SafetyCleanup();
      return pw::Status::Internal();
    }
  }

  if (current_state == Ds402State::OperationEnabled() ||
      current_state == Ds402State::QuickStopActive()) {
    // The operation enabled and quick-stop states require the motor control
    // steps to be run.
    if (!sensor_readings_.has_value() ||
        !sensor_derived_quantities_.has_value()) {
      return pw::Status::NotFound();
    }

    PW_TRY_ASSIGN(rotating_frame_voltage_commands_,
                  control_module_manager_.Step(
                      sensor_readings_.value(),
                      sensor_derived_quantities_.value(), references_));

  } else if (current_state == Ds402State::SwitchedOn()) {
    // In states where the power electronics are on but the controller is not
    // active, set the duty cycles to zero.
    PW_TRY(gate_driver_.SetDutyCycles(0.5f, 0.5f, 0.5f));  // Duty =0.5 = 0V.
  }

  return pw::OkStatus();
}

std::optional<RotatingFrameVoltages> Ds402Drive::GetRotatingFrameVoltages()
    const {
  return rotating_frame_voltage_commands_;
}
std::optional<ThreePhasePwmCommands> Ds402Drive::GetPhasePwmCommands() const {
  return phase_duty_cycles_;
}

// Setter methods for sensor readings, external inputs other than DS402 state
// machine commands.
void Ds402Drive::SetSensorReadings(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  sensor_readings_ = sensor_readings;
  sensor_derived_quantities_ = sensor_derived_quantities;
}

void Ds402Drive::SetControlReferences(const ControlReferences& references) {
  references_ = references;
}

void Ds402Drive::SetExternalError(bool external_error) {
  external_error_ = external_error;
}

void Ds402Drive::SafetyCleanup() {
  // Ensure PWM duty cycles are set to zero as a safety measure.
  static bool ignore_current_state = true;

  (void)gate_driver_.SetDutyCycles(0.5f, 0.5f, 0.5f, ignore_current_state);

  if (control_module_manager_.IsRunning()) {
    MotorControlModule::State control_state =
        control_module_manager_.GetActiveControlModuleState().value();

    if (control_state == MotorControlModule::State::kStarting ||
        control_state == MotorControlModule::State::kStarted ||
        control_state == MotorControlModule::State::kStopping) {
      control_module_manager_.ForceStop();
    }
  }
}

}  // namespace barkour.
