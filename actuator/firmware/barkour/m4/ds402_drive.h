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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_DS402_DRIVE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_DS402_DRIVE_H_

#include <cstdint>
#include <optional>

#include "actuator/firmware/barkour/common/commutation.h"
#include "actuator/firmware/barkour/common/ds402_state_machine.h"
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "actuator/firmware/barkour/common/motor_control_modules/control_module_manager.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// An implementation of the Ds402DriveInterface, which is called by
// Ds402StateMachine.
//
// This manages the power electronics (i.e. the gate driver).
class Ds402Drive : public Ds402DriveInterface {
 public:
  static constexpr uint8_t kNumSupportedControlModes = 2;

  enum class SupportedControlModes : uint8_t {
    kZeroPwmOutput = 0,
    kCurrentControl,
  };

  explicit Ds402Drive(
      GateDriverInterface& gate_driver,
      ControlModuleManager<SupportedControlModes, kNumSupportedControlModes>&
          control_module_manager);

  // Transition functions, inherited from Ds402DriveInterface.
  //
  // The DS 402 State machine will continue to call these drive functions until
  // TransitionStatus::kFinished is returned.
  //
  // Any errors returned will result in a controller fault.
  pw::Result<TransitionStatus> Initialize() override;
  pw::Result<TransitionStatus> EnablePower() override;
  pw::Result<TransitionStatus> DisablePower() override;
  pw::Result<TransitionStatus> EnableDrive() override;
  pw::Result<TransitionStatus> DisableDrive() override;
  pw::Result<TransitionStatus> ActivateQuickStop() override;
  TransitionStatus HandleFault() override;

  pw::Status DoStateAction(
      Ds402State current_state,
      ManufacturerStatusRegisterErrorCodes& manufacturer_status_register,
      GateDriverStatusRegisters& gate_driver_status_registers) override;

  // Getter methods for latest internal sensor and control state.
  std::optional<RotatingFrameVoltages> GetRotatingFrameVoltages() const;
  std::optional<ThreePhasePwmCommands> GetPhasePwmCommands() const;

  // Setter methods for sensor readings, external inputs other than DS402 state
  // machine commands.
  void SetSensorReadings(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities);
  void SetControlReferences(const ControlReferences& references);
  void SetExternalError(bool external_error);

 private:
  using ControlModuleManagerType =
      ::barkour::ControlModuleManager<SupportedControlModes,
                                 kNumSupportedControlModes>;
  void SafetyCleanup();

  GateDriverInterface& gate_driver_;
  ControlModuleManagerType& control_module_manager_;

  ControlReferences references_;
  bool external_error_;

  std::optional<SensorReadings> sensor_readings_;
  std::optional<SensorDerivedQuantities> sensor_derived_quantities_;
  std::optional<RotatingFrameVoltages> rotating_frame_voltage_commands_;
  std::optional<ThreePhasePwmCommands> phase_duty_cycles_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_DS402_DRIVE_H_
