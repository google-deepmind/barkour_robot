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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CURRENT_CONTROL_MODULE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CURRENT_CONTROL_MODULE_H_

#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"

namespace barkour {

// Control module for current control.
class CurrentControlModule : public MotorControlModule {
 public:
  // Factory method to build a module instance.
  //
  // This makes appropriate checks on the values of the parameters passed, and
  // returns an invalid argument error if any of the checks fail, or a
  // CurrentControlModule instance if they all pass.
  //
  // Args:
  // - current_limit: Limit on the absolute value of current which can be
  //   commanded using this control module. Larger requested currents will be
  //   clipped to this (absolute) value. Must be >= 0.
  // - foc: reference to the Foc object instance
  static pw::Result<CurrentControlModule> Build(float current_limit,
                                                FocInterface& foc);

  ~CurrentControlModule() override = default;

 private:
  CurrentControlModule(float current_limit, FocInterface& foc);

  pw::Result<PossibleStatesFromStarting> DoStart(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  PossibleStatesFromStopping DoStop(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  void DoForceStop() override;

  // Steps the control module while it is in the "started" phase, and produces
  // output voltages to control the motor with.
  StateAndRotatingFrameVoltages<PossibleStatesFromStarted> DoStep(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references) override;

  float current_limit_;

  FocInterface& foc_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CURRENT_CONTROL_MODULE_H_
