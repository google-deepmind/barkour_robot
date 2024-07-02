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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_ZERO_OUTPUT_CONTROL_MODULE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_ZERO_OUTPUT_CONTROL_MODULE_H_

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"

namespace barkour {

// Control module which always produces zero output. Useful as a way to
// implement a simple motor braking controller.
class ZeroOutputControlModule : public MotorControlModule {
 public:
  explicit ZeroOutputControlModule(GateDriverInterface& gate_driver);
  ~ZeroOutputControlModule() override = default;

 private:
  pw::Result<PossibleStatesFromStarting> DoStart(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  PossibleStatesFromStopping DoStop(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  void DoForceStop() override;

  StateAndRotatingFrameVoltages<PossibleStatesFromStarted> DoStep(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references) override;

 private:
  GateDriverInterface& gate_driver_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_ZERO_OUTPUT_CONTROL_MODULE_H_
