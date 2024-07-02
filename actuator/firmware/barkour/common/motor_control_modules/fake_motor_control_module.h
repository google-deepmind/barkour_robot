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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_FAKE_MOTOR_CONTROL_MODULE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_FAKE_MOTOR_CONTROL_MODULE_H_

#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"

namespace barkour {

// A fake motor control module, for testing.
class FakeMotorControlModule : public MotorControlModule {
 public:
  explicit FakeMotorControlModule(int num_starting_calls,
                                  int num_stopping_calls);

  ~FakeMotorControlModule() override = default;

  void SetVoltagesToReturn(const RotatingFrameVoltages& voltages);

 private:
  pw::Result<PossibleStatesFromStarting> DoStart(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  PossibleStatesFromStopping DoStop(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  void DoForceStop() override;

  PossibleStatesFromStarting DoStepWhileStarting(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

  StateAndRotatingFrameVoltages<PossibleStatesFromStarted> DoStep(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references) override;

  StateAndRotatingFrameVoltages<PossibleStatesFromStopping> DoStepWhileStopping(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) override;

 private:
  int num_starting_calls_;
  int num_stopping_calls_;

  int actual_starting_calls_;
  int actual_stopping_calls_;

  RotatingFrameVoltages voltages_to_return_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_FAKE_MOTOR_CONTROL_MODULE_H_
