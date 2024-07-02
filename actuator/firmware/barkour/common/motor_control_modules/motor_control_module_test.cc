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

#include "actuator/firmware/barkour/common/motor_control_modules/fake_motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(MotorControlModuleTest, CanOnlyStartFromStoppedState) {
  FakeMotorControlModule control_module(1, 1);

  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);

  ASSERT_TRUE(
      control_module.Start(SensorReadings{}, SensorDerivedQuantities{}).ok());

  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarting);
  EXPECT_EQ(control_module.Start(SensorReadings{}, SensorDerivedQuantities{}),
            pw::Status::FailedPrecondition());

  // Run once to start.
  ASSERT_TRUE(control_module.Step(SensorReadings{}, SensorDerivedQuantities{},
                        ControlReferences{}).ok());
  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);
  EXPECT_EQ(control_module.Start(SensorReadings{}, SensorDerivedQuantities{}),
            pw::Status::FailedPrecondition());

  // Run with negative control input, to force a change to "stopping".
  ASSERT_TRUE(control_module.Step(SensorReadings{}, SensorDerivedQuantities{},
                        ControlReferences{.reference_current = -1}).ok());
  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStopping);
  EXPECT_EQ(control_module.Start(SensorReadings{}, SensorDerivedQuantities{}),
            pw::Status::FailedPrecondition());
}

TEST(MotorControlModuleTest, CannotStopFromStoppedState) {
  FakeMotorControlModule control_module(0, 0);
  EXPECT_EQ(control_module.Stop(SensorReadings{}, SensorDerivedQuantities{}),
            pw::Status::FailedPrecondition());
}

TEST(MotorControlModuleTest, CanStopFromStartingState) {
  FakeMotorControlModule control_module(1, 0);

  ASSERT_TRUE(
      control_module.Start(SensorReadings{}, SensorDerivedQuantities{}).ok());

  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarting);

  EXPECT_TRUE(
      control_module.Stop(SensorReadings{}, SensorDerivedQuantities{}).ok());

  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);
}

TEST(MotorControlModuleTest, SinglePassThroughLifecycle) {
  FakeMotorControlModule control_module(1, 1);

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);

  ASSERT_TRUE(
      control_module.Start(SensorReadings{}, SensorDerivedQuantities{}).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStarting);

  ASSERT_TRUE(control_module.Step(SensorReadings{}, SensorDerivedQuantities{},
                        ControlReferences{}).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  // Run once in started.
  ASSERT_TRUE(control_module.Step(SensorReadings{}, SensorDerivedQuantities{},
                        ControlReferences{}).ok());

  ASSERT_TRUE(
      control_module.Stop(SensorReadings{}, SensorDerivedQuantities{}).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopping);

  ASSERT_TRUE(control_module.Step(SensorReadings{}, SensorDerivedQuantities{},
                        ControlReferences{}).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);
}

}  // namespace
}  // namespace barkour
