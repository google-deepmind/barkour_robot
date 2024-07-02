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

#include "actuator/firmware/barkour/common/motor_control_modules/control_module_manager.h"

#include "actuator/firmware/barkour/common/motor_control_modules/fake_motor_control_module.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_containers/flat_map.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

#define ASSERT_OK(expr) ASSERT_EQ(expr, pw::OkStatus())
#define ASSERT_RESULT_OK(expr) ASSERT_EQ((expr).status(), pw::OkStatus())

TEST(ControlModuleManagerTest, BuildWithNullControlModulePointerReturnsError) {
  FakeMotorControlModule fake(0, 0);

  // Make a map with one control module and one null pointer.
  pw::containers::FlatMap<int, MotorControlModule*, 2> controller_map(
      {{{0, &fake}, {1, nullptr}}});

  auto maybe_control_module_manager =
      ControlModuleManager<int, 2>::Build(controller_map);

  EXPECT_EQ(maybe_control_module_manager.status(),
            pw::Status::InvalidArgument());
}

TEST(ControlModuleManagerTest, RequestingInvalidControllerReturnsError) {
  FakeMotorControlModule fake_1(1, 1);
  FakeMotorControlModule fake_2(1, 1);

  pw::containers::FlatMap<int, MotorControlModule*, 2> controller_map(
      {{{1, &fake_2}, {2, &fake_2}}});

  auto maybe_control_module_manager =
      ControlModuleManager<int, 2>::Build(controller_map);

  ASSERT_RESULT_OK(maybe_control_module_manager);
  ControlModuleManager<int, 2>& control_module_manager =
      maybe_control_module_manager.value();

  // double-check the target state is as expected beforehand.
  ASSERT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 1);

  ASSERT_EQ(control_module_manager.SetSelectedControlModule(3),
            pw::Status::NotFound());

  // Check the desired control mode label hasn't changed.
  ASSERT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 1);
}

TEST(ControlModuleManagerTest, FullLifecycleWithOneControlModule) {
  FakeMotorControlModule fake(1, 1);

  // Make a map with one control module.
  pw::containers::FlatMap<int, MotorControlModule*, 1> controller_map(
      {{{1, &fake}}});

  auto maybe_control_module_manager =
      ControlModuleManager<int, 1>::Build(controller_map);

  ASSERT_RESULT_OK(maybe_control_module_manager);
  ControlModuleManager<int, 1>& control_module_manager =
      maybe_control_module_manager.value();

  EXPECT_FALSE(control_module_manager.IsRunning());
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().status(),
            pw::Status::FailedPrecondition());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleState().status(),
            pw::Status::FailedPrecondition());

  ASSERT_TRUE(
      control_module_manager.Start(SensorReadings{}, SensorDerivedQuantities{})
          .ok());

  EXPECT_TRUE(control_module_manager.IsRunning());
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 1);

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleState());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleState().value(),
            MotorControlModule::State::kStarting);

  ASSERT_TRUE(control_module_manager.Step(SensorReadings{},
                        SensorDerivedQuantities{}, ControlReferences{}).ok());

  EXPECT_TRUE(control_module_manager.IsRunning());
  EXPECT_TRUE(control_module_manager.IsSelectedControlModuleReady());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 1);

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleState());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleState().value(),
            MotorControlModule::State::kStarted);

  // Run once in started.
  RotatingFrameVoltages output_voltages = {.quadrature_voltage = 1.0,
                                           .direct_voltage = -4.7};
  fake.SetVoltagesToReturn(output_voltages);

  pw::Result<RotatingFrameVoltages> observed_output =
      control_module_manager.Step(SensorReadings{}, SensorDerivedQuantities{},
                                  ControlReferences{});

  ASSERT_RESULT_OK(observed_output);
  EXPECT_EQ(observed_output->direct_voltage, output_voltages.direct_voltage);
  EXPECT_EQ(observed_output->quadrature_voltage,
            output_voltages.quadrature_voltage);

  ASSERT_TRUE(control_module_manager.Stop(SensorReadings{},
                                          SensorDerivedQuantities{}).ok());

  EXPECT_TRUE(control_module_manager.IsRunning());
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 1);

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleState());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleState().value(),
            MotorControlModule::State::kStopping);

  ASSERT_TRUE(control_module_manager.Step(SensorReadings{},
                        SensorDerivedQuantities{}, ControlReferences{}).ok());

  EXPECT_FALSE(control_module_manager.IsRunning());
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().status(),
            pw::Status::FailedPrecondition());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleState().status(),
            pw::Status::FailedPrecondition());
}

TEST(ControlModuleManagerTest,
     SwitchBetweenControlModulesInstantaneousStopping) {
  FakeMotorControlModule fake_1(0, 0);
  FakeMotorControlModule fake_2(0, 0);

  pw::containers::FlatMap<int, MotorControlModule*, 2> controller_map(
      {{{1, &fake_1}, {2, &fake_2}}});

  auto maybe_control_module_manager =
      ControlModuleManager<int, 2>::Build(controller_map);

  ASSERT_RESULT_OK(maybe_control_module_manager);
  ControlModuleManager<int, 2>& control_module_manager =
      maybe_control_module_manager.value();

  // Set the desired control mode label to 2, and start up the controller.
  ASSERT_OK(control_module_manager.SetSelectedControlModule(2));
  EXPECT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 2);

  ASSERT_TRUE(
      control_module_manager.Start(SensorReadings{}, SensorDerivedQuantities{})
          .ok());

  EXPECT_TRUE(control_module_manager.IsRunning());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 2);

  // Check the correct controller has been started
  EXPECT_EQ(fake_2.GetState(), MotorControlModule::State::kStarted);
  EXPECT_EQ(fake_1.GetState(), MotorControlModule::State::kStopped);

  // Set the desired control mode label to 1. After the next step the new
  // controller should immediately be active.
  ASSERT_OK(control_module_manager.SetSelectedControlModule(1));
  EXPECT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 1);
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());

  ASSERT_TRUE(control_module_manager.Step(SensorReadings{},
                        SensorDerivedQuantities{}, ControlReferences{}).ok());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 1);
  EXPECT_TRUE(control_module_manager.IsSelectedControlModuleReady());

  EXPECT_EQ(fake_2.GetState(), MotorControlModule::State::kStopped);
  EXPECT_EQ(fake_1.GetState(), MotorControlModule::State::kStarted);
}

TEST(ControlModuleManagerTest, SwitchBetweenControlModulesDelayedStopping) {
  FakeMotorControlModule fake_1(1, 1);
  FakeMotorControlModule fake_2(1, 1);

  pw::containers::FlatMap<int, MotorControlModule*, 2> controller_map(
      {{{1, &fake_1}, {2, &fake_2}}});

  auto maybe_control_module_manager =
      ControlModuleManager<int, 2>::Build(controller_map);

  ASSERT_RESULT_OK(maybe_control_module_manager);
  ControlModuleManager<int, 2>& control_module_manager =
      maybe_control_module_manager.value();

  // Set the desired control mode label to 2, and start up the controller.
  ASSERT_OK(control_module_manager.SetSelectedControlModule(2));
  EXPECT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 2);

  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());

  ASSERT_TRUE(
      control_module_manager.Start(SensorReadings{}, SensorDerivedQuantities{})
          .ok());

  EXPECT_TRUE(control_module_manager.IsRunning());

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 2);

  ASSERT_TRUE(control_module_manager.Step(SensorReadings{},
                        SensorDerivedQuantities{}, ControlReferences{}).ok());

  // Check the correct controller has been started
  EXPECT_EQ(fake_2.GetState(), MotorControlModule::State::kStarted);
  EXPECT_EQ(fake_1.GetState(), MotorControlModule::State::kStopped);

  // Set the desired control mode label to 1. This should require two steps (one
  // to stop fake_2, and one to start fake_1), then the new control module
  // should be active.
  ASSERT_OK(control_module_manager.SetSelectedControlModule(1));
  EXPECT_EQ(control_module_manager.GetSelectedControlModuleLabel(), 1);
  EXPECT_FALSE(control_module_manager.IsSelectedControlModuleReady());

  for (int i = 0; i < 2; ++i) {
    ASSERT_TRUE(control_module_manager.Step(SensorReadings{},
                          SensorDerivedQuantities{}, ControlReferences{}).ok());
  }

  ASSERT_RESULT_OK(control_module_manager.GetActiveControlModuleLabel());
  EXPECT_EQ(control_module_manager.GetActiveControlModuleLabel().value(), 1);
  EXPECT_TRUE(control_module_manager.IsSelectedControlModuleReady());

  EXPECT_EQ(fake_2.GetState(), MotorControlModule::State::kStopped);
  EXPECT_EQ(fake_1.GetState(), MotorControlModule::State::kStarted);
}

}  // namespace
}  // namespace barkour
