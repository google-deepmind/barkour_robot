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

#include "actuator/firmware/barkour/common/motor_control_modules/zero_output_control_module.h"

#include "actuator/firmware/barkour/drivers/testing/fake_gate_driver.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr SensorReadings kSensorReadings = {
    .raw_encoder_count = 1234,
    .encoder_position = 1234,
    .electrical_angle = 0.0,
    .quadrature_current = 0,
    .direct_current = 0,
    .phase_a_current = 0.2,
    .phase_b_current = 0.3,
    .phase_c_current = 0.4,
    .bus_voltage = 36.0,
    .thermistor_1 = 25.0,
    .thermistor_2 = 26.0,
    .thermistor_3 = 27.0,
    .thermistor_4 = 0.0,
    .sto_24v_safe_adc_voltage = 0.0,
    .imu_state = {{0, 0, 0}, {0, 0, 0}}};

constexpr SensorDerivedQuantities kSensorDerivedQuantities = {
    .shaft_position = 6323,
    .shaft_velocity = -347,
};

TEST(ZeroOutputControlModuleTest, CannotStartWithoutGateDriverPoweredOn) {
  FakeGateDriver gate_driver;
  ZeroOutputControlModule control_module(gate_driver);

  EXPECT_EQ(
      control_module.Start(kSensorReadings, kSensorDerivedQuantities).code(),
      pw::Status::Code::PW_STATUS_FAILED_PRECONDITION);
}

TEST(ZeroOutputControlModuleTest, ImmediateLifecycleTransitions) {
  FakeGateDriver gate_driver;
  ZeroOutputControlModule control_module(gate_driver);

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);

  // Start the gate driver.
  gate_driver.SetTargetState(GateDriverState::PowerOnGateEnabled());
  gate_driver.voltage_on_sto_lines_ = true;
  ASSERT_EQ(gate_driver.Update(), GateDriverState::PowerOnGateEnabled());

  // Both stopping and starting should be immediate.
  ASSERT_TRUE(
      control_module.Start(kSensorReadings, kSensorDerivedQuantities).ok());
  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  ASSERT_TRUE(
      control_module.Stop(kSensorReadings, kSensorDerivedQuantities).ok());
  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);
}

TEST(ZeroOutputControlModuleTest, OutputIsZero) {
  FakeGateDriver gate_driver;
  ZeroOutputControlModule control_module(gate_driver);

  // Start the gate driver.
  gate_driver.SetTargetState(GateDriverState::PowerOnGateEnabled());
  gate_driver.voltage_on_sto_lines_ = true;
  ASSERT_EQ(gate_driver.Update(), GateDriverState::PowerOnGateEnabled());

  ASSERT_TRUE(
      control_module.Start(kSensorReadings, kSensorDerivedQuantities).ok());

  // Try running a few times to ensure the control module stays in the "started"
  // state.
  for (int i = 0; i < 10; ++i) {
    pw::Result<RotatingFrameVoltages> control_outputs =
        control_module.Step(kSensorReadings, kSensorDerivedQuantities,
                            ControlReferences{.reference_current = 100.0});
    ASSERT_TRUE(control_outputs.ok());
    EXPECT_EQ(control_outputs->direct_voltage, 0.0);
    EXPECT_EQ(control_outputs->quadrature_voltage, 0.0);
  }
}

TEST(ZeroOutputControlModuleTest, ControlModuleSetsPwmTo0pt5) {
  FakeGateDriver gate_driver;
  ZeroOutputControlModule control_module(gate_driver);

  // Start the gate driver.
  gate_driver.SetTargetState(GateDriverState::PowerOnGateEnabled());
  gate_driver.voltage_on_sto_lines_ = true;
  ASSERT_EQ(gate_driver.Update(), GateDriverState::PowerOnGateEnabled());

  ASSERT_TRUE(
      control_module.Start(kSensorReadings, kSensorDerivedQuantities).ok());

  ASSERT_TRUE(control_module.Step(kSensorReadings, kSensorDerivedQuantities,
                        ControlReferences{.reference_current = 100.0}).ok());

  ASSERT_EQ(gate_driver.duty_cycles_[0], 0.5);
  ASSERT_EQ(gate_driver.duty_cycles_[1], 0.5);
  ASSERT_EQ(gate_driver.duty_cycles_[2], 0.5);
}

}  // namespace
}  // namespace barkour
