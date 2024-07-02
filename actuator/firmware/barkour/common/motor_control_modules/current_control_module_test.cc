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

#include <cmath>

#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/drivers/testing/fake_adc.h"
#include "actuator/firmware/barkour/drivers/testing/fake_foc.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr float kNumericalTolerance = 1.0e-5;

TEST(CurrentControlModuleTest, FactoryMethodParametersChecked) {
  FakeFoc foc;

  // Negative current limit.
  EXPECT_EQ(CurrentControlModule::Build(-1.0, foc).status(),
            pw::Status::InvalidArgument());
}

TEST(CurrentControlModuleTest, SimpleRunTest) {
  FakeFoc fake_foc;

  pw::Result<CurrentControlModule> maybe_control_module =
      CurrentControlModule::Build(10.0, fake_foc);

  ASSERT_TRUE(maybe_control_module.ok());

  CurrentControlModule& control_module = maybe_control_module.value();

  SensorReadings sensor_readings = {.raw_encoder_count = 1234,
                                    .encoder_position = 1234,
                                    .electrical_angle = 3.92,
                                    .quadrature_current = 0,
                                    .direct_current = 0,
                                    .phase_a_current = 0.2,
                                    .phase_b_current = 0.3,
                                    .phase_c_current = -0.5,
                                    .bus_voltage = 36.0,
                                    .thermistor_1 = 25.0,
                                    .thermistor_2 = 26.0,
                                    .thermistor_3 = 27.0,
                                    .thermistor_4 = 0.0,
                                    .sto_24v_safe_adc_voltage = 0.0,
                                    .imu_state = {{0, 0, 0}, {0, 0, 0}}};

  SensorDerivedQuantities sensor_derived_quantities = {
      .shaft_position = 6323,
      .shaft_velocity = -347,
  };

  fake_foc.SetState(sensor_readings.quadrature_current,
                    sensor_readings.direct_current);

  ASSERT_TRUE(
      control_module.Start(sensor_readings, sensor_derived_quantities).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  pw::Result<RotatingFrameVoltages> result =
      control_module.Step(sensor_readings, sensor_derived_quantities,
                          ControlReferences{.reference_current = 5.0f});

  ASSERT_TRUE(result.ok());

  ASSERT_TRUE(
      control_module.Stop(sensor_readings, sensor_derived_quantities).ok());

  EXPECT_EQ(control_module.GetState(), MotorControlModule::State::kStopped);
}

TEST(CurrentControlModuleTest, CheckOutputValues) {
  // Set some parameters. Just use nonzero P gain for simplicity.
  float p_gain = 2.5;
  float current_limit = 15.0;

  // Float saturation_voltage_margin = 1.0;
  FakeAdc fake_adc;
  FakeFoc fake_foc(p_gain);

  pw::Result<CurrentControlModule> maybe_control_module =
      CurrentControlModule::Build(current_limit, fake_foc);

  ASSERT_TRUE(maybe_control_module.ok());

  CurrentControlModule& control_module = maybe_control_module.value();

  SensorReadings sensor_readings = {.raw_encoder_count = 1234,
                                    .encoder_position = 1234,
                                    .electrical_angle = 3.92,
                                    .quadrature_current = 0.3,
                                    .direct_current = 0.2,
                                    .phase_a_current = 0,
                                    .phase_b_current = 0,
                                    .phase_c_current = 0,
                                    .bus_voltage = 36.0,
                                    .thermistor_1 = 25.0,
                                    .thermistor_2 = 26.0,
                                    .thermistor_3 = 27.0,
                                    .thermistor_4 = 0.0,
                                    .sto_24v_safe_adc_voltage = 0.0,
                                    .imu_state = {{0, 0, 0}, {0, 0, 0}}};

  SensorDerivedQuantities sensor_derived_quantities = {
      .shaft_position = 6323,
      .shaft_velocity = -347,
  };

  fake_foc.SetState(sensor_readings.quadrature_current,
                    sensor_readings.direct_current);

  ASSERT_TRUE(
      control_module.Start(sensor_readings, sensor_derived_quantities).ok());
  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  // Set a small current reference, which won't clip.
  float reference_current = 0.1;
  pw::Result<RotatingFrameVoltages> result = control_module.Step(
      sensor_readings, sensor_derived_quantities,
      ControlReferences{.reference_current = reference_current});

  ASSERT_TRUE(result.ok());

  float expected_quadrature_voltage =
      p_gain * (reference_current - sensor_readings.quadrature_current);

  float expected_direct_voltage = p_gain * (-sensor_readings.direct_current);

  EXPECT_LT(std::fabs(result->quadrature_voltage - expected_quadrature_voltage),
            kNumericalTolerance);
  EXPECT_LT(std::fabs(result->direct_voltage - expected_direct_voltage),
            kNumericalTolerance);
}

TEST(CurrentControlModuleTest, CheckOutputValuesCurrentLimitClipping) {
  // Set some parameters. Just use nonzero P gain for simplicity.
  float p_gain = 2.5;
  float current_limit = 15.0;

  FakeFoc fake_foc(p_gain);

  pw::Result<CurrentControlModule> maybe_control_module =
      CurrentControlModule::Build(current_limit, fake_foc);

  ASSERT_TRUE(maybe_control_module.ok());

  CurrentControlModule& control_module = maybe_control_module.value();

  SensorReadings sensor_readings = {
      .raw_encoder_count = 1234,
      .encoder_position = 1234,
      .electrical_angle = 3.92,
      .quadrature_current = 0.3,
      .direct_current = 0.2,

      .phase_a_current = 0,
      .phase_b_current = 0,
      .phase_c_current = 0,
      // Set a high bus voltage to ensure the output saturation doesn't also
      // cause clipping.
      .bus_voltage = 1000.0,
      .thermistor_1 = 25.0,
      .thermistor_2 = 26.0,
      .thermistor_3 = 27.0,
      .thermistor_4 = 0.0,
      .sto_24v_safe_adc_voltage = 0.0,
      .imu_state = {{0, 0, 0}, {0, 0, 0}}};

  SensorDerivedQuantities sensor_derived_quantities = {
      .shaft_position = 6323,
      .shaft_velocity = -347,
  };

  fake_foc.SetState(sensor_readings.quadrature_current,
                    sensor_readings.direct_current);

  ASSERT_TRUE(
      control_module.Start(sensor_readings, sensor_derived_quantities).ok());
  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  // Set a large reference current, which should be clipped to the limit.
  float reference_current = -18.0;

  pw::Result<RotatingFrameVoltages> result = control_module.Step(
      sensor_readings, sensor_derived_quantities,
      ControlReferences{.reference_current = reference_current});

  ASSERT_TRUE(result.ok());
}

TEST(CurrentControlModuleTest, CheckOutputValuesSaturationLimitClipping) {
  // Set a high current limit to ensure the output saturation doesn't also
  // cause clipping.
  float current_limit = 100.0;

  FakeAdc fake_adc;
  FakeFoc fake_foc;

  pw::Result<CurrentControlModule> maybe_control_module =
      CurrentControlModule::Build(current_limit, fake_foc);

  ASSERT_TRUE(maybe_control_module.ok());

  CurrentControlModule& control_module = maybe_control_module.value();

  SensorReadings sensor_readings = {
      .raw_encoder_count = 1234,
      .encoder_position = 1234,
      .electrical_angle = 3.92,
      .quadrature_current = 0.3,
      // Set a high direct current to check it is also clipped.
      .direct_current = 200.0,
      // Shouldn't be used, only the D / Q currents in the derived sensor info
      // should matter.
      .phase_a_current = 0,
      .phase_b_current = 0,
      .phase_c_current = 0,
      .bus_voltage = 12.0,
      .thermistor_1 = 25.0,
      .thermistor_2 = 26.0,
      .thermistor_3 = 27.0,
      .thermistor_4 = 0.0,
      .sto_24v_safe_adc_voltage = 0.0,
      .imu_state = {{0, 0, 0}, {0, 0, 0}}};

  SensorDerivedQuantities sensor_derived_quantities = {
      .shaft_position = 6323,
      .shaft_velocity = -347,
  };

  ASSERT_TRUE(
      control_module.Start(sensor_readings, sensor_derived_quantities).ok());
  ASSERT_EQ(control_module.GetState(), MotorControlModule::State::kStarted);

  // Set a large reference current, which should be clipped to the limit.
  float reference_current = 18.0;

  pw::Result<RotatingFrameVoltages> result = control_module.Step(
      sensor_readings, sensor_derived_quantities,
      ControlReferences{.reference_current = reference_current});

  ASSERT_TRUE(result.ok());
}

}  // namespace
}  // namespace barkour
