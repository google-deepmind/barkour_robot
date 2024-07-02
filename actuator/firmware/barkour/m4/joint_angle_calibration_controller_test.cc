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

#include "actuator/firmware/barkour/m4/joint_angle_calibration_controller.h"

#include <algorithm>
#include <cstdint
#include <cmath>

#include "pw_result/result.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

using TickTypeForTest = uint32_t;

constexpr float kFloatTolerance = 1.0e-5;

TEST(JointAngleCalibrationControllerTest, SuccessfulCalibrationRoutine) {
  const JointAngleCalibrationConstants constants = {
      .p_gain = 5.0f,
      .saturation_voltage = 3.0f,
      .radians_per_tick = 0.1f,
      .controller_error_finished_threshold = 1.0f,
  };

  TickTypeForTest tick_counter = 6543;
  float motor_angle = -0.79f;

  JointAngleCalibrationController<TickTypeForTest> calibration_controller(
      constants, motor_angle, tick_counter);

  // Step the tick counter 5 times, moving the motor position just behind the
  // setpoint.
  float motor_angle_offset = 0.05f;
  pw::Result<float> output = 0f;
  for (uint8_t i = 0; i < 5; ++i) {
    motor_angle += constants.radians_per_tick;
    output = calibration_controller.Update(motor_angle - motor_angle_offset,
                                           ++tick_counter);
    ASSERT_TRUE(output.ok());

    float expected_output = motor_angle_offset * constants.p_gain;
    EXPECT_LE(std::fabs(output.value() - expected_output), kFloatTolerance);
    EXPECT_FALSE(calibration_controller.Finished());
  }

  // Step the tick counter 9 more times, without moving the motor.
  for (uint8_t i = 0; i < 9; ++i) {
    output = calibration_controller.Update(motor_angle - motor_angle_offset,
                                           ++tick_counter);
    ASSERT_TRUE(output.ok());

    float expected_error =
        (i + 1) * constants.radians_per_tick + motor_angle_offset;
    float expected_output =
        std::clamp(expected_error * constants.p_gain,
                   -constants.saturation_voltage, constants.saturation_voltage);

    EXPECT_LE(std::fabs(output.value() - expected_output), kFloatTolerance);
    EXPECT_FALSE(calibration_controller.Finished());
  }

  // The final step (again without moving the motor) should cause the
  // calibration routine to finish and the output to be zero.
  output = calibration_controller.Update(motor_angle - motor_angle_offset,
                                         ++tick_counter);
  ASSERT_TRUE(output.ok());
  EXPECT_EQ(output.value(), 0);
  EXPECT_TRUE(calibration_controller.Finished());
}

TEST(JointAngleCalibrationControllerTest, TickCounterOverflowCausesError) {
  const JointAngleCalibrationConstants constants = {
      .p_gain = 5.0f,
      .saturation_voltage = 3.0f,
      .radians_per_tick = 0.1f,
      .controller_error_finished_threshold = 1.0f,
  };

  JointAngleCalibrationController<TickTypeForTest> calibration_controller(
      constants, 0.0f, 5000);

  // Reduce the tick count.
  EXPECT_TRUE(
      calibration_controller.Update(0.0f, 2000).status().IsInvalidArgument());
}

}  // namespace
}  // namespace barkour
