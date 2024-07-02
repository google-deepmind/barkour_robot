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

#include "actuator/firmware/barkour/common/commutation.h"

#include <cmath>

#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

constexpr float kNumericalTolerance = 1.0e-4;

TEST(SinusoidalCommutationTest, OutputValues) {
  float u_q = 1;
  float u_d = 0;
  float bus_voltage = 24.0;
  float electrical_angle = 0.622;

  SinusoidalCommutation commutation;

  ThreePhasePwmCommands output_pwm =
      commutation.DoCommutation(u_q, u_d, electrical_angle, bus_voltage);

  // pre-calculated expected result
  float expected_phase_a_pwm = 0.5359;
  float expected_phase_b_pwm = 0.5061;
  float expected_phase_c_pwm = 0.4641;

  EXPECT_LT(std::fabs(output_pwm.phase_a - expected_phase_a_pwm),
            kNumericalTolerance);
  EXPECT_LT(std::fabs(output_pwm.phase_b - expected_phase_b_pwm),
            kNumericalTolerance);
  EXPECT_LT(std::fabs(output_pwm.phase_c - expected_phase_c_pwm),
            kNumericalTolerance);
}

}  // namespace
}  // namespace barkour
