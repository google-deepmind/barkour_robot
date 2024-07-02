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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gate_driver.h"

#include <cstdint>

#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/common/cycle_counter.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(GateDriverTest, CanUpdate) {
  GateDriver& driver = GateDriver::Get();
  ASSERT_FALSE(driver.Update().error());
  ASSERT_FALSE(driver.Update().error());
}

TEST(GateDriverTest, CurrentStateMatchesLastUpdateCall) {
  GateDriver& driver = GateDriver::Get();
  auto status = driver.Update();
  ASSERT_EQ(driver.CurrentState(), status);
}

TEST(GateDriverTest, SetTargetStateValidatesTargetStates) {
  GateDriver& driver = GateDriver::Get();
  EXPECT_FALSE(driver.SetTargetState(GateDriverState::NotInitialized()).ok());
  EXPECT_FALSE(
      driver.SetTargetState(GateDriverState::WaitingForPowerOn()).ok());
  EXPECT_FALSE(
      driver.SetTargetState(GateDriverState::PowerOnGateEnabledNotConfigured())
          .ok());
  EXPECT_FALSE(driver.SetTargetState(GateDriverState::UnknownError()).ok());
  EXPECT_FALSE(driver.Update().error());
}

TEST(GateDriverTest, CanTurnMotorPowerOnAndOff) {
  // Note: Using EXPECT in this test to make sure it runs all the way till
  // the end and turns off the motor power.
  GateDriver& driver = GateDriver::Get();
  EXPECT_NE(driver.Update(), GateDriverState::PowerOnGateEnabled());
  // Power off, driver should go to PowerOff state.
  EXPECT_TRUE(driver.SetTargetState(GateDriverState::PowerOff()).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(1);
  }
  EXPECT_EQ(driver.CurrentState(), GateDriverState::PowerOff());
  // Power on, driver should go to PowerOnGateEnabled state.
  EXPECT_TRUE(
      driver.SetTargetState(GateDriverState::PowerOnGateEnabled()).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(200);
  }
  EXPECT_EQ(driver.CurrentState(), GateDriverState::PowerOnGateEnabled());
  // Verify that the STO lines are high:
  EXPECT_TRUE(BoardConfig::Get().MotorPowerEnabled());
  // Power off, driver should go to PowerOff state.
  EXPECT_TRUE(driver.SetTargetState(GateDriverState::PowerOff()).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(1);
  }
  EXPECT_EQ(driver.CurrentState(), GateDriverState::PowerOff());
}

TEST(GateDriverTest, CanControlPwm) {
  // Note: Using EXPECT in this test to make sure it runs all the way till
  // the end and turns off the motor power.
  GateDriver& driver = GateDriver::Get();
  // Power on, driver should go to PowerOnGateEnabled state.
  EXPECT_TRUE(
      driver.SetTargetState(GateDriverState::PowerOnGateEnabled()).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(200);
  }
  EXPECT_EQ(driver.CurrentState(), GateDriverState::PowerOnGateEnabled());
  std::array<float, 3> pwm = {
      0.01, 0.01, 0.0};  // A small current that will brake the motor.
  EXPECT_TRUE(driver.SetDutyCycles(pwm[0], pwm[1], pwm[2]).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(1);
  }
  pwm[0] = pwm[1] = pwm[2] = 0.0f;
  EXPECT_TRUE(driver.SetDutyCycles(pwm[0], pwm[1], pwm[2]).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(1);
  }
  // Power off, driver should go to PowerOff state.
  EXPECT_TRUE(driver.SetTargetState(GateDriverState::PowerOff()).ok());
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(driver.Update().error());
    barkour::CycleCounter::Get().BlockForMicroseconds(1);
  }
  EXPECT_EQ(driver.CurrentState(), GateDriverState::PowerOff());
}

TEST(GateDriverTest, PwmChecksPreconditions) {
  GateDriver& driver = GateDriver::Get();
  EXPECT_NE(driver.Update(), GateDriverState::PowerOnGateEnabled());
  std::array<float, 3> pwm = {0.0f, 0.0f, 0.0f};

  EXPECT_EQ(driver.SetDutyCycles(pwm[0], pwm[1], pwm[2]).status(),
            pw::Status::FailedPrecondition());

  EXPECT_TRUE(driver.SetDutyCycles(pwm[0], pwm[1], pwm[2], true).ok());
}

TEST(GateDriver, DutCycleToCompare) {
  GateDriver& driver = GateDriver::Get();

  float duty = 0.0f;
  uint32_t period = 6000;

  uint32_t cmp = driver.DutyCycleToCompareValue(period, duty);

  EXPECT_EQ(cmp, period / 2);

  duty = -1.0f;

  cmp = driver.DutyCycleToCompareValue(period, duty);

  EXPECT_EQ(cmp, 5u);

  duty = 1.0f;

  cmp = driver.DutyCycleToCompareValue(period, duty);

  EXPECT_EQ(cmp, period - 5);

  duty = 0.5f;

  cmp = driver.DutyCycleToCompareValue(period, duty);

  EXPECT_EQ(cmp,
            static_cast<uint32_t>(3.0f / 4.0f * static_cast<float>(period)));

  duty = -0.5f;

  cmp = driver.DutyCycleToCompareValue(period, duty);

  EXPECT_EQ(cmp,
            static_cast<uint32_t>(1.0f / 4.0f * static_cast<float>(period)));
}

}  // namespace
}  // namespace barkour
