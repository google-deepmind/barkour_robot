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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/adc.h"

#include <cstdint>

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gate_driver.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

int enable_gate_driver(uint32_t timeout_ms) {
  uint32_t tick = HAL_GetTick();
  uint32_t tick_end = tick + timeout_ms;
  GateDriver& gate_driver = GateDriver::Get();
  gate_driver.SetTargetState(barkour::GateDriverState::PowerOnGateEnabled());
  while (tick < tick_end) {
    if (gate_driver.CurrentState() == GateDriverState::PowerOnGateEnabled()) {
      return tick_end - tick;
    }
    gate_driver.Update();
    tick = HAL_GetTick();
  }
  return -1;
}

int disable_gate_driver(uint32_t timeout_ms) {
  uint32_t tick = HAL_GetTick();
  uint32_t tick_end = tick + timeout_ms;
  GateDriver& gate_driver = GateDriver::Get();
  gate_driver.SetTargetState(barkour::GateDriverState::PowerOff());
  while (tick < tick_end) {
    if (gate_driver.CurrentState() == GateDriverState::PowerOff()) {
      return tick_end - tick;
    }
    gate_driver.Update();
    tick = HAL_GetTick();
  }
  return -1;
}

TEST(Adc, GetSampleSucceeds) {
  Adc adc = Adc::Get();
  auto get_sample_result = adc.GetSample(AdcAnalogSignal::kMotorTherm1);
  EXPECT_EQ(get_sample_result.status(), pw::OkStatus());
}

TEST(Adc, InjectedModeSamplesOccur) {
  Adc& adc = Adc::Get();

  // Wait for the gate driver to come up.
  // The gate drive needs to be enabled for the Injected mode sampling
  // functionality of the ADC to work.
  auto time = enable_gate_driver(1000);
  ASSERT_GT(time, 0);

  // First call will ensure that the counter is clear.
  adc.WaitForInjectedConversion();

  uint32_t notification_value = adc.WaitForInjectedConversion(100);
  notification_value = adc.WaitForInjectedConversion(100);
  EXPECT_GT(notification_value, 0u);

  // Turn the gate driver off now that it is no longer needed.
  time = disable_gate_driver(1000);
  ASSERT_GT(time, 0);
}

TEST(Adc, AdcSampleValuesAreSane) {
  Adc& adc = Adc::Get();

  // Wait for the gate driver to come up.
  // The gate drive needs to be enabled for the Injected mode sampling
  // functionality of the ADC to work.
  auto time = enable_gate_driver(1000);
  ASSERT_GT(time, 0);

  // We expect that the values are floating about mid-range.
  adc.WaitForInjectedConversion();
  constexpr int16_t kLowerAdcBound = -1000;
  constexpr int16_t kUpperAdcBound = +1000;
  EXPECT_GT(adc.motor_adc_samples_injected[0], kLowerAdcBound);
  EXPECT_LE(adc.motor_adc_samples_injected[0], kUpperAdcBound);
  EXPECT_GT(adc.motor_adc_samples_injected[1], kLowerAdcBound);
  EXPECT_LE(adc.motor_adc_samples_injected[1], kUpperAdcBound);
  EXPECT_GT(adc.motor_adc_samples_injected[2], kLowerAdcBound);
  EXPECT_LE(adc.motor_adc_samples_injected[2], kUpperAdcBound);

  // Set the Gate driver back to its initial state.
  time = disable_gate_driver(1000);
  ASSERT_GT(time, 0);
}

}  // namespace
}  // namespace barkour
