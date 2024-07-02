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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gpio_debug.h"

#include "pw_log/log.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

// Check that Gpio Debugging is enabled.
TEST(GpioDebug, IsEnabled) {
  GpioDebug gpio_debug = GpioDebug::Get();
  if (!gpio_debug.IsGpioDebugEnabled()) {
    PW_LOG_WARN("GPIO Debug Class disabled, test will be ignored");
  }
}

// Test Init Hardware Call returns ok.
TEST(GpioDebug, InitHardwareOK) {
  GpioDebug gpio_debug = GpioDebug::Get();

  if (!gpio_debug.IsGpioDebugEnabled()) {
    return;
  }

  auto result = gpio_debug.InitHardware();
  EXPECT_EQ(result, pw::Status());
}

// Test SetPWMDACChannel returns ok, channel valid.
TEST(GpioDebug, SetPWMDACOutputOK) {
  GpioDebug gpio_debug = GpioDebug::Get();
  if (!gpio_debug.IsGpioDebugEnabled()) {
    return;
  }
  auto result =
      gpio_debug.SetPWMDACOutput(PWMDACChannel::kPwmDacChannel1, 0.0f);
  EXPECT_EQ(result, pw::Status());

  result = gpio_debug.SetPWMDACOutput(PWMDACChannel::kPwmDacChannel2, 0.0f);
  EXPECT_EQ(result, pw::Status());
}

// Test SetPWMDACChannel returns ok, channel invalid.
TEST(GpioDebug, SetPWMDACOutputInvalidChannel) {
  GpioDebug gpio_debug = GpioDebug::Get();

  if (!gpio_debug.IsGpioDebugEnabled()) {
    return;
  }

  auto result = gpio_debug.SetPWMDACOutput((PWMDACChannel)-1, 0.0f);
  EXPECT_EQ(result, pw::Status::InvalidArgument());
}

// Test SetGpioChannel returns ok.
TEST(GpioDebug, SetDigitalOutputOk) {
  GpioDebug gpio_debug = GpioDebug::Get();

  if (!gpio_debug.IsGpioDebugEnabled()) {
    return;
  }

  auto result = gpio_debug.SetDigitalOutput(GPIOChannel::kGpioChannel1, true);
  EXPECT_EQ(result, pw::Status());
}
}  // namespace
}  // namespace barkour
