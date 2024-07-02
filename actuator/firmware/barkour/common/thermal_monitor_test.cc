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

#include "actuator/firmware/barkour/common/thermal_monitor.h"

#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(ThermalMonitorTest, BuildParametersAreChecked) {
  // counter_threshold == 0
  EXPECT_EQ(ThermalMonitor<3>::Build(50.0, 0, 10).status(),
            pw::Status::InvalidArgument());

  // counter_saturation == 0
  EXPECT_EQ(ThermalMonitor<3>::Build(50.0, 10, 0).status(),
            pw::Status::InvalidArgument());

  // counter_saturation < counter_threshold
  EXPECT_EQ(ThermalMonitor<3>::Build(50.0, 10, 5).status(),
            pw::Status::InvalidArgument());
}

TEST(ThermalMonitorTest, OverheatConditionWorksAsExpected) {
  float temperature_threshold = 50.0;
  int counter_threshold = 10;
  int counter_saturation = 20;

  pw::Result<ThermalMonitor<4>> maybe_thermal_monitor =
      ThermalMonitor<4>::Build(
          temperature_threshold, counter_threshold, counter_saturation);
  ASSERT_TRUE(maybe_thermal_monitor.ok());

  ThermalMonitor<4>& thermal_monitor = *maybe_thermal_monitor;

  // Arbitrarily, consider temperature at index 2.

  // First set two readings above the threshold. After this, counter should be
  // at 2.
  thermal_monitor.Update({20, 10, temperature_threshold + 0.1f, 30});
  thermal_monitor.Update({20, 10, temperature_threshold + 10, 30});

  EXPECT_FALSE(thermal_monitor.Overheated());

  // Then set one below the threshold. After this, counter should be at 1.
  thermal_monitor.Update({20, 10, temperature_threshold - 0.1f, 30});

  EXPECT_FALSE(thermal_monitor.Overheated());

  // Then set 8 above the threshold. After this, counter should be at 9.
  for (int i = 0; i < 8; ++i) {
    thermal_monitor.Update({20, 10, temperature_threshold + 30, 30});
  }

  EXPECT_FALSE(thermal_monitor.Overheated());

  // Then set one more above threshold, counter should be at 10 => overheated.
  thermal_monitor.Update({20, 10, temperature_threshold + 0.5f, 30});
  EXPECT_TRUE(thermal_monitor.Overheated());
}

TEST(ThermalMonitorTest, SaturationWorksAsExpected) {
  float temperature_threshold = 50.0;
  int counter_threshold = 10;
  int counter_saturation = 20;

  pw::Result<ThermalMonitor<4>> maybe_thermal_monitor =
      ThermalMonitor<4>::Build(
          temperature_threshold, counter_threshold, counter_saturation);
  ASSERT_TRUE(maybe_thermal_monitor.ok());

  ThermalMonitor<4>& thermal_monitor = *maybe_thermal_monitor;

  // Wind up the counter to the saturation limit.
  for (int i = 0; i < 200; ++i) {
    thermal_monitor.Update({20, 10, temperature_threshold + 10, 30});
  }

  ASSERT_TRUE(thermal_monitor.Overheated());

  // Wind it back down so the counter == threshold.
  for (int i = 0; i < (counter_saturation - counter_threshold); ++i) {
    thermal_monitor.Update({20, 10, temperature_threshold - 10, 30});
  }

  ASSERT_TRUE(thermal_monitor.Overheated());

  // Set one more reading below the temp threshold, now counter should be one
  // less than the counter threshold.
  thermal_monitor.Update({20, 10, temperature_threshold - 1, 30});

  ASSERT_FALSE(thermal_monitor.Overheated());
}

}  // namespace

}  // namespace barkour
