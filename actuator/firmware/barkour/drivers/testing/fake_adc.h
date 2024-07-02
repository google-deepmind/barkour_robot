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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ADC_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ADC_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

class FakeAdc : public AdcInterface {
 public:
  pw::Result<int32_t> GetSample(const AdcAnalogSignal&) override {
    PW_TRY(return_status_);
    return sample_return_;
  }

  pw::Result<int32_t> GetInjectedSample(
      const AdcAnalogSignal& signal) override {
    PW_TRY(return_status_);

    if (signal <= AdcAnalogSignal::kPhaseCCurrent) {
      return injected_sample_return_[(uint32_t)signal];
    }
    return 0;
  }

  void SetInjectedSamples(int32_t a, int32_t b, int32_t c) {
    injected_sample_return_[0] = a;
    injected_sample_return_[1] = b;
    injected_sample_return_[2] = c;
  }

  PhaseSampleSelection GetSelectedPhases() override { return phases_; }

  void SetInjectedChannels(PhaseSampleSelection phases) override {
    phases_ = phases;
  }

  uint32_t WaitForInjectedConversion(uint32_t) override { return 0; }

  int32_t sample_return_ = 0;
  int32_t injected_sample_return_[3] = {0, 0, 0};
  pw::Status return_status_;
  PhaseSampleSelection phases_ = PhaseSampleSelection::kABC;
};

};  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_ADC_H_
