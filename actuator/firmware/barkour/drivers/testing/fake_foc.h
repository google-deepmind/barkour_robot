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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_FOC_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_FOC_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/phase_order.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_status/status.h"

namespace barkour {

class FakeFoc : public FocInterface {
 public:
  explicit FakeFoc(float p_gain = 1.0f)
      : ia_(0.0f),
        ib_(0.0f),
        ic_(0.0f),

        iq_(0.0f),
        id_(0.0f),

        elec_ang_(0.0f),

        ref_i_(0.0f),

        p_gain_(p_gain),

        bv_(0.0f) {}

  void SetState(float iq, float id) {
    iq_ = iq;
    id_ = id;
  }

  void SetState(float ia, float ib, float ic) {
    ia_ = ia;
    ib_ = ib;
    ic_ = ic;
  }

  void SetState(float elec_ang) { elec_ang_ = elec_ang; }

  FocState GetState() override {
    foc_state_.phase_a_current_ = ia_;
    foc_state_.phase_b_current_ = ib_;
    foc_state_.phase_c_current_ = ic_;

    foc_state_.phase_a_duty_ = 0.0f;
    foc_state_.phase_b_duty_ = 0.0f;
    foc_state_.phase_c_duty_ = 0.0f;

    foc_state_.i_q_ = iq_;
    foc_state_.i_d_ = id_;

    foc_state_.v_q_ = p_gain_ * (ref_i_ - iq_);
    foc_state_.v_d_ = p_gain_ * -id_;

    foc_state_.elec_angle_ = elec_ang_;

    return foc_state_;
  }

  PhaseSampleSelection GetPhaseSampleSelection() override {
    return PhaseSampleSelection::kABC;
  }

  PhaseOrder GetPhaseOrder() override { return PhaseOrder::kAbc; }

  pw::Status SetTargetQ(float ref_i, float bv) override {
    ref_i_ = ref_i;
    bv_ = bv;
    return pw::OkStatus();
  }

  pw::Status Start(float) override { return pw::OkStatus(); }
  pw::Status Stop() override { return pw::OkStatus(); }

  PhaseSampleSelection adc_isr(int32_t, int32_t, int32_t) override {
    return PhaseSampleSelection::kABC;
  }

 private:
  FocInterface::FocState foc_state_;

  float ia_;
  float ib_;
  float ic_;

  float iq_;
  float id_;

  float elec_ang_;

  float ref_i_;

  float p_gain_;

  float bv_;
};

};  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_TESTING_FAKE_FOC_H_
