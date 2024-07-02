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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_REALTIME_FOC_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_REALTIME_FOC_INTERFACE_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/phase_order.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_status/status.h"

// Interface to FOC class.
// FOC runs FOC algorithm in ADC interrupt.

namespace barkour {

// Global wrapper function for interrupt to call Foc class method to run isr
// processing.
extern PhaseSampleSelection foc_adc_isr(int32_t adc_a, int32_t adc_b,
                                        int32_t adc_c);

// Global wrapper function for interrupt to call Foc class method to get the
// configured phase_ordering.
extern PhaseOrder foc_get_phase_order();

// FocInterface provides interface to Foc class that hides hardware dependencies
// and allows fake implementation for testing.
class FocInterface {
 public:
  // Data values calculated by foc_adc_isr and exposed to higher level system.
  class FocState {
   public:
    float phase_a_current_;
    float phase_b_current_;
    float phase_c_current_;

    float phase_a_duty_;
    float phase_b_duty_;
    float phase_c_duty_;

    float i_d_;
    float i_q_;

    float v_d_;
    float v_q_;

    float elec_angle_;

    PhaseSampleSelection pss_;

    FocState(float phase_a_current, float _phase_b_current,
             float _phase_c_current,

             float _phase_a_duty, float _phase_b_duty, float _phase_c_duty,

             float _i_d, float _i_q,

             float _v_d, float _v_q,

             float _elec_angle, PhaseSampleSelection _pss) :
               phase_a_current_(phase_a_current),
               phase_b_current_(_phase_b_current),
               phase_c_current_(_phase_c_current),

               phase_a_duty_(_phase_a_duty),
               phase_b_duty_(_phase_b_duty),
               phase_c_duty_(_phase_c_duty),

               i_d_(_i_d), i_q_(_i_q),

               v_d_(_v_d), v_q_(_v_q),

               elec_angle_(_elec_angle), pss_(_pss) {}

    FocState() {
      phase_a_current_ = 0.0f;
      phase_b_current_ = 0.0f;
      phase_c_current_ = 0.0f;

      phase_a_duty_ = 0.5f;
      phase_b_duty_ = 0.5f;
      phase_c_duty_ = 0.5f;

      i_d_ = 0.0f;
      i_q_ = 0.0f;

      v_d_ = 0.0f;
      v_q_ = 0.0f;

      elec_angle_ = 0.0f;

      pss_ = PhaseSampleSelection::kABC;
    }
  };

  virtual ~FocInterface() = default;

  virtual FocState GetState() = 0;
  virtual PhaseOrder GetPhaseOrder() = 0;

  virtual PhaseSampleSelection GetPhaseSampleSelection() = 0;

  virtual pw::Status Start(float bus_voltage) = 0;
  virtual pw::Status Stop() = 0;

  virtual pw::Status SetTargetQ(float iq, float bus_voltage) = 0;

  virtual PhaseSampleSelection adc_isr(int32_t adc_a, int32_t adc_b,
                                       int32_t adc_c) = 0;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_REALTIME_FOC_INTERFACE_H_
