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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/realtime_foc.h"

#include <cstdint>
#include <cmath>

#include "FreeRTOS.h" // NOLINT
#include "actuator/firmware/barkour/common/adc_conversions.h"
#include "actuator/firmware/barkour/common/derived_sensor_information.h"
#include "actuator/firmware/barkour/common/foc_math.h"
#include "actuator/firmware/barkour/common/interfaces/gpio_debug_interface.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "actuator/firmware/barkour/common/math_constants.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "actuator/firmware/barkour/common/pid_controller.h"
#include "actuator/firmware/barkour/common/trigonometry.h"
#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

// Need a global pointer to FOC class instance so it
// can be accessed from ADC interrupt.
// This pointer is set by the FOC class constructor.
barkour::Foc* foc_instance = nullptr;

namespace barkour {

pw::Status CheckPositive(float arg) {
  if (arg <= 0.0f) {
    return pw::Status::InvalidArgument();
  }
  return pw::OkStatus();
}

pw::Status CheckNonNegative(float arg) {
  if (arg < 0) {
    return pw::Status::InvalidArgument();
  }
  return pw::OkStatus();
}

void Foc::Init(float dt, float p_gain, float i_gain, float anti_wind_gain,
               float saturation_voltage, float phase_resistance,
               float phase_inductance, PhaseOrder phase_order,
               uint32_t num_pole_pairs, int32_t elec_zero_offset) {
  dt_ = dt;

  running_ = false;
  target_q_ = 0.0f;
  feed_forward_ = 0.0f;
  bus_voltage_ = 0.0f;

  num_pole_pairs_ = num_pole_pairs;
  elec_zero_offset_ = elec_zero_offset;

  phase_order_ = phase_order;

  saturation_voltage_margin_ = saturation_voltage;
  phase_resistance_ = phase_resistance;
  phase_inductance_ = phase_inductance;

  phase_select_ = PhaseSampleSelection::kABC;

  last_elec_angle_ = 0.0f;
  last_delta_elec_angle_ = 0.0f;

  pid_q_.SetProportionalGain(p_gain);
  pid_q_.SetIntegralGain(i_gain);
  pid_q_.SetAntiWindGain(anti_wind_gain);
  pid_q_.SetSaturation(-saturation_voltage, saturation_voltage);

  pid_d_.SetProportionalGain(p_gain);
  pid_d_.SetIntegralGain(i_gain);
  pid_d_.SetAntiWindGain(anti_wind_gain);
  pid_d_.SetSaturation(-saturation_voltage, saturation_voltage);
}

Foc::Foc(float dt, float p_gain, float i_gain, float anti_wind_gain,
         float saturation_voltage, float phase_resistance,
         float phase_inductance, RotaryEncoder& encoder,
         GateDriverInterface& gate_driver, PhaseOrder phase_order,
         uint32_t num_pole_pairs, int32_t elec_zero_offset)
    : dt_(dt),
      encoder_(encoder),
      pid_d_(dt),
      pid_q_(dt),
      gate_driver_(gate_driver) {
  Init(dt, p_gain, i_gain, anti_wind_gain, saturation_voltage, phase_resistance,
       phase_inductance, phase_order, num_pole_pairs, elec_zero_offset);
  // Save global pointer for isr to find this instance.
  foc_instance = this;
}

pw::Result<Foc*> Foc::Build(float dt, float p_gain, float i_gain,
                            float anti_wind_gain, float saturation_voltage,
                            float phase_resistance, float phase_inductance,
                            RotaryEncoder& encoder,
                            GateDriverInterface& gate_driver,
                            PhaseOrder phase_order, uint32_t num_pole_pairs,
                            int32_t elec_zero_offset) {
  // Verify.
  PW_TRY(CheckPositive(dt));
  PW_TRY(CheckNonNegative(p_gain));
  PW_TRY(CheckNonNegative(i_gain));
  PW_TRY(CheckNonNegative(anti_wind_gain));
  PW_TRY(CheckPositive(saturation_voltage));
  PW_TRY(CheckPositive(phase_resistance));
  PW_TRY(CheckPositive(phase_inductance));
  PW_TRY(CheckPositive(num_pole_pairs));

  static Foc foc(dt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
                 phase_resistance, phase_inductance, encoder, gate_driver,
                 phase_order, num_pole_pairs, elec_zero_offset);

  return &foc;
}

// Start generating Foc PWM output.
pw::Status Foc::Start(float bus_voltage) {
  portENTER_CRITICAL();

  target_q_ = 0.0f;
  feed_forward_ = 0.0f;
  bus_voltage_ = bus_voltage;

  last_elec_angle_ = 0.0f;
  last_delta_elec_angle_ = 0.0f;

  float saturation_voltage = (bus_voltage / 2.0f) - saturation_voltage_margin_;

  if (saturation_voltage <= 0.0f) {
    PW_LOG_ERROR(
        "Invalid Foc saturation voltage : %f V. Aborting "
        "controller start",
        saturation_voltage);

    portEXIT_CRITICAL();

    return pw::Status::FailedPrecondition();
  }

  PW_LOG_INFO("Set Foc saturation voltage to: %f V.", saturation_voltage);

  pid_d_.SetSaturation(-saturation_voltage, saturation_voltage);
  pid_q_.SetSaturation(-saturation_voltage, saturation_voltage);

  pid_d_.Reset(0.0f);
  pid_q_.Reset(0.0f);

  running_ = true;

  portEXIT_CRITICAL();

  return pw::OkStatus();
}

// Stop updating FOC state, no PWM output when stopped
pw::Status Foc::Stop(void) {
  portENTER_CRITICAL();
  target_q_ = 0.0f;
  feed_forward_ = 0.0f;
  running_ = false;
  portEXIT_CRITICAL();
  return pw::OkStatus();
}

// Set target Q current, called from controller thread vi
// state machine.
// - iq: target current (Amps)
// - bus_voltage: measured bus_voltage (Volts)
pw::Status Foc::SetTargetQ(float iq, float bus_voltage) {
  if (!running_) {
    return pw::OkStatus();
  }
  portENTER_CRITICAL();
  target_q_ = iq;
  feed_forward_ = 0.0f;  // iq * phase_resistance_;
  bus_voltage_ = bus_voltage;
  portEXIT_CRITICAL();

  return pw::OkStatus();
}

Foc::FocState Foc::GetState(void) {
  FocState state;

  portENTER_CRITICAL();
  if ((!running_) ||
      (gate_driver_.CurrentState() != GateDriverState::PowerOnGateEnabled())) {
    state = FocState();
  } else {
    state = state_;
  }
  portEXIT_CRITICAL();

  return state;
}

// Set PIDs to open loop mode.
void Foc::SetOpenLoop(bool enable) {
  pid_d_.SetOpenLoop(enable);
  pid_q_.SetOpenLoop(enable);
}

// Run all the hard real time stuff here (RUNS under ADC Injected Interrupt @
// 10Khz).
// - adc_a, adc_b, adc_c:  measured phase currents in raw adc scale.
//
//  Returns code to indicate which 2 phase currents should be sampled on the
//  next cycle.
PhaseSampleSelection Foc::adc_isr(int32_t adc_a, int32_t adc_b, int32_t adc_c) {
  // Get latest encoder reading. (This will block to read the encoder).
  pw::Result<RotaryEncoderState> result = encoder_.Update();

  if ((!running_) ||
      (gate_driver_.CurrentState() != GateDriverState::PowerOnGateEnabled())) {
    state_ = FocState();

    return PhaseSampleSelection::kABC;
  }

#ifdef DEBUG
  DBG_GPIO(true);
#endif  // DEBUG

  // Convert raw adc readings to Amps.
  float ia = AdcReadingToFloat(AdcAnalogSignal::kPhaseACurrent, adc_a).value();
  float ib = AdcReadingToFloat(AdcAnalogSignal::kPhaseBCurrent, adc_b).value();
  float ic = AdcReadingToFloat(AdcAnalogSignal::kPhaseCCurrent, adc_c).value();

  uint32_t raw_encoder_counts = 0;
  bool missed_value = false;

  if (result.ok()) {
    raw_encoder_counts = result.value().raw_encoder_counts;
  } else {
    raw_encoder_counts = last_raw_encoder_counts_.value_or(0.0f);

    missed_value = true;
  }

  last_raw_encoder_counts_ = raw_encoder_counts;

  // elec_angle 0 <-> 2 pi.
  float elec_angle = EncoderCountToElectricalAngle(raw_encoder_counts,
                                                   encoder_.GetCountsPerTurn(),
                                                   elec_zero_offset_,
                                                   num_pole_pairs_);
  if (missed_value) {  // Missed encoder update, extrapolate from last velocity.
    elec_angle += last_delta_elec_angle_;
  }

  // Calculate electrical angle change since last update.
  float delta_elec_angle = elec_angle - last_elec_angle_;

  // Bound delta to +- pi.
  if (delta_elec_angle > kPi) {
    delta_elec_angle -= kTwoPi;
  }
  if (delta_elec_angle < -kPi) {
    delta_elec_angle += kTwoPi;
  }

  last_elec_angle_ = elec_angle;  // Update saved angle for next update.
  last_delta_elec_angle_ = delta_elec_angle;  // Save last delta.

// #define FORCE_DEBUG_FIXED_VELOCITY
#ifdef FORCE_DEBUG_FIXED_VELOCITY
  // For debugging this simulates d/q and angle to force a fixed velocity.

  float acc = 10.0f;                // rpm/sec^2
  constexpr float max_rpm = 60.0f;  // output shaft speed (rpm)
  static float rpm = 0.0f;

  constexpr float ke = 0.009524f;

  float svq = 0;
  float svd = 0.0f;
  constexpr float sdea = 0.0f;

  static float phase = 0.0f;
  float selec_angle = kTwoPi * phase;

  float elect_hz = rpm * 9.0f * 21.0f / 60.0f;
  phase += elect_hz / kFocLoopRate;
  if (phase > 1.0f) {
    phase -= 1.0f;
  }
  if (rpm < max_rpm && svq < (bus_voltage_ / 2.0f)) {
    rpm += acc * dt_;
  }

  float svdq = (rpm * 7 * ke) * sqrtf(2.0f);

  svq = svdq * cos(0.8);

  svd = svdq * sin(0.8f);

#endif  // FORCE_DEBUG_FIXED_VELOCITY

  // Run FOC calculations.
  FocAlphaBeta ab = ClarkeTransform(ia, ib, ic);
  FocDQ dq = ParkTransform(ab.alpha, ab.beta, elec_angle);

  // https://www.ti.com/lit/ug/spruhj1i/spruhj1i.pdf?ts=1662572119483&ref_url=https%253A%252F%252Fwww.google.com%252F
  // constexpr float l = (57.0e-6)/2.0f;
  // float w =delta_enc_ang/kDT; // electrical velocity (rad/sec)
  // float wl = l * w;
  float feed_forward_q = 0.0f;  // wl*dq.first+ w * 0.091f;
  float feed_forward_d = 0.0f;  // wl*dq.second;

  pid_d_.SetTrajectoryReference(0.0f, 0.0f, -feed_forward_d);
  float vd = pid_d_.ComputeOutput(dq.d, 0.0f);

  pid_q_.SetTrajectoryReference(target_q_, 0.0f, feed_forward_q);
  float vq = pid_q_.ComputeOutput(dq.q, 0.0f);

#ifdef FORCE_DEBUG_FIXED_VELOCITY
  // force the commutation to a fixed velocity based on simulated d/q/and angle
  ThreePhasePwmCommands phase_duty_cycles = commutator_.DoCommutation(
      svq, svd,
      selec_angle + sdea,  // use extrapolated angle for next update cycle
      bus_voltage_);

#else   // FORCE_DEBUG_FIXED_VELOCITY

  // add `w * kDT/2.0f` to extrapolate 1/2 pwm cycle forward
  float next_elec_angle = elec_angle;

  // Calculate Space Vector voltages.
  // outputs are scaled and shifted to 0-1
  // 0 = max negative voltage,
  // 1 = max positive voltage
  // 0.5 = mid voltage (ac 0V)
  // Returns the duty cycle for each phase that will generate the voltage.
  ThreePhasePwmCommands phase_duty_cycles = commutator_.DoCommutation(
      vq, vd,
      next_elec_angle,  // Use extrapolated angle for next update cycle.
      bus_voltage_);
#endif  // else FORCE_DEBUG_FIXED_VELOCITY

  pw::Result<PhaseSampleSelection> maybe_phase_select;

  // Swap voltages to correct phase if phase swap is required
  switch (phase_order_) {
    case PhaseOrder::kAbc: {
      maybe_phase_select =
          gate_driver_.SetDutyCycles(phase_duty_cycles.phase_a,
                                     phase_duty_cycles.phase_b,
                                     phase_duty_cycles.phase_c);
      break;
    }
    case PhaseOrder::kAcb: {
      maybe_phase_select =
          gate_driver_.SetDutyCycles(phase_duty_cycles.phase_a,
                                     phase_duty_cycles.phase_c,
                                     phase_duty_cycles.phase_b);
      break;
    }
    default: {
      // Shouldn't happen, explicitly crash or might get weirdness.
      PW_CRASH("Invalid CommutationPhaseOrder enum.");
    }
  }

  if (maybe_phase_select.ok()) {
    phase_select_ = maybe_phase_select.value();
  } else {
    PW_CRASH("SetDutyCycle Failed.");
  }

#ifdef DEBUG  // Useful macros calls for debugging.
  // float ead = elec_angle + dea;
  // ead = atan2f(sin(ead),cos(ead));
  // if(ead < 0.0f) {
  //   ead+=kTwoPi;
  // }

  // DBG_DAC1(selec_angle / (kPi)-1.0f);
  DBG_DAC1(elec_angle / (kPi)-1.0f);
  //  DBG_DAC2(ead / (kPi)-1.0f);

  // DBG_DAC1(ia/54.7f);
  // DBG_DAC2(ib/54.7f);
  // DBG_DAC1(ic/54.7f);

  // DBG_DAC1(target_q_/54.7f);

  // DBG_DAC1(dq.q / 54.7f);
  // DBG_DAC2(dq.d / 54.7f);

  // DBG_DAC2(ab.beta/54.7f);
  //  DBG_DAC2(ab.b/54.7f);

  DBG_DAC2(vq / (bus_voltage_ / 2.0f));
  // DBG_DAC2(vd/(bus_voltage_));

  // DBG_DAC2(phase_duty_cycles.phase_a*2.0f-1.0f);
  //  DBG_DAC1(phase_duty_cycles.phase_b*2.0f-1.0f);
  //  DBG_DAC1(phase_duty_cycles.phase_c*2.0f-1.0f);

  // DBG_DAC2(static_cast<double>(raw_encoder_counts)/encoder_.GetCountsPerTurn());

  DBG_GPIO(false);

#endif  // DEBUG

  // Update state values.
  state_.phase_a_current_ = ia;
  state_.phase_b_current_ = ib;
  state_.phase_c_current_ = ic;

  state_.phase_a_duty_ = phase_duty_cycles.phase_a;
  state_.phase_b_duty_ = phase_duty_cycles.phase_b;
  state_.phase_c_duty_ = phase_duty_cycles.phase_c;

  state_.i_d_ = dq.d;
  state_.i_q_ = dq.q;

  state_.v_d_ = vd;
  state_.v_q_ = vq;

  state_.elec_angle_ = elec_angle;

  state_.pss_ = phase_select_;

  return phase_select_;
}

// ADC Injected interrupt handler calls this.
PhaseOrder foc_get_phase_order(void) {
  if (foc_instance == nullptr) {
    return PhaseOrder::kAbc;
  }

  // Handle processing in class instance.
  return foc_instance->GetPhaseOrder();
}

// ADC Injected interrupt handler calls this.
PhaseSampleSelection foc_adc_isr(int32_t adc_a, int32_t adc_b, int32_t adc_c) {
  if (foc_instance == nullptr) {
    return PhaseSampleSelection::kABC;
  }

  // Handle processing in class instance.
  return foc_instance->adc_isr(adc_a, adc_b, adc_c);
}

}  // namespace barkour
