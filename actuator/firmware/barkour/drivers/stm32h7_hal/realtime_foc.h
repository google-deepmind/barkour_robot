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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_REALTIME_FOC_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_REALTIME_FOC_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "actuator/firmware/barkour/common/commutation.h"
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "actuator/firmware/barkour/common/pid_controller.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "actuator/firmware/barkour/common/interfaces/realtime_foc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"

namespace barkour {

class Foc : public FocInterface {
 public:
  // Rate the FOC loop is run (10KHz).
  static inline constexpr float kFocLoopRate = 10000.0f;  // ADC Interrupt Rate.

  // Time period the FOC loop.
  static inline constexpr float kFocDt = 1.0f / kFocLoopRate;

  // BUILD: Create singleton instance of Foc class.
  //  Args:
  // - dt: Control step time, in seconds. Must be > 0.
  // - p_gain: P gain of the PI current controller, in Volts / Amp. Must be >=
  //   0.
  // - i_gain: I gain of the PI current controller, in Volts / (Amp second).
  //   Must be >= 0.
  // - anti_wind_gain: Anti-wind gain of the controller, in Amps / Volt. Must be
  //   >= 0.
  // - saturation_voltage_margin: Margin to use between the saturation voltage
  //   of the controller, and the maximum theoretical sinusoidal commutation
  //   amplitude, i.e.,  supply voltage / 2 i.e.,  the maximum output voltage in
  //   each of the Q and D frames. Must be >= 0.
  // - phase_resistance: motor resistance per phase
  // - phase_inductance: motor inductance per phase
  // - encoder: reference to the encoder
  // - gate_driver: reference to gate driver
  // - phase_order: phase ordering of motor windings
  // - num_pole_pairs: number of pole pairs in th motor
  // - elect_zero_offset: calibrated zero offset for the motor electrical angle
  //   relative to encoder position
  static pw::Result<Foc*> Build(float dt, float p_gain, float i_gain,
                                float anti_wind_gain, float saturation_voltage,
                                float phase_resistance, float phase_inductance,
                                RotaryEncoder& encoder,
                                GateDriverInterface& gate_driver,
                                PhaseOrder phase_order, uint32_t num_pole_pairs,
                                int32_t elec_zero_offset);

  pw::Status Start(float bus_voltage) override;
  pw::Status Stop() override;

  bool IsRunning() { return running_; };

  pw::Status SetTargetQ(float iq, float bus_voltage);

  FocState GetState() override;

  PhaseOrder GetPhaseOrder() override { return phase_order_; };

  PhaseSampleSelection GetPhaseSampleSelection() override {
    return phase_select_;
  };

  PhaseSampleSelection adc_isr(int32_t adc_a, int32_t adc_b,
                               int32_t adc_c) override;

  RotaryEncoder& GetEncoder() { return encoder_; }

  GateDriverInterface& GetGateDriver() { return gate_driver_; }

  // set the D/Q pid controllers to open loop
  void SetOpenLoop(bool enable);

  void Init(float dt, float p_gain, float i_gain, float anti_wind_gain,
            float saturation_voltage, float phase_resistance,
            float phase_inductance, PhaseOrder phase_order,
            uint32_t num_pole_pairs, int32_t elec_zero_offset);

 private:
  explicit Foc(float dt, float p_gain, float i_gain, float anti_wind_gain,
               float saturation_voltage, float phase_resistance,
               float phase_inductance, RotaryEncoder& encoder,
               GateDriverInterface& gate_driver, PhaseOrder phase_order,
               uint32_t num_pole_pairs, int32_t elec_zero_offset);

  float dt_;
  bool running_;
  float target_q_;
  float feed_forward_;
  float bus_voltage_;

  uint32_t num_pole_pairs_;
  int32_t elec_zero_offset_;
  PhaseOrder phase_order_;
  float saturation_voltage_margin_;
  float phase_resistance_;
  float phase_inductance_;
  FocState state_;

  PhaseSampleSelection phase_select_;

  SinusoidalCommutation commutator_;
  RotaryEncoder& encoder_;
  PidController pid_d_;
  PidController pid_q_;
  GateDriverInterface& gate_driver_;

  // Save last angle and vel for next update
  float last_elec_angle_;
  float last_delta_elec_angle_;
  std::optional<uint32_t> last_raw_encoder_counts_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_REALTIME_FOC_H_
