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

#include <cmath>
#include <cstdint>

#include "actuator/firmware/barkour/common/adc_conversions.h"
#include "actuator/firmware/barkour/common/derived_sensor_information.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/foc_commutation_test_data.h"  // include generated test data
#include "actuator/firmware/barkour/drivers/testing/fake_adc.h"
#include "actuator/firmware/barkour/drivers/testing/fake_gate_driver.h"
#include "actuator/firmware/barkour/drivers/testing/fake_rotary_encoder.h"
#include "pw_assert/check.h"
#include "pw_log/log.h"
#include "pw_status/status.h"
#include "pw_status/try.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

extern barkour::Foc* foc_instance;

namespace barkour {
namespace {

// Data structure for generated test data.
typedef struct {
  struct {
    int32_t raw_enc_reading;
    uint16_t adc1;
    uint16_t adc2;
  } input;
  FocInterface::FocState expect;
} FOC_COM_TEST_DATA;

// Import generated test data
FOC_COM_TEST_DATA foc_com_test_data[NUM_COM_TESTS] = FOC_COM_TEST_DATA_INIT;

TEST(RealTimeFocTest, StartBadBusVolts) {
  static FakeRotaryEncoder def_fake_encoder(1024);
  static FakeGateDriver def_fake_gate_driver;

  float p_gain = 0.1f;
  float i_gain = 0.1f;
  float anti_wind_gain = 0.0f;
  float saturation_voltage = 1.0f;
  float phase_resistance = 1.0f;
  float phase_inductance = 1.0f;
  uint32_t num_pole_pairs = 21;
  int32_t elec_zero_offset = 0;

  pw::Result<Foc*> foc = Foc::Build(
      Foc::kFocDt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
      phase_resistance, phase_inductance, def_fake_encoder,
      def_fake_gate_driver, PhaseOrder::kAbc, num_pole_pairs, elec_zero_offset);

  foc_instance = nullptr;

  EXPECT_EQ(foc.ok(), true);

  EXPECT_NE(foc.value(), nullptr);

  EXPECT_EQ(foc.value()->Start(1.0f), pw::Status::FailedPrecondition());

  EXPECT_EQ(foc.value()->Stop(), pw::OkStatus());

  EXPECT_TRUE(true);
}

TEST(RealTimeFocTest, Start) {
  static FakeRotaryEncoder def_fake_encoder(1024);
  static FakeGateDriver def_fake_gate_driver;

  float p_gain = 1.0f;
  float i_gain = 1.0f;
  float anti_wind_gain = 1.0f;
  float saturation_voltage = 1.0f;
  float phase_resistance = 1.0f;
  float phase_inductance = 1.0f;
  uint32_t num_pole_pairs = 21;
  int32_t elec_zero_offset = 100;

  pw::Result<Foc*> foc = Foc::Build(
      Foc::kFocDt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
      phase_resistance, phase_inductance, def_fake_encoder,
      def_fake_gate_driver, PhaseOrder::kAbc, num_pole_pairs, elec_zero_offset);

  EXPECT_EQ(foc.ok(), true);

  EXPECT_NE(foc.value(), nullptr);

  EXPECT_EQ(foc.value()->Start(24.0f), pw::OkStatus());

  EXPECT_EQ(foc.value()->Stop(), pw::OkStatus());

  EXPECT_TRUE(true);
}

TEST(RealTimeFocTest, Stop) {
  static FakeRotaryEncoder def_fake_encoder(1024);
  static FakeGateDriver def_fake_gate_driver;

  float p_gain = 1.0f;
  float i_gain = 1.0f;
  float anti_wind_gain = 1.0f;
  float saturation_voltage = 1.0f;
  float phase_resistance = 1.0f;
  float phase_inductance = 1.0f;
  uint32_t num_pole_pairs = 21;
  int32_t elec_zero_offset = 100;

  pw::Result<Foc*> foc = Foc::Build(
      Foc::kFocDt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
      phase_resistance, phase_inductance, def_fake_encoder,
      def_fake_gate_driver, PhaseOrder::kAbc, num_pole_pairs, elec_zero_offset);

  EXPECT_EQ(foc.ok(), true);

  EXPECT_EQ(foc.value()->Stop(), pw::OkStatus());
}

TEST(RealTimeFocTest, Ranges) {
  static FakeRotaryEncoder def_fake_encoder(1024);
  static FakeGateDriver def_fake_gate_driver;

  float p_gain = 1.0f;
  float i_gain = 1.0f;
  float anti_wind_gain = 1.0f;
  float saturation_voltage = 1.0f;
  float phase_resistance = 1.0f;
  float phase_inductance = 1.0f;
  uint32_t num_pole_pairs = 21;
  int32_t elec_zero_offset = 100;

  // All Ok
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::OkStatus());

  // Time Step and set negative P gain
  p_gain = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative P gain.
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative I gain.
  p_gain = 1.0f;
  i_gain = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative anti-wind gain.
  i_gain = 1.0f;
  anti_wind_gain = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative saturation voltage margin.
  anti_wind_gain = 1.0f;
  saturation_voltage = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative phase resistance.
  saturation_voltage = 1.0f;
  phase_resistance = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Negative phase inductance.
  phase_resistance = 1.0f;
  phase_inductance = -1.0f;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());

  // Zero Pole pairs
  phase_inductance = 1.0f;
  num_pole_pairs = 0;
  EXPECT_EQ(Foc::Build(Foc::kFocDt, p_gain, i_gain, anti_wind_gain,
                       saturation_voltage, phase_resistance, phase_inductance,
                       def_fake_encoder, def_fake_gate_driver, PhaseOrder::kAbc,
                       num_pole_pairs, elec_zero_offset)
                .status(),
            pw::Status::InvalidArgument());
}

// Test the math of the complete FOC algorithm implemented in Foc::adc_isr
// function
TEST(RealTimeFocTest, FocUpdateCycles) {
  uint32_t encoder_counts_per_turn = 0xFFFF;
  float bus_voltage = 24.0f;

  static FakeRotaryEncoder def_fake_encoder(encoder_counts_per_turn);
  static FakeGateDriver def_fake_gate_driver;

  float p_gain = 1.0f;
  float i_gain = 1.0f;
  float anti_wind_gain = 0.0f;
  float saturation_voltage = 1.0f;
  float phase_resistance = 1.0f;
  float phase_inductance = 1.0f;
  // encoder_counts_per_turn / num_pole_pairs / 8;
  uint32_t num_pole_pairs = 21;
  int32_t elec_zero_offset = 0;

  // All Ok
  pw::Result<Foc*> maybe_foc = Foc::Build(
      Foc::kFocDt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
      phase_resistance, phase_inductance, def_fake_encoder,
      def_fake_gate_driver, PhaseOrder::kAbc, num_pole_pairs, elec_zero_offset);

  // Stops ADC actual interrupts calling the adc_isr handler while we are trying
  // to run tests.
  foc_instance = nullptr;

  EXPECT_EQ(maybe_foc.status(), pw::OkStatus());

  Foc* foc = maybe_foc.value();

  // re-init the global instance with test specific prams.
  foc->Init(Foc::kFocDt, p_gain, i_gain, anti_wind_gain, saturation_voltage,
            phase_resistance, phase_inductance, PhaseOrder::kAbc,
            num_pole_pairs, elec_zero_offset);

  FakeRotaryEncoder& fake_encoder = (FakeRotaryEncoder&)foc->GetEncoder();
  FakeGateDriver& fake_gate_driver = (FakeGateDriver&)foc->GetGateDriver();

  fake_encoder.counts_per_turn_ = encoder_counts_per_turn;

  EXPECT_EQ(fake_gate_driver.SetTargetState(
                barkour::GateDriverState::PowerOnGateEnabled()),
            pw::OkStatus());

  fake_gate_driver.voltage_on_sto_lines_ = true;

  EXPECT_EQ(fake_gate_driver.Update(),
            barkour::GateDriverState::PowerOnGateEnabled());
  EXPECT_EQ(foc->Start(bus_voltage), pw::OkStatus());

  // Test data based on open loop pid (pids are tested elsewhere).
  foc->SetOpenLoop(true);
  foc->SetTargetQ(1.0f, bus_voltage);

  float f_tol = 0.005f;  // tolerance for results

  PhaseSampleSelection pss = PhaseSampleSelection::kAB;

  // Iterate over each data point in the test data.
  // Apply inputs and calculate outputs, compare generated output
  // matches expected output from test data.
  for (int idx = 0; idx < NUM_COM_TESTS; idx++) {
    // Get data point.
    FOC_COM_TEST_DATA& foc_com_test = foc_com_test_data[idx];

    // Setup function inputs.
    int32_t raw_encoder_counts = foc_com_test.input.raw_enc_reading;
    int32_t adc1 = (int32_t)foc_com_test.input.adc1 - 0x7fff;
    int32_t adc2 = (int32_t)foc_com_test.input.adc2 - 0x7fff;
    int32_t adc3 = -(adc1 + adc2);

    int32_t ia = 0;
    int32_t ib = 0;
    int32_t ic = 0;

    switch (pss) {
      case barkour::PhaseSampleSelection::kAB: {
        ia = adc1;
        ib = adc2;
        ic = adc3;
        break;
      }
      case barkour::PhaseSampleSelection::kAC: {
        ia = adc2;
        ib = adc3;
        ic = adc1;
        break;
      }
      case barkour::PhaseSampleSelection::kBC: {
        ia = adc3;
        ib = adc2;
        ic = adc1;
        break;
      }
      default: {
        break;
      }
    }

    fake_encoder.SetRawEncoderCounts(raw_encoder_counts);
    fake_encoder.multiturn_encoder_counts_ = raw_encoder_counts;
    pw::Result<RotaryEncoderState> enc_state = fake_encoder.Update();

    EXPECT_EQ(enc_state.status(), pw::OkStatus());

    // Run the FOC step with given inputs, (this is what we are testing).
    pss = foc->adc_isr(ia, ib, ic);
    EXPECT_NE(pss, PhaseSampleSelection::kABC);

    // Get results.
    FocInterface::FocState foc_state = foc->GetState();

    // Get expected results.
    float expect_ia = foc_com_test.expect.phase_a_current_;
    float expect_ib = foc_com_test.expect.phase_b_current_;
    float expect_ic = foc_com_test.expect.phase_c_current_;

    float expect_elec_angle = foc_com_test.expect.elec_angle_;

    float expect_phase_a_duty = foc_com_test.expect.phase_a_duty_;
    float expect_phase_b_duty = foc_com_test.expect.phase_b_duty_;
    float expect_phase_c_duty = foc_com_test.expect.phase_c_duty_;

    float expect_i_d = foc_com_test.expect.i_d_;
    float expect_i_q = foc_com_test.expect.i_q_;

    float expect_v_d = foc_com_test.expect.v_d_;
    float expect_v_q = foc_com_test.expect.v_q_;

    PhaseSampleSelection expect_pss = foc_com_test.expect.pss_;

    // Verify all outputs are within tolerance.
    EXPECT_EQ(pss, expect_pss);

    EXPECT_LT(fabsf(foc_state.phase_a_current_ - expect_ia), f_tol);
    EXPECT_LT(fabsf(foc_state.phase_b_current_ - expect_ib), f_tol);
    EXPECT_LT(fabsf(foc_state.phase_c_current_ - expect_ic), f_tol);

    EXPECT_LT(fabsf(foc_state.phase_a_duty_ - expect_phase_a_duty), f_tol);
    EXPECT_LT(fabsf(foc_state.phase_b_duty_ - expect_phase_b_duty), f_tol);
    EXPECT_LT(fabsf(foc_state.phase_c_duty_ - expect_phase_c_duty), f_tol);

    EXPECT_LT(fabsf(foc_state.i_d_ - expect_i_d), f_tol);
    EXPECT_LT(fabsf(foc_state.i_q_ - expect_i_q), f_tol);

    EXPECT_LT(fabsf(foc_state.v_d_ - expect_v_d), f_tol);
    EXPECT_LT(fabsf(foc_state.v_q_ - expect_v_q), f_tol);

    EXPECT_LT(fabsf(foc_state.elec_angle_ - expect_elec_angle), f_tol);
  }
}

}  // namespace
}  // namespace barkour
