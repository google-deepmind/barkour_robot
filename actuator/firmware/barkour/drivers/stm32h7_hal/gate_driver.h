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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GATE_DRIVER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GATE_DRIVER_H_

#include <cstdint>
#include <utility>

#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_result/result.h"
#include "pw_span/span.h"
#include "pw_status/status.h"
#include "drv8353.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT

namespace barkour {

// Provides simplified access to the gate driver.
// All methods in this class are non-blocking and intended to be used in an
// asynchronous way. This class implements a simple state machine to keep track
// of the state of the gate driver based on the hardware state (e.g. STO) and
// software state.
class GateDriver : public GateDriverInterface {
 public:
  // How many consecutive times that each FB_STO line is allowed to be observed
  // down for (during calls to `Update`) before powering off the gate driver.
  //
  // See b/265942182 for more context.
  inline static constexpr uint8_t kMaxConsecutiveStoDownReadings = 5;

  // Singleton access to GateDriver.
  static GateDriver& Get();

  // Updates the internal state of the gate driver.
  //
  // Call this regularly to keep the track of hardware state.
  //
  // Returns the state of the gate driver after update has been performed.
  GateDriverState Update() override;

  // Returns whether there was voltage on the STO lines during the last call to
  // `Update`.
  inline bool VoltageOnStoLines() const override {
    return voltage_on_sto_lines_.first && voltage_on_sto_lines_.second;
  }

  inline bool VoltageOnStoLine1() const override {
    return voltage_on_sto_lines_.first;
  }

  inline bool VoltageOnStoLine2() const override {
    return voltage_on_sto_lines_.second;
  }

  // Returns the current state of the gate driver.
  GateDriverState CurrentState() const override;

  // Sets the desired state of the gate driver.
  // This method first calls Update() before evaluating whether the desired
  // state transitions are valid.
  //
  // Args:
  // - state: Desired state of the gate driver. This should be either
  //   AP_GATE_DRIVER_STATE_POWER_OFF or
  //   AP_GATE_DRIVER_STATE_POWER_ON_GATE_ENABLED.
  //
  // Returns:
  // - PW_STATUS_OK: if the target state is valid and the gate driver is
  //   currently not in an error state.
  // - PW_STATUS_INVALID_ARGUMENT: If state is not a permissible target state
  //   (e.g. AP_GATE_DRIVER_STATE_NOT_INITIALIZED).
  // - PW_STATUS_FAILED_PRECONDITION: If the current state is an error state.
  pw::Status SetTargetState(const GateDriverState& state) override;

  // Controls the duty cycles for the PWM outputs.
  //
  // Args:
  // - duty_cycles: da,db,dc  are 3 floating point numbers between 0 and 1
  //   (inclusive) specifying the PWM duty cycle, one value for each phase. 0 ==
  //   low side FET on for the phase 100% of the time, 1 == high side FET on
  //   100% of the time. Values outside the range [0, 1] will be clipped to this
  //   range.
  // - ignore_current_state: if true, this method does not verify that the gate
  //   driver is currently in PowerOnGateEnabled mode.
  //
  //
  //  NOTE: assumes da,db,dc are in physical order, if phases are swapped the
  //  parameters
  //        must be swapped before passing to this function.
  //
  //
  // Returns:
  // - pw::Ok: If the PWM duty cycles were updated successfully.
  // - pw::Status::FailedPrecondition: If ignore_current_state == false and the
  //   gate driver is not in the PowerOnGateEnabled state.
  // - For pw::Ok the returned value is a value of the PhaseSampleSelection enum
  //   indicating which two phase currents should be sampled by the injected
  //   ADCs on the next FOC cycle.
  pw::Result<PhaseSampleSelection> SetDutyCycles(
      float da, float db, float dc, bool ignore_current_state = false) override;

  // Convert a duty cycle in rang +1 1 to the value to load into timer CMP
  // register period = PWM Full period time duty_cycle = 0-1  or -1-0 if
  // duty_cycle > 0 compare value is for the turn off time and set > period/2 if
  // duty_cycle < 0 compare value is for the turn on  time and set < period/2
  // duty = 0 compare = period /2
  // duty_cycle = +1 compare = period
  // duty_cycle = -1 compare = 0
  uint32_t DutyCycleToCompareValue(uint32_t period, float duty_cycle);

  // Gets status register contents via SPI transactions.
  GateDriverStatusRegisters GetFaultStatusRegisters() override;

 private:
  GateDriver();
  // Current state of the gate driver.
  GateDriverState state_;

  // Whether there was voltage on the two STO lines, during the last call to
  // Update().
  std::pair<bool, bool> voltage_on_sto_lines_;

  // Desired state of the gate driver (user controlled).
  GateDriverState target_state_;
  // The low-level gate driver implementation.
  DRV8353_Obj motor_driver_;
  // The low-level SPI handle for the gate driver's SPI interface.
  SPI_HandleTypeDef spi_handle_;
  // Handles for HRTIM/PWM configuration.
  static constexpr uint8_t kNumPhases = 3;
  HRTIM_HandleTypeDef high_resolution_timer_config_;
  HRTIM_TimeBaseCfgTypeDef timebase_config_[kNumPhases];
  HRTIM_TimerCfgTypeDef timer_config_[kNumPhases];
  HRTIM_OutputCfgTypeDef output_config_[kNumPhases];
  HRTIM_CompareCfgTypeDef compare_config_[kNumPhases];
  HRTIM_DeadTimeCfgTypeDef deadtime_config_[kNumPhases];
  HRTIM_FaultCfgTypeDef fault_config_[kNumPhases];

  // Pulls the Gate_Enable pin on the gate driver high. If 24Vsafe is turned on,
  // this will effectively enable the DRV8353 by enabling the charge pumps and
  // SPI interface.
  void GateEnable();
  // Pull the Gate_Enable pin on the gate driver low. If 24Vsafe is turned on,
  // this will put the DRV8353 in a low-power state and disable the charge pumps
  // and SPI interface. The device must be reconfigured after exiting the
  // low-power mode.
  void GateDisable();

  // Uses the nFault pin to determine whether or not the motor has encountered
  // an error condition.
  bool MotorFault();

  // Configures SPI during NOT_INITIALIZED -> DISABLED transition.
  pw::Status Initialize();
  // Configures the DRV8353 over SPI.
  pw::Status ConfigureMotorDriver();
  // Configures the PWM module and pins and starts the timer.
  pw::Status StartPwm();
  // Sets the PWM output to a safe mode (e.g. motor coasting).
  pw::Status StopPwm();

  // Displays Gate Driver State.
  void LogGateDriverState(DRV_SPI_8353_Vars_t& driver_vars);

  // Down counters for how many times consecutively that each FB_STO line is
  // observed to be down during the PowerOnGateEnabled state. Once either of
  // these goes above kMaxConsecutiveStoDownReadings, the gate driver machine
  // will be shut down.
  uint8_t sto_line_1_down_counter_;
  uint8_t sto_line_2_down_counter_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_DRIVERS_STM32H7_HAL_GATE_DRIVER_H_
