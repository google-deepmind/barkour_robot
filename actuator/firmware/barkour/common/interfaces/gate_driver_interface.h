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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GATE_DRIVER_INTERFACE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GATE_DRIVER_INTERFACE_H_

#include <cstdint>
#include <string_view>

#include "actuator/firmware/barkour/common/phase_sample_selection.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// Container to hold values from the gate driver status registers. See
// https://www.ti.com/lit/ds/symlink/drv8353.pdf, page 56 for more
// information. Note this has a typo, “VGS Status 2” should be “Fault Status
// 2”.
struct GateDriverStatusRegisters {
  uint16_t fault_register_1 = 0;
  uint16_t fault_register_2 = 0;
};

class GateDriverState {
 public:
  // These state code values can be cast in order to obtain the state
  // bit values of the CiA402 control word.
  enum class Code : uint8_t {
    kUnknownError = 0,
    kNotInitialized = 1,
    kPowerOff = 2,
    kWaitingForPowerOn = 3,
    kPowerOnGateEnabledNotConfigured = 4,
    kPowerOnGateEnabled = 5,
  };

  // Functions that create a GateDriverState with the specified code.
  // clang-format off
  static constexpr GateDriverState NotInitialized() {
    return GateDriverState(Code::kNotInitialized, "NOT_INITIALIZED");
  }

  static constexpr GateDriverState PowerOff() {
    return GateDriverState(Code::kPowerOff, "POWER_OFF");
  }

  static constexpr GateDriverState WaitingForPowerOn() {
    return GateDriverState(Code::kWaitingForPowerOn, "WAITING_FOR_POWER_ON");
  }

  static constexpr GateDriverState PowerOnGateEnabledNotConfigured() {
    return GateDriverState(Code::kPowerOnGateEnabledNotConfigured,
                           "POWER_ON_GATE_ENABLED_NOT_CONFIGURED");
  }

  static constexpr GateDriverState PowerOnGateEnabled() {
    return GateDriverState(Code::kPowerOnGateEnabled, "POWER_ON_GATE_ENABLED");
  }

  static constexpr GateDriverState UnknownError() {
    return GateDriverState(Code::kUnknownError, "UNKNOWN_ERROR");
  }
  // clang-format on

  constexpr /* explicit */ operator Code() const { return code_; }  // NOLINT

  bool operator==(const GateDriverState& other) const {
    return code_ == other.code_;
  }

  constexpr bool error() const { return code_ == Code::kUnknownError; }

  // Returns a null-terminated string representation of the Ds402State.
  const char* str() const { return name_.data(); }

 private:
  // Note: It is the caller's responsibility to ensure that the state does not
  //   outlive the underlying character array that the name value points to.
  constexpr GateDriverState(Code code, std::string_view name)
      : code_(code), name_(name) {}

  Code code_;
  std::string_view name_;
};

class GateDriverInterface {
 public:
  virtual ~GateDriverInterface() = default;

  // Returns the current state of the gate driver.
  virtual GateDriverState CurrentState() const = 0;

  // Returns whether there was voltage on the STO lines during the last call to
  // `Update`.
  virtual bool VoltageOnStoLines() const = 0;
  virtual bool VoltageOnStoLine1() const = 0;
  virtual bool VoltageOnStoLine2() const = 0;

  // Updates the internal state of the gate driver.
  //
  // Returns the state of the gate driver after update has been performed.
  virtual GateDriverState Update() = 0;

  // Sets the desired state of the gate driver.
  virtual pw::Status SetTargetState(const GateDriverState& state) = 0;

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
  virtual pw::Result<PhaseSampleSelection> SetDutyCycles(
      float da, float db, float dc, bool ignore_current_state) = 0;

  // Controls PWM outputs and verifies that the gate driver is in
  // PowerOnGateEnabled mode.
  //
  // Args:
  // - duty_cycles: da, db, dc floating points in range of <0; 1>
  //
  // Returns:
  // - pw::Ok: If the PWM duty cycles were updated successfully.
  // - pw::Status::FailedPrecondition: If gate driver is not in the
  //   PowerOnGateEnabled state.
  // - For pw::Ok the returned value is a value of the PhaseSampleSelection enum
  //   indicating which two phase currents should be sampled by the injected
  //   ADCs on the next FOC cycle.
  pw::Result<PhaseSampleSelection> SetDutyCycles(
      float da, float db, float dc) {
    return SetDutyCycles(da, db, dc, /*ignore_current_state=*/false);
  }

  // Gets status register contents.
  virtual GateDriverStatusRegisters GetFaultStatusRegisters() = 0;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_INTERFACES_GATE_DRIVER_INTERFACE_H_
