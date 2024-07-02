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

#include "actuator/firmware/barkour/common/pid_controller.h"

#include <algorithm>
#include <limits>

#include "pw_log/log.h"

namespace barkour {

PidController::PidController(float dt)
    : dt_(dt),
      p_gain_(0.0f),
      d_gain_(0.0f),
      i_gain_(0.0f),
      anti_wind_gain_(0.0f),
      saturation_low_(std::numeric_limits<float>::lowest()),
      saturation_high_(std::numeric_limits<float>::max()),
      current_reference_(0.0f),
      current_derivative_reference_(0.0f),
      current_feedforward_reference_(0.0f),
      error_(0.0f),
      error_integral_(0.0f),
      open_loop_(false) {}

void PidController::SetProportionalGain(float p_gain) {
  p_gain_ = p_gain;
  PW_LOG_DEBUG("PID Controller P-Gain set=%f", p_gain);
}

void PidController::SetDerivativeGain(float d_gain) {
  d_gain_ = d_gain;
  PW_LOG_DEBUG("PID Controller D-Gain set=%f", d_gain);
}

void PidController::SetIntegralGain(float i_gain) {
  i_gain_ = i_gain;
  PW_LOG_DEBUG("PID Controller I-Gain set=%f", i_gain);
}

void PidController::SetAntiWindGain(float anti_wind_gain) {
  anti_wind_gain_ = anti_wind_gain;
}

void PidController::SetSaturation(float saturation_low, float saturation_high) {
  saturation_low_ = saturation_low;
  saturation_high_ = saturation_high;
}

void PidController::Reset(float current_state) {
  // Set the current reference to the current state, and everything else to
  // zero.
  current_reference_ = current_state;
  current_derivative_reference_ = 0.0f;
  current_feedforward_reference_ = 0.0f;
  error_ = 0.0f;
  error_integral_ = 0.0f;
}

void PidController::SetReference(float reference) {
  current_reference_ = reference;
  current_derivative_reference_ = 0.0f;
  current_feedforward_reference_ = 0.0f;
}

void PidController::SetTrajectoryReference(float reference,
                                           float derivative_reference,
                                           float feedforward) {
  current_reference_ = reference;
  current_derivative_reference_ = derivative_reference;
  current_feedforward_reference_ = feedforward;
}

float PidController::ComputeOutput(float current_state,
                                   float current_derivative) {
  // in open loop mode just bypass the controller and return the reference
  if (open_loop_) {
    return current_reference_;
  }

  // Compute the error: e:= ref- current; also the error on the derivative.
  // Now compute the output.
  float unsaturated_output = 0.0f;

  error_ = current_reference_ - current_state;
  float error_derivative = (current_derivative_reference_ - current_derivative);

  unsaturated_output = current_feedforward_reference_;  // Feedforward
  unsaturated_output += p_gain_ * error_;               // Proportional term
  unsaturated_output += d_gain_ * error_derivative;     // Derivative term
  unsaturated_output += i_gain_ * error_integral_;      // Integral_term

  float output =
      std::clamp(unsaturated_output, saturation_low_, saturation_high_);

  // Now that the output is saved, compute the distance between the actual value
  // and the saturated value as it is needed by the antiwindup term.
  // We want to compute sat(u) -u
  unsaturated_output -= output;  // This computes u - sat(u).

  // Now update the integral for the next time step.
  // Note the integral is approximated as a forward difference, i.e.
  //   I(t+1) = I(t) + e dt.
  // That is, we update the integral after computing the current output.
  error_integral_ += (error_ * dt_ - anti_wind_gain_ * unsaturated_output);

  return output;
}

float PidController::GetCurrentError() const { return error_; }

void PidController::SetOpenLoop(bool enable) { open_loop_ = enable; }

}  // namespace barkour
