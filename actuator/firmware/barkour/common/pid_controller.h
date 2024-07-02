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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PID_CONTROLLER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PID_CONTROLLER_H_

// Simple 1-dimensional PID controller implementation.

namespace barkour {

// Implements a PID controller with anti-windup correction, discretized with
// sample time `dt`.
//
// Denoting the system state and first derivative as x, x_dot, the corresponding
// references as x_ref, x_dot_ref, the feedforward reference as ff, the output
// of the PID as u, and the gains as K_x, this implements the control law
//
// u = ff
//     - Kp (x - x_ref)
//     - Kd (x_dot - x_dot_ref)
//     - Ki _int_ (x - x_ref)
//
// The anti-windup correction is a further modification to the integral term
// when output saturation is active.
//
// See [1] for a description of the problem and of the possible solutions. This
// class uses the back-calculation method.
//
// [1] Control System Design. Lecture notes for ME 155A. Karl Johan Astrom. 2002
// - Section 6.5.
class PidController {
 public:
  // Constructor. `dt` is step time in seconds. All gains are initialized to
  // zero, and the saturation limits are set to the minimum and maximum float
  // values.
  explicit PidController(float dt);

  // Setters for the gains. It is the user's responsibility to ensure that the
  // gain values are appropriate (e.g. gains >= 0, saturation_low <=
  // saturation_high).
  void SetProportionalGain(float p_gain);
  void SetDerivativeGain(float d_gain);
  void SetIntegralGain(float i_gain);
  void SetAntiWindGain(float anti_wind_gain);
  void SetSaturation(float saturation_low, float saturation_high);

  // Resets the controller, assuming that the state of the system is
  // `current_state`.
  void Reset(float current_state);

  // Sets a reference for the system state. The feedforward and derivative
  // references are assumed to be zero, and set accordingly.
  void SetReference(float reference);

  // Sets a full set of references for the system state.
  void SetTrajectoryReference(float reference, float derivative_reference,
                              float feedforward);

  // Computes and returns the controller output. `SetReference` or
  // `SetTrajectoryReference` must have been called since construction in order
  // for the output to be valid.
  float ComputeOutput(float current_state, float current_derivative);

  // Returns the current error of the controller.
  float GetCurrentError() const;

  // Set PID controller to open loop mode
  // enable = true, controller in OL mode (output = reference)
  // enable = false, controller in normal mode
  void SetOpenLoop(bool enable);

 private:
  float dt_;

  // Gains
  float p_gain_;
  float d_gain_;
  float i_gain_;
  float anti_wind_gain_;
  float saturation_low_;
  float saturation_high_;

  // References
  float current_reference_;
  float current_derivative_reference_;
  float current_feedforward_reference_;

  // Internal state
  float error_;
  float error_integral_;

  bool open_loop_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_PID_CONTROLLER_H_
