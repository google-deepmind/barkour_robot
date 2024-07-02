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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_COMMUTATION_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_COMMUTATION_H_

namespace barkour {

// PWM duty cycles for each of the three phases. Should be between zero and one.
struct ThreePhasePwmCommands {
  float phase_a;
  float phase_b;
  float phase_c;
};

// Base class for commutation implementations.
class CommutationLogic {
 public:
  virtual ~CommutationLogic() = default;

  // Method to perform commutation logic.
  //
  // Args:
  // - u_q: Quadrature axis voltage to apply, in Volts.
  // - u_d: Direct axis voltage to apply, in Volts.
  // - electrical_angle: Electrical angle of the motor, in radians.
  // - bus voltage: motor bus voltage in Volts, i.e. the voltage applied to a
  //   phase at full PWM.
  virtual ThreePhasePwmCommands DoCommutation(float u_q,
                                              float u_d,
                                              float electrical_angle,
                                              float bus_voltage) = 0;
};

// Sinusoidal commutation.
class SinusoidalCommutation : public CommutationLogic {
 public:
  explicit SinusoidalCommutation() = default;
  ~SinusoidalCommutation() override = default;

  ThreePhasePwmCommands DoCommutation(float u_q,
                                      float u_d,
                                      float electrical_angle,
                                      float bus_voltage) override;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_COMMUTATION_H_
