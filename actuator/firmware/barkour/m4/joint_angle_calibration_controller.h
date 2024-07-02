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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_JOINT_ANGLE_CALIBRATION_CONTROLLER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_JOINT_ANGLE_CALIBRATION_CONTROLLER_H_

#include <type_traits>

#include "actuator/firmware/barkour/common/pid_controller.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// Container for calibration parameters which are likely to be constants.
struct JointAngleCalibrationConstants {
  // P gain for the position controller used in the calibration routine.
  // Units: Volts / radian.
  float p_gain;

  // The maximum quadrature voltage to apply during the routine. Note that the
  // maximum torque applied will be no more than:
  // `saturation_voltage` * (motor torque constant) / (motor resistance).
  // Units: Volts.
  float saturation_voltage;

  // The rate at which to move the position setpoint during the calibration
  // routine.
  // Units: radians / tick.
  float radians_per_tick;

  // The threshold for observed position controller error before the
  // calibration routine will be deemed to have finished.
  // Units: radians.
  float controller_error_finished_threshold;
};

// Implements the joint angle calibration routine.
//
// This rotates the actuator slowly in a positive direction, until it hits the
// endstop of the calibration rig. It uses a P controller (inputs: actuator
// position, outputs: actuator quadrature voltage) with a
// continuously-increasing position reference, and finishes when the error in
// the controller is larger than a given value, indicating that the position of
// the motor has not been able to follow the setpoint due to the calibration
// endstop.
//
// This class is templated on the TickType so that the unit tests do not have to
// rely on FreeRTOS, but the actual controller can be instantiated with the
// actual FreeRTOS tick type.
template <typename TickType>
class JointAngleCalibrationController {
  static_assert(std::is_unsigned<TickType>::value,
                "TickType must be an unsigned integer type.");

 public:
  explicit JointAngleCalibrationController(
      const JointAngleCalibrationConstants& constants, float starting_angle,
      TickType starting_tick)
      : constants_(constants),
        starting_angle_(starting_angle),
        starting_tick_(starting_tick),
        // Set the update period of the PID controller to be 1 ms. We don't use
        // the I term so this shouldn't matter.
        pid_(1.0e-3),
        finished_(false) {
    pid_.SetProportionalGain(constants_.p_gain);
    pid_.SetSaturation(-constants.saturation_voltage,
                       constants.saturation_voltage);
    pid_.Reset(starting_angle_);

    PW_LOG_INFO(
        "Creating joint angle calibration controller, with parameters:");
    PW_LOG_INFO("P gain: %f", constants.p_gain);
    PW_LOG_INFO("Saturation voltage: %f", constants.saturation_voltage);
    PW_LOG_INFO("Radians per tick: %f", constants.radians_per_tick);
    PW_LOG_INFO("Controller error finished threshold: %f",
                constants.controller_error_finished_threshold);
  }

  // Updates the controller with the current shaft angle.
  //
  // If the calibration has finished, the returned quadrature voltage to apply
  // will be zero.
  //
  // Returns:
  // - Invalid argument error: If the current tick is less than the starting
  //   tick, indicating a tick counter overflow.
  // - float: Quadrature voltage to apply, if the calibration update has
  //   succeeded.
  pw::Result<float> Update(float current_angle, TickType current_tick) {
    if (current_tick < starting_tick_) {
      // Tick counter has overflowed.
      return pw::Status::InvalidArgument();
    }

    // `ticks_since_start` is guaranteed to be positive by the overflow check
    // above.
    TickType ticks_since_start = current_tick - starting_tick_;
    float target_position =
        starting_angle_ + ticks_since_start * constants_.radians_per_tick;

    // Compute target output. We just use the P term, so can assume that shaft
    // velocity is zero for this calculation.
    pid_.SetReference(target_position);
    float target_q_voltage = pid_.ComputeOutput(current_angle, 0);

    finished_ = finished_ || (target_position - current_angle >
                              constants_.controller_error_finished_threshold);

    if (finished_) {
      // Disable voltage.
      return 0.0f;
    }

    return target_q_voltage;
  }

  // Returns whether the routine has finished yet.
  bool Finished() const { return finished_; }

 private:
  const JointAngleCalibrationConstants constants_;
  float starting_angle_;
  TickType starting_tick_;
  PidController pid_;
  bool finished_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_JOINT_ANGLE_CALIBRATION_CONTROLLER_H_
