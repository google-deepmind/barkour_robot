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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_MOTOR_CONTROL_MODULE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_MOTOR_CONTROL_MODULE_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

// --- Types for the motor control module abstraction ---

// Control references. Each motor control module will select which of these
// fields it uses as its references.
struct ControlReferences {
  float reference_current = 0.0;  // Amps
};

// Voltages to apply to each axis in the rotating frame. These are used as the
// outputs of the motor control modules.
struct RotatingFrameVoltages {
  float quadrature_voltage = 0.0;
  float direct_voltage = 0.0;
};

// Base class for a motor control module.
//
// A motor control module should define the logic for a particular mode of motor
// control, e.g. FOC-based current control, or a cascaded position -> torque
// controller.
//
// Note that *only* the logic for the control should be defined, i.e. the
// mapping from the control input structs to the output struct. Acquisition of
// and interaction with device hardware, and control over the execution flow of
// the controllers, will be carried out externally and should not be managed by
// derived classes.
//
// This class is not designed to be thread-safe.
class MotorControlModule {
 public:
  // Possible states the controller may be in.
  enum class State : uint8_t {
    // The control module is stopped. `Step()` cannot be called.
    kStopped = 0,

    // The control module is starting. `Step()` should be called cyclically, but
    // the controller is not yet ready to produce usable output. For example,
    // the control module may be starting up signal filters which are yet to
    // stabilize.
    kStarting,

    // The control module is running. `Step()` should be called cyclically, and
    // the output used to control the motor.
    kStarted,

    // The control module is stopping, but it is not yet ready to relinquish
    // control safely. `Step()` should be called cyclically, and the output used
    // to control the motor. For example, the control may be bringing the motor
    // to a controlled stop, before it is able to enter the "stopped" state.
    kStopping,
  };

  // States which can be transitioned to from the "starting" state.
  enum class PossibleStatesFromStarting : uint8_t {
    kStarting,
    kStarted,
    kStopping,
    kStopped,
  };

  // States which can be transitioned to from the "started" state.
  enum class PossibleStatesFromStarted : uint8_t {
    kStarted,
    kStopping,
    kStopped,
  };

  // States which can be transitioned to from the "stopped" state.
  enum class PossibleStatesFromStopping : uint8_t {
    kStopping,
    kStopped,
  };

  MotorControlModule();
  virtual ~MotorControlModule() = default;

  // Starts the control module.
  //
  // After a successful call, the control module will be in either the
  // "starting" or "started" state, depending on the implementation of
  // `DoStart()`.
  //
  // Note that this may fail for implementation-defined reasons, for example the
  // control module may only be able to start if the current motor velocity is
  // close to zero.
  //
  // Returns:
  // - Failed precondition error: If the current state is not "stopped."
  // - Any other error: As forwarded from the derived class' implementation of
  //   `DoStart()`.
  // - OK: If the control module has started successfully.
  pw::Status Start(const SensorReadings& sensor_readings,
                   const SensorDerivedQuantities& sensor_derived_quantities);

  // Requests a stop of the control module.
  //
  // After a successful call, the control module will be in either the
  // "stopping" or "stopped" state, depending on the implementation of
  // `DoStop()`.
  //
  // Note that this cannot fail for implementation-defined reasons,
  // implementations of control modules must always cleanly handle a stop
  // request.
  //
  // Returns:
  // - Failed precondition error: If the current state is not "starting" or
  //   "started".
  // - OK: If the control module has acknowledged the request to stop.
  pw::Status Stop(const SensorReadings& sensor_readings,
                  const SensorDerivedQuantities& sensor_derived_quantities);

  // Requests an instantanous stop of the control module.
  //
  // This should *only* be called in circumstances where the controller is no
  // longer actually exerting any control over the motor, (for example, if the
  // E-stop has been enabled), or something drastic has gone wrong and
  // force-stopping is the least-worst option.
  //
  // Note that this cannot fail for implementation-defined reasons,
  // implementations of control modules must always cleanly handle a force stop.
  //
  // Returns:
  // - Failed precondition error: If the current state is not "starting" or
  //   "started" or "stopping".
  // - OK: If the control module has been stopped.
  pw::Status ForceStop();

  // Steps the control module while it is in the "starting", "started" or
  // "stopping" phase, and produces output voltages to control the motor with.
  //
  // In the "starting" and "stopping" states, the contents of `references` will
  // be ignored.
  //
  // In the "starting" state, if returned result holds a RotatingFrameVoltages
  // value then the voltages are guaranteed to be zero.
  //
  // Note that this cannot fail for implementation-defined reasons, the error
  // handling path for implementations is to transition to the "stopped" state.
  //
  // Returns:
  // - Failed precondition error: If the current state is not "starting" or
  //   "started" or "stopping".
  pw::Result<RotatingFrameVoltages> Step(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references);

  // Returns the current state of the motor control module.
  State GetState() const;

 protected:
  // Helper struct, containing a state type to signal which state to transition
  // to, and rotating frame voltages which are the control outputs.
  template <typename StateType>
  struct StateAndRotatingFrameVoltages {
    StateType state;
    RotatingFrameVoltages voltages;
  };

 private:
  // Method to be called during `Start`, to be overridden by implementations.
  //
  // This will only be called if the motor control module is in the "stopped"
  // state.
  //
  // This should return:
  // - An error result: If it is not possible to start the motor control module
  //   at this time, or:
  // - A value of PossibleStatesFromStarting, indicating the next state to
  //   transition to.
  virtual pw::Result<PossibleStatesFromStarting> DoStart(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) = 0;

  // Method to be called during `Stop`, to be overridden by implementations.
  //
  // This will only be called if the motor control module is in the "starting"
  // or "started" state.
  //
  // This should return a value of PossibleStatesFromStopping, indicating the
  // next state to transition to.
  virtual PossibleStatesFromStopping DoStop(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities) = 0;

  // Method to be called during `ForceStop`, to be overridden by
  // implementations. Following a call to this method, the motor control module
  // will immediately transition to the "stopped" state.
  virtual void DoForceStop() = 0;

  // Method to be called during `Step`, when the motor control module is in the
  // "starting" state, to be overridden by implementations.
  //
  // This should return a value of PossibleStatesFromStarting, indicating the
  // next state to transition to.
  //
  // Note that there are no output voltages returned, since no output is
  // produced while the control module is starting.
  virtual PossibleStatesFromStarting DoStepWhileStarting(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities);

  // Method to be called during `Step`, when the motor control module is in the
  // "started" state, to be overridden by implementations.
  //
  // This should return the control voltages for this step, and a value of
  // PossibleStatesFromStarted, indicating the next state to transition to.
  virtual StateAndRotatingFrameVoltages<PossibleStatesFromStarted> DoStep(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references) = 0;

  // Method to be called during `Step`, when the motor control module is in the
  // "stopping" state, to be overridden by implementations.
  //
  // This should return the control voltages for this step, and a value of
  // PossibleStatesFromStopping, indicating the next state to transition to.
  virtual StateAndRotatingFrameVoltages<PossibleStatesFromStopping>
  DoStepWhileStopping(const SensorReadings& sensor_readings,
                      const SensorDerivedQuantities& sensor_derived_quantities);

  State current_state_;
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_MOTOR_CONTROL_MODULE_H_
