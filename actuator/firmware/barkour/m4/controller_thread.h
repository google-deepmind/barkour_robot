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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CONTROLLER_THREAD_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CONTROLLER_THREAD_H_

#include <cstdint>

#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/common/ds402_state_machine.h"
#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/interfaces/rotary_encoder_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "actuator/firmware/barkour/common/read_sensors.h"
#include "actuator/firmware/barkour/m4/ds402_drive.h"
#include "actuator/firmware/barkour/m4/ecat_device.h"
#include "pw_status/status.h"
#include "pw_sync/lock_annotations.h"
#include "pw_sync/thread_notification.h"
#include "pw_thread/thread_core.h"

namespace barkour {

// ControllerThread is responsible for maintaining and updating the controller.
//
// In addition to performing control loop updates, it also:
// - Maintains the Ds402 state machine and Ds402 drive interface
// - Reads/writes controller-related values from/to the ethercat object
//   dictionary.
//
// Note: This task should be high priority, to ensure that the thread is woken
//   immediately following the release of the of controller notification.
class ControllerThread : public pw::thread::ThreadCore {
 public:
  // Constructs an instance of ControllerThread
  //
  // Args:
  // - encoder: RotaryEncoder instance.
  // - controller_notification: ThreadNotification instance used for controller
  //    synchronization. This notification establishes the time-base for all
  //    controller loop updates and should be released at a consistent
  //    interval.
  // - controller_notification_period: The time interval between controller
  //    notifications, in seconds.
  explicit ControllerThread(
      RotaryEncoder& encoder,
      pw::sync::ThreadNotification& controller_notification,
      float controller_notification_period);

 private:
#if (MOTOR_TYPE == MOTOR_AK80_9)
  static constexpr uint8_t kNumTemperatureReadingsUsed = 1;
#else
#error "Unknown motor type. Update controller_thread.h"
#endif

  // The entrypoint for the thread.
  void Run() override PW_LOCKS_EXCLUDED(ObjMutex);

  // Runs the electrical angle calibration, and restarts the EtherCAT stack,
  // which causes the calibration to be written to EEPROM.
  //
  // This will block until the calibration routine is completed, and the
  // EtherCAT stack has been requested to restart.
  pw::Status CalibrateElectricalAngleAndRestartEthercat()
      PW_LOCKS_EXCLUDED(EtherCatFlagsMutex, ObjMutex);

  // Runs the joint angle calibration, and restarts the EtherCAT stack,
  // which causes the calibration to be written to EEPROM.
  //
  // This will block until the calibration routine is completed, and the
  // EtherCAT stack has been requested to restart.
  pw::Status CalibrateJointAngleAndRestartEthercat()
      PW_LOCKS_EXCLUDED(EtherCatFlagsMutex, ObjMutex);

  float controller_notification_period_;  // seconds
  AdcInterface& adc_;
  BoardConfig& board_;
  RotaryEncoder& encoder_;
  FocInterface* foc_;
  GateDriverInterface& gate_driver_;
  EthercatDevice& ecat_device_;
  pw::sync::ThreadNotification& controller_notification_;

  ManufacturerStatusRegisterErrorCodes manufacturer_status_register_;
  GateDriverStatusRegisters gate_driver_status_registers_;
};

}  // namespace barkour.

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CONTROLLER_THREAD_H_
