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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_UTYPES_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_UTYPES_H_

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#include "actuator/firmware/barkour/m4/ecat/ecat_options.h"
#include "pw_sync/mutex.h"

// Object dictionary storage.

// Motor Parameters in DS402 units.
enum PhaseOrder { kAbcPhaseOrder = 0, kAcbPhaseOrder = 1 };

struct MotorParameters {
  uint32_t rated_current;    // mA
  uint32_t rated_torque;     // mNm
  uint32_t torque_constant;  // mNm/A
  uint16_t current_limit;    // 1000s of rated_current
  uint16_t torque_limit;     // 1000s of rated_torque
  uint32_t serial_number;
  uint64_t extended_serial_number;
  uint16_t num_pole_pairs;
  uint8_t phase_order;  // Order of the phases. See PhaseOrder enum.
  uint32_t phase_to_phase_resistance;  // phase to phase resistance in mOhm
  uint32_t encoder_increments;         // Counts per revolution
  uint32_t encoder_motor_revolutions;  // == 1
  uint32_t gear_motor_revolutions;     // Number of motor revolutions per shaft
                                       // revolutions
  uint32_t gear_shaft_revolutions;     // == 1
};

struct FoeData {
  uint32_t bank_swap_reset;
};

struct _Objects {
  // Tx Data (inputs)
  struct {
    // CiA402 objects
    int32_t position;  // observed position.
    uint16_t statusword;
    int32_t velocity;  // observed torque.
    int16_t torque;    // observed torque.
    uint8_t mode;
    uint8_t error;                 // CANopen error register.
    uint32_t manufacturer_status;  // Manufacturer-specific status register.
    int16_t motor_current;         // observed current.

    // Blob for M7
    uint8_t blob[MAX_M7BLOB_OBJECTS];

    // Barkour motor driver specific information.
    // Mainly used for debugging purposes.

    // Observed bus voltage [V]
    float bus_voltage;
    // Observed phase currents [A].
    float phase_a_current;
    float phase_b_current;
    float phase_c_current;
    // Thermistor readings [C].
    float thermistor_1;  // FPC connector
    float thermistor_2;  // FPC connector
    float thermistor_3;  // FPC connector
    float thermistor_4;  // Encoder pigtail.
    // Observed quadrature and direct currents [A].
    float quadrature_current;
    float direct_current;
    // Shaft angle, relative to the encoder zero angle [Radian].
    float shaft_angle;
    // Electrical angle [Radian]
    float electrical_angle;
    // Latest quadrature and direct voltage commands (u_q, u_d) [V]
    float quadrature_voltage;
    float direct_voltage;
    // PWM duty cycles (motor controller output) [0-1]
    float phase_a_duty_cycle;
    float phase_b_duty_cycle;
    float phase_c_duty_cycle;

    // STO information
    uint8_t fb_sto1_state;
    uint8_t fb_sto2_state;
    float sto_24v_safe_adc_voltage;

    // Gate driver information
    uint16_t gate_driver_fault_status_1;
    uint16_t gate_driver_fault_status_2;

    // IMU
    // Accelerometer [m/s^2]
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
  } tx;

  // Rx Data (outputs)
  struct {
    // CiA402 objects
    uint32_t speed_limit;
    uint16_t controlword;
    int16_t torque_offset;
    int16_t torque;
    int32_t velocity;
    uint8_t mode;

    // Blob for M7
    uint8_t blob[MAX_M7BLOB_OBJECTS];
  } rx;

  struct MotorParameters parameter;

  struct FoeData foe;

  struct {
    uint16_t Sync_mode;
    uint32_t CycleTime;
    uint32_t ShiftTime;
    uint16_t Sync_modes_supported;
    uint32_t Minimum_Cycle_Time;
    uint32_t Calc_and_Copy_Time;
    uint32_t Minimum_Delay_Time;
    uint16_t GetCycleTime;
    uint32_t DelayTime;
    uint32_t Sync0CycleTime;
    uint16_t SMEventMissedCnt;
    uint16_t CycleTimeTooSmallCnt;
    uint16_t Shift_too_short_counter;
    uint16_t RxPDOToggleFailed;
    uint32_t Minimum_Cycle_Distance;
    uint32_t Maximum_Cycle_Distance;
    uint32_t Minimum_SM_Sync_Distance;
    uint32_t Maximum_SM_Sync_Distance;
    uint8_t SyncError;
  } SyncMgrParam;

  struct {
    char name[20];
  } SDO1008;

  struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision_no;
    uint32_t serial_no;
  } SDO1018;

  // Manufacturer specific data.
  //
  // See barkour/doc/custom_objects.md for a detailed description.

  // The git commit at which the firmware was compiled, this is used as the
  // software version.
  struct {
    char git_commit[40];  // A git commit hash is always 40 characters long.
  } SDO100A;

  // SDO 3000 relates to the electrical angle offset calibration, which is
  // required to be completed before the motor can operate.
  struct {
    int32_t electrical_angle_offset;
    uint8_t electrical_angle_calibration_completed;
  } SDO3000;

  // SDO 3001 relates to the joint angle offset calibration, which is required
  // to be completed before the motor can operate.
  struct {
    int32_t joint_angle_offset;
    uint8_t joint_angle_calibration_completed;
  } SDO3001;

  // Dynamic TX PDO:s
  struct {
    uint8_t maxsub;
    uint32_t value[16];
  } SDO1620;

  // Dynamic RX PDO:s
  struct {
    uint8_t maxsub;
    uint32_t value[16];
  } SDO1A20;

  // Dynamic Sync Managers
  struct {
    uint8_t maxsub;
    uint32_t value[8];
  } SM1C12;

  struct {
    uint8_t maxsub;
    uint32_t value[27];
  } SM1C13;
};

extern pw_sync_Mutex ObjMutex;
extern struct _Objects Obj PW_GUARDED_BY(ObjMutex);

// Flags to be communicated between the EtherCAT and controller tasks, which
// don't need to be exposed over EtherCAT.
//
// Portability note: there is no compiler-independent guarantee that `bool` in C
// as defined by stdbool.h is cross-compatible with `bool` in C++. This
// generally works for linking binaries which are entirely complied by a single
// mainstream compiler, but strange things could happen if ABI compatibility is
// relied on.
struct _EthercatFlags {
  // Whether the calibration flags have been loaded from EEPROM yet.
  bool calibration_loaded_from_eeprom;

  // Whether the electrical and joint angle calibration values will be written
  // to EEPROM on the next transition from INIT -> PREOP.
  bool write_electrical_angle_calibration_value;
  bool write_joint_angle_calibration_value;
};

extern pw_sync_Mutex EthercatFlagsMutex;
extern struct _EthercatFlags EthercatFlags PW_GUARDED_BY(EthercatFlagsMutex)
    PW_ACQUIRED_AFTER(ObjMutex);

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_UTYPES_H_
