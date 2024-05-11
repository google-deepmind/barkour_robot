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

#ifndef BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CUSTOM_OBJECTS_H_
#define BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CUSTOM_OBJECTS_H_

#include <array>
#include <cstdint>
#include <memory>
#include <variant>

#include "canopen.h"

namespace barkour {

// The following objects are custom CANopen objects defined by the motor driver.
// For more details, see /doc/custom_objects.md in the firmware repo.
//
// Please keep this sorted by index and update the array at the end.

// Electrical angle calibration
inline constexpr CanObjectInfo<float> kElectricalAngleOffsetObjectInfo(0x3000,
                                                                       1);
inline constexpr CanObjectInfo<uint8_t>
    kElectricalAngleOffsetCompletedObjectInfo(0x3000, 2);

// Joint angle calibration
inline constexpr CanObjectInfo<int32_t> kJointAngleOffsetObjectInfo(0x3001, 1);
inline constexpr CanObjectInfo<uint8_t> kJointAngleOffsetCompletedObjectInfo(
    0x3001, 2);

// Extended serial number
inline constexpr CanObjectInfo<uint64_t> kExtendedSerialNumberObjectInfo(0x3010,
                                                                         0);

// Torque constant (mNm/A)
inline constexpr CanObjectInfo<uint32_t> kTorqueConstantObjectInfo(0x3020, 0);

// Number of pole pairs
inline constexpr CanObjectInfo<uint16_t> kNumPolePairsObjectInfo(0x3021, 0);

// Phase to phase resistance (mOhm)
inline constexpr CanObjectInfo<uint32_t> kPhaseToPhaseResistanceObjectInfo(
    0x3022, 0);

// Phase ordering
inline constexpr CanObjectInfo<uint8_t> kPhaseOrderingObjectInfo(0x3023, 0);

// Bank swap and reboot (activate new firmware)
inline constexpr CanObjectInfo<uint32_t> kBankSwapObjectInfo(0x3030, 0);

// Bus voltage (V)
inline constexpr CanObjectInfo<float> kBusVoltageObjectInfo(0x3040, 0);

// Phase currents (A)
inline constexpr CanObjectInfo<float> kPhaseACurrentObjectInfo(0x3041, 1);
inline constexpr CanObjectInfo<float> kPhaseBCurrentObjectInfo(0x3041, 2);
inline constexpr CanObjectInfo<float> kPhaseCCurrentObjectInfo(0x3041, 3);

// Thermistors (C)
inline constexpr CanObjectInfo<float> kThermistor1ObjectInfo(0x3042, 1);
inline constexpr CanObjectInfo<float> kThermistor2ObjectInfo(0x3042, 2);
inline constexpr CanObjectInfo<float> kThermistor3ObjectInfo(0x3042, 3);
inline constexpr CanObjectInfo<float> kThermistor4ObjectInfo(0x3042, 4);

// Rotating frame currents (A)
inline constexpr CanObjectInfo<float> kQuadratureCurrentObjectInfo(0x3043, 1);
inline constexpr CanObjectInfo<float> kDirectCurrentObjectInfo(0x3043, 2);

// Encoder data (rad)
inline constexpr CanObjectInfo<float> kShaftAngleObjectInfo(0x3044, 1);
inline constexpr CanObjectInfo<float> kElectricalAngleObjectInfo(0x3044, 2);

// Rotating frame voltages (V)
inline constexpr CanObjectInfo<float> kQuadratureVoltageObjectInfo(0x3045, 1);
inline constexpr CanObjectInfo<float> kDirectVoltageObjectInfo(0x3045, 2);

// PWM duty cycles [0-1]
inline constexpr CanObjectInfo<float> kPhaseADutyCycleObjectInfo(0x3046, 1);
inline constexpr CanObjectInfo<float> kPhaseBDutyCycleObjectInfo(0x3046, 2);
inline constexpr CanObjectInfo<float> kPhaseCDutyCycleObjectInfo(0x3046, 3);

// Accelerometer readings [m/s^s]
inline constexpr CanObjectInfo<float> kLinearAccelerationXObjectInfo(0x3050, 1);
inline constexpr CanObjectInfo<float> kLinearAccelerationYObjectInfo(0x3050, 2);
inline constexpr CanObjectInfo<float> kLinearAccelerationZObjectInfo(0x3050, 3);

// Note: Reserve the 0x3050 - 0x305F range for IMU related data.

inline constexpr std::array<CanObjectInfoVariant, 30> kCustomCanObjects{
    kElectricalAngleOffsetObjectInfo,
    kElectricalAngleOffsetCompletedObjectInfo,
    kJointAngleOffsetObjectInfo,
    kJointAngleOffsetCompletedObjectInfo,
    kExtendedSerialNumberObjectInfo,
    kTorqueConstantObjectInfo,
    kNumPolePairsObjectInfo,
    kPhaseToPhaseResistanceObjectInfo,
    kPhaseOrderingObjectInfo,
    kBankSwapObjectInfo,
    kBusVoltageObjectInfo,
    kPhaseACurrentObjectInfo,
    kPhaseBCurrentObjectInfo,
    kPhaseCCurrentObjectInfo,
    kThermistor1ObjectInfo,
    kThermistor2ObjectInfo,
    kThermistor3ObjectInfo,
    kThermistor4ObjectInfo,
    kQuadratureCurrentObjectInfo,
    kDirectCurrentObjectInfo,
    kShaftAngleObjectInfo,
    kElectricalAngleObjectInfo,
    kQuadratureVoltageObjectInfo,
    kDirectVoltageObjectInfo,
    kPhaseADutyCycleObjectInfo,
    kPhaseBDutyCycleObjectInfo,
    kPhaseCDutyCycleObjectInfo,
    kLinearAccelerationXObjectInfo,
    kLinearAccelerationYObjectInfo,
    kLinearAccelerationZObjectInfo};

// These objects are untyped, so a CanObjectAddress is provided rather than a
// CanObjectInfo.
//
// The Rx and Tx terminology is from the point of view of the devices, so an Rx
// PDO sends data from the host to the device, and a Tx PDO sends data from the
// device to the host.
inline constexpr CanObjectAddress kRxPdoBlobAddress(0x2010, 0);
inline constexpr CanObjectAddress kTxPdoBlobAddress(0x2011, 0);
inline constexpr int kBinaryBlobNumBytes = 128;

}  // namespace barkour

#endif  // BARKOUR_ROBOT_HARDWARE_V1_ETHERCAT_CUSTOM_OBJECTS_H_
