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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MANUFACTURER_STATUS_REGISTER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MANUFACTURER_STATUS_REGISTER_H_

#include <cstdint>

namespace barkour {

// Defines the bit masks for the error codes used in the CANopen object
// `0x1002`, the "manufacturer status register". These are explained further in
// doc/error_reporting.md.
enum class ManufacturerStatusRegisterErrorCodes : uint32_t {
  kNone = 0x0,
  kElectricalAngleCalibrationFailed = 0x1,
  kJointAngleCalibrationFailed = 0x1 << 1,
  kElectricalAngleOffsetNotFound = 0x1 << 2,
  kJointAngleOffsetNotFound = 0x1 << 3,
  kInvalidMotorParametersSet = 0x1 << 4,
  kInvalidCurrentLimit = 0x1 << 5,
  kFailedToConfigureControllerComponent = 0x1 << 6,
  kFailedToGetSensorReadings = 0x1 << 7,
  // NOTE: kStoEnabled is deprecated, DO NOT USE.
  kStoEnabled = 0x1 << 8,
  kGateDriverStateChangeWhileVoltageApplied = 0x1 << 9,
  kEthercatNotOperationalWhileCia402Initialized = 0x1 << 10,
  kBusVoltageTooLow = 0x1 << 11,
  kOverheatDetected = 0x1 << 12,
  kStoLine1DownWhileVoltageApplied = 0x1 << 13,
  kStoLine2DownWhileVoltageApplied = 0x1 << 14,
};

// Bitwise operators
constexpr ManufacturerStatusRegisterErrorCodes operator|(
    ManufacturerStatusRegisterErrorCodes l,
    ManufacturerStatusRegisterErrorCodes r) {
  return ManufacturerStatusRegisterErrorCodes(static_cast<uint32_t>(l) |
                                              static_cast<uint32_t>(r));
}

constexpr ManufacturerStatusRegisterErrorCodes operator&(
    ManufacturerStatusRegisterErrorCodes l,
    ManufacturerStatusRegisterErrorCodes r) {
  return ManufacturerStatusRegisterErrorCodes(static_cast<uint32_t>(l) &
                                              static_cast<uint32_t>(r));
}
constexpr ManufacturerStatusRegisterErrorCodes operator^(
    ManufacturerStatusRegisterErrorCodes l,
    ManufacturerStatusRegisterErrorCodes r) {
  return ManufacturerStatusRegisterErrorCodes(static_cast<uint32_t>(l) ^
                                              static_cast<uint32_t>(r));
}
constexpr ManufacturerStatusRegisterErrorCodes operator~(
    ManufacturerStatusRegisterErrorCodes error_code) {
  return ManufacturerStatusRegisterErrorCodes(
      ~static_cast<uint32_t>(error_code));
}

constexpr ManufacturerStatusRegisterErrorCodes& operator|=(
    ManufacturerStatusRegisterErrorCodes& l,
    ManufacturerStatusRegisterErrorCodes r) {
  return l = l | r;
}

constexpr ManufacturerStatusRegisterErrorCodes& operator&=(
    ManufacturerStatusRegisterErrorCodes& l,
    ManufacturerStatusRegisterErrorCodes r) {
  return l = l & r;
}

constexpr ManufacturerStatusRegisterErrorCodes& operator^=(
    ManufacturerStatusRegisterErrorCodes& l,
    ManufacturerStatusRegisterErrorCodes r) {
  return l = l ^ r;
}

// Error codes which indicate / explain a CiA 402 fault condition.
inline constexpr ManufacturerStatusRegisterErrorCodes
    kManufacturerStatusRegisterCia402FaultMask =
        ManufacturerStatusRegisterErrorCodes::
            kGateDriverStateChangeWhileVoltageApplied |
        ManufacturerStatusRegisterErrorCodes::
            kEthercatNotOperationalWhileCia402Initialized |
        ManufacturerStatusRegisterErrorCodes::kBusVoltageTooLow |
        ManufacturerStatusRegisterErrorCodes::kOverheatDetected |
        ManufacturerStatusRegisterErrorCodes::kStoLine1DownWhileVoltageApplied |
        ManufacturerStatusRegisterErrorCodes::kStoLine2DownWhileVoltageApplied;

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MANUFACTURER_STATUS_REGISTER_H_
