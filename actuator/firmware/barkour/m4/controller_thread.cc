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

#include "actuator/firmware/barkour/m4/controller_thread.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string_view>

#include "actuator/firmware/barkour/common/adc_conversions.h"
#include "actuator/firmware/barkour/common/barkour_system.h"
#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/common/canopen_unit_conversions.h"
#include "actuator/firmware/barkour/common/commutation.h"
#include "actuator/firmware/barkour/common/derived_sensor_information.h"
#include "actuator/firmware/barkour/common/ds402_state_machine.h"
#include "actuator/firmware/barkour/common/interfaces/gate_driver_interface.h"
#include "actuator/firmware/barkour/common/interfaces/gpio_debug_interface.h"
#include "actuator/firmware/barkour/common/interfaces/imu_interface.h"
#include "actuator/firmware/barkour/common/manufacturer_status_register.h"
#include "actuator/firmware/barkour/common/motor_control_modules/control_module_manager.h"
#include "actuator/firmware/barkour/common/motor_control_modules/current_control_module.h"
#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/motor_control_modules/zero_output_control_module.h"
#include "actuator/firmware/barkour/common/phase_order.h"
#include "actuator/firmware/barkour/common/read_sensors.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "actuator/firmware/barkour/common/simple_lowpass_iir_filter.h"
#include "actuator/firmware/barkour/common/thermal_monitor.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/adc.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/gate_driver.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/realtime_foc.h"
#include "actuator/firmware/barkour/m4/ds402_drive.h"
#include "actuator/firmware/barkour/m4/ecat/utypes.h"
#include "actuator/firmware/barkour/m4/ecat_device.h"
#include "actuator/firmware/barkour/devices/imu/lsm6ds0/lsm6ds0_imu.h"
#include "actuator/firmware/barkour/devices/imu/lsm9ds1/lsm9ds1_imu.h"
#include "pw_assert/check.h"
#include "pw_chrono/system_clock.h"
#include "pw_containers/flat_map.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"
#include "pw_sync/mutex.h"
#include "pw_sync/thread_notification.h"
#include "pw_thread/sleep.h"

namespace barkour {

namespace {

struct Ds402ControlCommands {
  int16_t torque_setpoint;
  uint16_t controlword;
};

#if MOTOR_TYPE == MOTOR_AK80_9
constexpr float kVelocityFilterCutoffFreq = 200.0f;  // Hz.

// R = 0.22/2 ohm
// L = 57uH = 0.000057/2
// Loop BW = 10Khz = 62832 rad/sec
// Series PID Kps = L * Loop BW/20
// Series PID Kis = R/L
//
// Parallel Kpp = Kps
// Parallel Kip = Kis * Kpp
constexpr float kCurrentControlPGain = 0.089535f;  // Kp for parallel controller
constexpr float kCurrentControlIGain = 345.58f;    // Ki for parallel controller
constexpr float kCurrentControlAntiWindGain = 0.0f;
constexpr float kPhaseInductance = 57e-6f / 2.0f;

#else

#error "No other motor_type supported"
constexpr float kVelocityFilterCutoffFreq = 200.0f;  // Hz
constexpr float kCurrentControlPGain = 0.0f;
constexpr float kCurrentControlIGain = 0.0f;
constexpr float kCurrentControlAntiWindGain = 0.0f;
constexpr float kPhaseInductance = 0.0f

#endif

// Electrical angle calibration constants.
constexpr float kElectricalAngleCalibrationQVoltage = 1.0f;  // Volts.
constexpr float kElectricalAngleCalibrationStabilizationPeriod =
    3.0f;  // Seconds.

// Joint angle calibration constants.
constexpr uint8_t kJointAngleCalibrationNumEncoderSamples = 100;

constexpr uint8_t kGateDriverStartupTimeoutCycles = 100;
constexpr uint8_t kGateDriverShutdownTimeoutCycles = 100;

constexpr float kOverheatTemperatureThreshold = 90.0f;  // Celsius.
constexpr float kOverheatDurationThreshold = 1.0f;      // Seconds.
constexpr float kOverheatSaturationDuration = 3.0f;     // Seconds.

constexpr float kMinimumBusVoltage = 18.0f;  // Volts.

constexpr uint8_t kMaxConsecutiveEncoderDataLossErrors = 2;
constexpr uint8_t kInitialGoodSensorReadingRetries = 5;

// The difference in Volts between the maximum theoretical sinusoidal voltage
// which can be applied (i.e. half the measured supply voltage at the start of
// current control), and the saturation point of the PID controllers which
// produce sinusoidal voltages.
constexpr float kSaturationVoltageMargin = 1.0f;

// A general-purpose delay, for hanging around in non-critical timing
// situations.
constexpr uint8_t kGeneralPurposeDelayMs = 20;

// Define some default sensor readings to send over EtherCAT, in the case that
// we fail to read the sensors, for some reason.
constexpr SensorReadings kDefaultReportedSensorReadings{
    .raw_encoder_count = 0,
    .encoder_position = 0,
    .electrical_angle = 0,
    .quadrature_current = 0,
    .direct_current = 0,
    .phase_a_current = 0,
    .phase_b_current = 0,
    .phase_c_current = 0,
    .bus_voltage = 0,
    .thermistor_1 = 0,
    .thermistor_2 = 0,
    .thermistor_3 = 0,
    .thermistor_4 = 0,
    .sto_24v_safe_adc_voltage = 0,
    .imu_state = {{0, 0, 0}, {0, 0, 0}}};

constexpr SensorDerivedQuantities kDefaultReportedDerivedSensorInfo{
    .shaft_position = 0,
    .shaft_velocity = 0,
};

static Lsm6Ds0Imu lsm6ds0_imu;
static Lsm9Ds1Imu lsm9ds1_imu;

Ds402ControlCommands ReadDs402CommandsFromObjDict(pw::sync::Mutex& obj_mutex) {
  Ds402ControlCommands commands{0, 0};

  obj_mutex.lock();
  commands.torque_setpoint = Obj.rx.torque;
  commands.controlword = Obj.rx.controlword;
  obj_mutex.unlock();

  return commands;
}

void CopyStateToObjDictTxValues(
    pw::sync::Mutex& obj_mutex,
    ManufacturerStatusRegisterErrorCodes manufacturer_status_register,
    GateDriverStatusRegisters gate_driver_status_registers, uint16_t statusword,
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& derived_sensor_readings,
    const MotorParameters& motor_parameters,
    const RotatingFrameVoltages& rotating_frame_voltage_commands,
    const ThreePhasePwmCommands& phase_pwm_commands, bool fb_sto1_state,
    bool fb_sto2_state) {
  obj_mutex.lock();

  Obj.tx.manufacturer_status =
      static_cast<uint32_t>(manufacturer_status_register);
  Obj.tx.statusword = statusword;
  Obj.tx.position = derived_sensor_readings.shaft_position;
  Obj.tx.velocity = derived_sensor_readings.shaft_velocity;

  // Send zeros in the case that we fail to convert the values correctly.
  Obj.tx.torque = CurrentToCia402Torque(sensor_readings.quadrature_current,
                                        motor_parameters.torque_constant,
                                        motor_parameters.rated_torque)
                      .value_or(0);

  Obj.tx.motor_current =
      CurrentToCia402Current(sensor_readings.quadrature_current,
                             motor_parameters.rated_current)
          .value_or(0);

  Obj.tx.bus_voltage = sensor_readings.bus_voltage;
  Obj.tx.phase_a_current = sensor_readings.phase_a_current;
  Obj.tx.phase_b_current = sensor_readings.phase_b_current;
  Obj.tx.phase_c_current = sensor_readings.phase_c_current;
  Obj.tx.shaft_angle = 0;  // Not relevant anymore.
  Obj.tx.electrical_angle = sensor_readings.electrical_angle;
  Obj.tx.quadrature_current = sensor_readings.quadrature_current;
  Obj.tx.direct_current = sensor_readings.direct_current;
  Obj.tx.quadrature_voltage =
      rotating_frame_voltage_commands.quadrature_voltage;
  Obj.tx.direct_voltage = rotating_frame_voltage_commands.direct_voltage;
  Obj.tx.phase_a_duty_cycle = phase_pwm_commands.phase_a;
  Obj.tx.phase_b_duty_cycle = phase_pwm_commands.phase_b;
  Obj.tx.phase_c_duty_cycle = phase_pwm_commands.phase_c;
  Obj.tx.thermistor_1 = sensor_readings.thermistor_1;
  Obj.tx.thermistor_2 = sensor_readings.thermistor_2;
  Obj.tx.thermistor_3 = sensor_readings.thermistor_3;
  Obj.tx.thermistor_4 = sensor_readings.thermistor_4;

  Obj.tx.fb_sto1_state = static_cast<uint8_t>(fb_sto1_state);
  Obj.tx.fb_sto2_state = static_cast<uint8_t>(fb_sto2_state);
  Obj.tx.sto_24v_safe_adc_voltage = sensor_readings.sto_24v_safe_adc_voltage;

  Obj.tx.gate_driver_fault_status_1 =
      gate_driver_status_registers.fault_register_1;
  Obj.tx.gate_driver_fault_status_2 =
      gate_driver_status_registers.fault_register_2;

  Obj.tx.acceleration_x = sensor_readings.imu_state.accelerometer[0];
  Obj.tx.acceleration_y = sensor_readings.imu_state.accelerometer[1];
  Obj.tx.acceleration_z = sensor_readings.imu_state.accelerometer[2];

  // The blob contains:
  //  float accelerometer[3];
  //  float gyro[3];
  //  float sto_24v_safe_adc_voltage;
  //  uint16 gate_driver_status_registers.fault_register_1;
  //  uint16 gate_driver_status_registers.fault_register_2;
  //  uint16 actual FB_STO1/2 values (raw read);

  // Put full imu_state data at start of blob.
  float* ptr = reinterpret_cast<float*>(&Obj.tx.blob[0]);

  for (uint8_t i = 0; i < 3; i++) {
    ptr[i] = sensor_readings.imu_state.accelerometer[i];
    ptr[i + 3] = sensor_readings.imu_state.gyroscope[i];
  }

  ptr[6] = sensor_readings.sto_24v_safe_adc_voltage;
  uint16_t* uint16_blob_ptr = reinterpret_cast<uint16_t*>(&ptr[7]);
  uint16_blob_ptr[0] = gate_driver_status_registers.fault_register_1;
  uint16_blob_ptr[1] = gate_driver_status_registers.fault_register_2;

  uint16_blob_ptr[2] = 0x0000;
  uint16_blob_ptr[2] |=
      (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4) == GPIO_PIN_RESET) ? 0b01 : 0;
  uint16_blob_ptr[2] |=
      (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) == GPIO_PIN_RESET) ? 0b10 : 0;

  obj_mutex.unlock();
}

struct CalibrationOffsets {
  std::optional<uint32_t> electrical_angle_offset;
  std::optional<uint32_t> joint_angle_offset;

  pw::Result<uint32_t> GetElectricalAngleOffset() const {
    return electrical_angle_offset.has_value()
               ? pw::Result<uint32_t>(electrical_angle_offset.value())
               : pw::Result<uint32_t>(pw::Status::NotFound());
  }

  pw::Result<uint32_t> GetJointAngleOffset() const {
    return joint_angle_offset.has_value()
               ? pw::Result<uint32_t>(joint_angle_offset.value())
               : pw::Result<uint32_t>(pw::Status::NotFound());
  }
};

CalibrationOffsets LoadCalibrationOffsetsfromObjDict(
    pw::sync::Mutex& obj_mutex) {
  obj_mutex.lock();
  int32_t electrical_angle_offset = Obj.SDO3000.electrical_angle_offset;
  bool electrical_angle_calibration_completed =
      Obj.SDO3000.electrical_angle_calibration_completed == 1;
  int32_t joint_angle_offset = Obj.SDO3001.joint_angle_offset;
  bool joint_angle_calibration_completed =
      Obj.SDO3001.joint_angle_calibration_completed == 1;
  obj_mutex.unlock();

  return {
      .electrical_angle_offset =
          electrical_angle_calibration_completed
              ? std::optional<uint32_t>(electrical_angle_offset)
              : std::nullopt,
      .joint_angle_offset = joint_angle_calibration_completed
                                ? std::optional<uint32_t>(joint_angle_offset)
                                : std::nullopt,
  };
}

// Functions to hang forever, and set the "manufacturer status register" object
// value, if an error is observed that we can't continue from. This is preferred
// to exiting from the ControllerTask::Run function, which will result in a
// FreeRTOS fault that can stop other tasks from running.
//
// Returns the value held by the result, or hangs forever.
template <typename T>
T GetValueOrHangAndSetStatusRegister(
    pw::Result<T>&& maybe_value, const char* error_message,
    ManufacturerStatusRegisterErrorCodes status_register) {
  if (maybe_value.ok()) {
    return maybe_value.value();
  } else {
    PW_LOG_ERROR("%s, stopping controller task.", error_message);

    ObjMutex.lock();
    Obj.tx.manufacturer_status = static_cast<uint32_t>(status_register);
    ObjMutex.unlock();

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(kGeneralPurposeDelayMs));
    }
  }
}

// Hangs forever if the status is not OK.
void CheckOkOrHangAndSetStatusRegister(
    pw::Status status, const char* error_message,
    ManufacturerStatusRegisterErrorCodes status_register) {
  if (!status.ok()) {
    PW_LOG_ERROR("%s, stopping controller task.", error_message);

    ObjMutex.lock();
    Obj.tx.manufacturer_status = static_cast<uint32_t>(status_register);
    ObjMutex.unlock();

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(kGeneralPurposeDelayMs));
    }
  }
}

// Returns true iff the passed motor parameters are all valid.
pw::Status CheckMotorParametersValid(const MotorParameters& params) {
  if (params.rated_current > 0 && params.rated_torque > 0 &&
      params.torque_constant > 0 && params.num_pole_pairs > 0 &&
      (params.phase_order == kAbcPhaseOrder ||
       params.phase_order == kAcbPhaseOrder) &&
      params.phase_to_phase_resistance > 0 && params.encoder_increments > 0 &&
      params.encoder_motor_revolutions > 0 &&
      params.gear_motor_revolutions > 0 && params.gear_shaft_revolutions > 0) {
    return pw::OkStatus();
  } else {
    return pw::Status::InvalidArgument();
  }
}

}  // namespace.

ControllerThread::ControllerThread(
    barkour::RotaryEncoder& encoder,
    pw::sync::ThreadNotification& controller_notification,
    float controller_notification_period)
    : controller_notification_period_(controller_notification_period),
      adc_(Adc::Get()),
      board_(BoardConfig::Get()),
      encoder_(encoder),
      foc_(nullptr),
      gate_driver_(GateDriver::Get()),
      ecat_device_(EthercatDevice::Get()),
      controller_notification_(controller_notification),
      manufacturer_status_register_{0u} {}

void ControllerThread::Run() {
  // Run calibration routines if needed.
  if (board_.DebugButtonEnabled()) {
    CheckOkOrHangAndSetStatusRegister(
        CalibrateElectricalAngleAndRestartEthercat(),
        "Electrical angle calibration routine failed",
        manufacturer_status_register_ | ManufacturerStatusRegisterErrorCodes::
                                            kElectricalAngleCalibrationFailed);

    if (board_.DebugButtonEnabled()) {
      PW_LOG_WARN(
          "Electrical angle calibration completed, no need to keep the button "
          "pressed!");
    }

    board_.Blinky1SetIOMode(false);  // Got no error so Reset Blinky1 to input.

  } else if (board_.Blinky1ButtonEnabled()) {
    CheckOkOrHangAndSetStatusRegister(
        CalibrateJointAngleAndRestartEthercat(),
        "Joint angle calibration routine failed",
        manufacturer_status_register_ |
            ManufacturerStatusRegisterErrorCodes::kJointAngleCalibrationFailed);
    if (board_.Blinky1ButtonEnabled()) {
      PW_LOG_WARN(
          "Joint angle calibration completed, no need to keep the button "
          "pressed!");
      manufacturer_status_register_ |=
          ManufacturerStatusRegisterErrorCodes::kJointAngleOffsetNotFound;
    }
  }

  // Wait until the state machine has entered PREOP, SAFEOP or OP state,
  // i.e., it has completed the INIT->PREOP transition which involves reading
  // from EEPROM.
  while (true) {
    EthercatApplicationLayerState current_ecat_state =
        ecat_device_.GetCurrentState();
    if (current_ecat_state == EthercatApplicationLayerState::kPreop ||
        current_ecat_state == EthercatApplicationLayerState::kSafeop ||
        current_ecat_state == EthercatApplicationLayerState::kOp) {
      break;
    }
    controller_notification_.acquire();
  }

  // Copies the motor parameters now that the device has been configured.
  ObjMutex.lock();
  MotorParameters motor_parameters = Obj.parameter;
  ObjMutex.unlock();

  CheckOkOrHangAndSetStatusRegister(
      CheckMotorParametersValid(motor_parameters),
      "Received invalid motor parameters",
      manufacturer_status_register_ |
          ManufacturerStatusRegisterErrorCodes::kInvalidMotorParametersSet);

  PhaseOrder phase_order = (motor_parameters.phase_order == kAcbPhaseOrder)
                               ? PhaseOrder::kAcb
                               : PhaseOrder::kAbc;

  // Copy the electrical angle calibration variables from those loaded from
  // EEPROM.
  CalibrationOffsets calibration_offsets =
      LoadCalibrationOffsetsfromObjDict(ObjMutex);

  uint32_t electrical_angle_offset = GetValueOrHangAndSetStatusRegister(
      calibration_offsets.GetElectricalAngleOffset(),
      "No electrical angle offset found, stopping controller task.",
      manufacturer_status_register_ |
          ManufacturerStatusRegisterErrorCodes::kElectricalAngleOffsetNotFound);

  uint32_t joint_angle_offset = 0;
  if (pw::Result<uint32_t> maybe_joint_offset =
          calibration_offsets.GetJointAngleOffset();
      maybe_joint_offset.ok()) {
    joint_angle_offset = maybe_joint_offset.value();
  } else {
    PW_LOG_INFO("No joint angle offset found, using default value of 0.");
    manufacturer_status_register_ |=
        ManufacturerStatusRegisterErrorCodes::kJointAngleOffsetNotFound;
  }

  pw::Result<float> maybe_current_limit = Cia402CurrentToCurrent(
      motor_parameters.current_limit, motor_parameters.rated_current);
  if (!maybe_current_limit.ok()) {
    PW_LOG_WARN("Invalid current limit %u, ignoring value.",
                motor_parameters.current_limit);
    manufacturer_status_register_ |=
        ManufacturerStatusRegisterErrorCodes::kInvalidCurrentLimit;
  }

  pw::Result<Foc*> foc = Foc::Build(
      Foc::kFocDt, kCurrentControlPGain, kCurrentControlIGain,
      kCurrentControlAntiWindGain, kSaturationVoltageMargin,
      1e-3f * static_cast<float>(motor_parameters.phase_to_phase_resistance) /
          2.0f,
      kPhaseInductance, encoder_, gate_driver_, phase_order,
      motor_parameters.num_pole_pairs, electrical_angle_offset);

  if (!foc.ok()) {
    PW_CRASH("Failed to create Foc Object.");
  }
  foc_ = foc.value();

  // Identify which IMU is connected by attempting to initialize both.
  ImuInterface* imu_interface = nullptr;
  if (lsm9ds1_imu.Initialize().ok()) {
    imu_interface = reinterpret_cast<ImuInterface*>(&lsm9ds1_imu);
  } else {
    lsm9ds1_imu.Deinitialize();
    if (lsm6ds0_imu.Initialize().ok()) {
      imu_interface = reinterpret_cast<ImuInterface*>(&lsm6ds0_imu);
    } else {
      lsm6ds0_imu.Deinitialize();
      PW_CRASH("Failed to initialize IMU");
    }
  }

  SensorReader sensor_reader(adc_, encoder_, *foc_, *imu_interface,
                             kMaxConsecutiveEncoderDataLossErrors);

  // Instantiate and configure the control primitives.
  SimpleLowpassIirFilter velocity_filter(kVelocityFilterCutoffFreq,
                                         controller_notification_period_);

  CurrentControlModule current_control_module =
      GetValueOrHangAndSetStatusRegister(
          CurrentControlModule::Build(
              maybe_current_limit.value_or(
                  std::numeric_limits<float>::infinity()),
              *foc.value()),
          "Failed to configure current controller",
          manufacturer_status_register_ |
              ManufacturerStatusRegisterErrorCodes::
                  kFailedToConfigureControllerComponent);

  ZeroOutputControlModule braking_control_module(gate_driver_);

  pw::containers::FlatMap<Ds402Drive::SupportedControlModes,
                          MotorControlModule*,
                          Ds402Drive::kNumSupportedControlModes>
      motor_control_modules({{
          {Ds402Drive::SupportedControlModes::kZeroPwmOutput,
           &braking_control_module},
          {Ds402Drive::SupportedControlModes::kCurrentControl,
           &current_control_module},
      }});

  ControlModuleManager control_module_manager =
      GetValueOrHangAndSetStatusRegister(
          ControlModuleManager<Ds402Drive::SupportedControlModes,
                               Ds402Drive::kNumSupportedControlModes>::
              Build(motor_control_modules),
          "Failed to build control module manager",
          manufacturer_status_register_ |
              ManufacturerStatusRegisterErrorCodes::
                  kFailedToConfigureControllerComponent);

  // Set up DS402 drive and state machine.
  Ds402Drive ds402_drive(gate_driver_, control_module_manager);
  Ds402StateMachine ds402_state_machine(ds402_drive);

  // Set up thermal monitor.
  uint16_t thermal_overheat_count_threshold = static_cast<uint16_t>(
      kOverheatDurationThreshold / controller_notification_period_);
  uint16_t thermal_overheat_count_saturation = static_cast<uint16_t>(
      kOverheatSaturationDuration / controller_notification_period_);

  ThermalMonitor<kNumTemperatureReadingsUsed> thermal_monitor =
      GetValueOrHangAndSetStatusRegister(
          ThermalMonitor<kNumTemperatureReadingsUsed>::Build(
              kOverheatTemperatureThreshold, thermal_overheat_count_threshold,
              thermal_overheat_count_saturation),
          "Failed to build thermal monitor",
          manufacturer_status_register_ |
              ManufacturerStatusRegisterErrorCodes::
                  kFailedToConfigureControllerComponent);

  DerivedSensorInformationUpdater derived_sensor_info_updater =
      GetValueOrHangAndSetStatusRegister(
          DerivedSensorInformationUpdater::Build(
              velocity_filter, motor_parameters.encoder_increments,
              joint_angle_offset, controller_notification_period_),
          "Failed to build derived sensor information updater",
          manufacturer_status_register_ |
              ManufacturerStatusRegisterErrorCodes::
                  kFailedToConfigureControllerComponent);

  derived_sensor_info_updater.Reset();

  // Get a first set of sensor information, so that we always have a "last good"
  // reading to fall back on. Try a few times, in case that the first encoder
  // reading is bad.
  pw::Result<SensorReadings> maybe_initial_sensor_readings =
      pw::Status::Unknown();
  for (uint8_t i = 0; i < kInitialGoodSensorReadingRetries; ++i) {
    (void)encoder_.Update();
    maybe_initial_sensor_readings =
        sensor_reader.GetLatestReadingsFromAllSensors();
    if (maybe_initial_sensor_readings.ok()) {
      break;
    }
  }

  CheckOkOrHangAndSetStatusRegister(
      maybe_initial_sensor_readings.status(),
      "Failed to get initial sensor readings",
      manufacturer_status_register_ |
          ManufacturerStatusRegisterErrorCodes::kFailedToGetSensorReadings);

  SensorReadings last_good_sensor_readings =
      maybe_initial_sensor_readings.value();

  SensorDerivedQuantities last_good_derived_sensor_readings =
      GetValueOrHangAndSetStatusRegister(
          derived_sensor_info_updater.GetDerivedSensorInformation(
              last_good_sensor_readings),
          "Failed to get initial derived sensor information",
          manufacturer_status_register_ |
              ManufacturerStatusRegisterErrorCodes::kFailedToGetSensorReadings);

  // --- Main control loop ---
  while (true) {
    // --- Read sensors ---

    pw::Result<SensorReadings> maybe_sensor_readings =
        sensor_reader.GetLatestReadingsFromAllSensors();
    pw::Result<SensorDerivedQuantities> maybe_derived_sensor_readings =
        maybe_sensor_readings.ok()
            ? derived_sensor_info_updater.GetDerivedSensorInformation(
                  maybe_sensor_readings.value())
            : pw::Status::FailedPrecondition();

    // If sensor reading or derived info updating failed, use the last good
    // readings. Cache the statuses so we can still report this error and
    // trigger a fault, if needed.
    pw::Status sensor_reading_status = maybe_sensor_readings.status();
    pw::Status derived_sensor_reading_status =
        maybe_derived_sensor_readings.status();

    SensorReadings sensor_readings =
        maybe_sensor_readings.value_or(last_good_sensor_readings);
    SensorDerivedQuantities derived_sensor_readings =
        maybe_derived_sensor_readings.value_or(
            last_good_derived_sensor_readings);

    // Update last good readings.
    if (sensor_reading_status.ok()) {
      last_good_sensor_readings = maybe_sensor_readings.value();
    }

    if (derived_sensor_reading_status.ok()) {
      last_good_derived_sensor_readings = maybe_derived_sensor_readings.value();
    }

    // Update overheat monitor, using current or last good sensor readings.
#if (MOTOR_TYPE == MOTOR_AK80_9)
    thermal_monitor.Update({sensor_readings.thermistor_4});
#else
#error "Unknown motor type. Update controller_thread.cc"
#endif

    // --- Get control inputs ---
    Ds402ControlCommands control_commands =
        ReadDs402CommandsFromObjDict(ObjMutex);

    // Clip according to torque limit.
    int16_t torque_setpoint = std::clamp<int16_t>(
        control_commands.torque_setpoint, -motor_parameters.torque_limit,
        motor_parameters.torque_limit);

    // Motor parameters have been verified, so this can't fail.
    float reference_current =
        Cia402TorqueToCurrent(torque_setpoint, motor_parameters.torque_constant,
                              motor_parameters.rated_torque)
            .value();

    // --- Handle safety conditions which occur outside of DS402 ---

    bool error_for_ds402 = false;

    Ds402State ds402_state = ds402_state_machine.GetState();
    bool in_fault_state = ds402_state == Ds402State::FaultReactionActive() ||
                          ds402_state == Ds402State::Fault();

    bool ecat_not_in_op_while_ds402_initialized =
        (!in_fault_state && ds402_state != Ds402State::NotReadyToSwitchOn() &&
         ds402_state != Ds402State::SwitchOnDisabled() &&
         ecat_device_.GetCurrentState() != EthercatApplicationLayerState::kOp);

    if (ecat_not_in_op_while_ds402_initialized) {
      error_for_ds402 = true;
      // Only log if we're not in a fault state, to avoid repeating the message
      // too much in the log output.
      if (!in_fault_state) {
        PW_LOG_ERROR(
            "EtherCAT not in OP state while DS402 in an active state. "
            "Triggering DS402 fault.");
      }
      manufacturer_status_register_ |= ManufacturerStatusRegisterErrorCodes::
          kEthercatNotOperationalWhileCia402Initialized;
    }

    if (!sensor_reading_status.ok()) {
      error_for_ds402 = true;
      if (!in_fault_state) {
        PW_LOG_ERROR(
            "Failed to read data from sensors, status: %s. Triggering DS402 "
            "fault.",
            sensor_reading_status.str());
      }
      manufacturer_status_register_ |=
          ManufacturerStatusRegisterErrorCodes::kFailedToGetSensorReadings;
    }

    if (!derived_sensor_reading_status.ok()) {
      error_for_ds402 = true;
      if (!in_fault_state) {
        PW_LOG_ERROR(
            "Failed to update derived data from sensors, status: %s. "
            "Triggering DS402 fault.",
            derived_sensor_reading_status.str());
      }
      manufacturer_status_register_ |=
          ManufacturerStatusRegisterErrorCodes::kFailedToGetSensorReadings;
    }

    if (sensor_readings.bus_voltage <= kMinimumBusVoltage) {
      error_for_ds402 = true;
      if (!in_fault_state) {
        PW_LOG_ERROR(
            "Bus voltage too low. Bus voltage: %f, required minimum voltage: "
            "%f. Triggering DS402 fault.",
            sensor_readings.bus_voltage, kMinimumBusVoltage);
      }
      manufacturer_status_register_ |=
          ManufacturerStatusRegisterErrorCodes::kBusVoltageTooLow;
    }

    if (thermal_monitor.Overheated()) {
      error_for_ds402 = true;
      if (!in_fault_state) {
        PW_LOG_ERROR("Observed an overheat condition, triggering DS402 fault.");
      }
      manufacturer_status_register_ |=
          ManufacturerStatusRegisterErrorCodes::kOverheatDetected;
    }

    // --- Step the DS402 state machine ---
    //
    // This will call motor control logic and send the PWM commands to the gate
    // driver.
    ds402_drive.SetExternalError(error_for_ds402);

    // Set the last good sensor readings, even in the case of an error in the
    // readings. This would allow e.g., the "fault reaction" DS402 state handler
    // to have some recent information it could use to decide on a shutdown
    // strategy.
    ds402_drive.SetSensorReadings(sensor_readings, derived_sensor_readings);

    ds402_drive.SetControlReferences({.reference_current = reference_current});

    ds402_state_machine.SetControlWord(control_commands.controlword);
    ds402_state_machine.Update(manufacturer_status_register_,
                               gate_driver_status_registers_);

    // --- Write Tx data over EtherCAT ---
    RotatingFrameVoltages rotating_frame_voltage_commands =
        ds402_drive.GetRotatingFrameVoltages().value_or(
            RotatingFrameVoltages{0, 0});
    ThreePhasePwmCommands phase_pwm_commands =
        ds402_drive.GetPhasePwmCommands().value_or(
            ThreePhasePwmCommands{0, 0, 0});

    CopyStateToObjDictTxValues(
        ObjMutex, manufacturer_status_register_, gate_driver_status_registers_,
        ds402_state_machine.GetStatusWord(), sensor_readings,
        derived_sensor_readings, motor_parameters,
        rotating_frame_voltage_commands, phase_pwm_commands,
        gate_driver_.VoltageOnStoLine1(), gate_driver_.VoltageOnStoLine2());

    // --- Wait for timer to signal next iteration. ---
    controller_notification_.acquire();
  }
}

//===============================================================================================
pw::Status ControllerThread::CalibrateElectricalAngleAndRestartEthercat() {
  PW_LOG_INFO("Running electrical angle calibration.");

  // Set debug IOs to outputs to use as Led indicators.
  board_.Blinky1SetIOMode(true);
  board_.Blinky2SetIOMode(true);

  // Clear Blinky1 Led, (Red Led on Matraic) to indicate no error
  board_.DisableBlinky1Led();

  // Set Blinky2 Led (Yellow Led on Matraic) to indicate performing calibration.
  board_.EnableBlinky2Led();

  uint16_t stabilization_cycles_to_run =
      static_cast<uint16_t>(kElectricalAngleCalibrationStabilizationPeriod /
                            controller_notification_period_);

  // Make sure that the gate driver us up and running before we run the
  // calibration.
  gate_driver_.SetTargetState(barkour::GateDriverState::PowerOnGateEnabled());

  for (uint8_t i = 0; i < kGateDriverStartupTimeoutCycles; ++i) {
    gate_driver_.Update();
    if (gate_driver_.CurrentState() ==
        barkour::GateDriverState::PowerOnGateEnabled()) {
      break;
    }
    controller_notification_.acquire();
  }

  if (gate_driver_.CurrentState() !=
      barkour::GateDriverState::PowerOnGateEnabled()) {
    PW_LOG_ERROR("Failed to start up the gate driver after %d attempts.",
                 kGateDriverStartupTimeoutCycles);

    board_.EnableBlinky1Led();   // Set Blinky1 Led, (Red Led on Matraic) to
                                 // indicate error
    board_.DisableBlinky2Led();  // Clear Blinky2 Led, (Yellow Led on Matraic)
                                 // to indicate calibration ended.
    return pw::Status::Aborted();
  }

  while (gate_driver_.CurrentState() !=
         barkour::GateDriverState::PowerOnGateEnabled()) {
    gate_driver_.Update();
    controller_notification_.acquire();
  }

  class ShutdownGateDriverGuard {
   public:
    ShutdownGateDriverGuard(GateDriverInterface& gate_driver)
        : gate_driver_(gate_driver), counter_(0) {}

    ~ShutdownGateDriverGuard() {
      gate_driver_.SetTargetState(barkour::GateDriverState::PowerOff());
      while (gate_driver_.CurrentState() !=
             barkour::GateDriverState::PowerOff()) {
        gate_driver_.Update();
        if (++counter_ > kGateDriverShutdownTimeoutCycles) {
          PW_LOG_ERROR("Failed to shut down the gate driver after %d attempts.",
                       kGateDriverShutdownTimeoutCycles);
          break;
        }
        // Zero-length sleep, to yield.
        pw::this_thread::sleep_for(pw::chrono::SystemClock::duration::zero());
      }
    }

   private:
    GateDriverInterface& gate_driver_;
    uint8_t counter_;
  };

  std::optional<ShutdownGateDriverGuard> shutdown_guard(gate_driver_);

  // Read the encoder initial state
  PW_TRY_ASSIGN(RotaryEncoderState encoder_initial_state, encoder_.Update());

  // to save encoder count when motor @ 90 Deg.
  RotaryEncoderState encoder_state_90;
  // to save encoder count when motor @ 180 Deg.
  RotaryEncoderState encoder_state_180;

  // Move motor to 180 Deg, then 90 Deg electrical angles.
  for (uint8_t p = 0; p < 2; p++) {
    uint16_t cycle_counter = 0;

    while (++cycle_counter <= stabilization_cycles_to_run) {
      PW_TRY_ASSIGN(int32_t raw_value,
                    adc_.GetSample(AdcAnalogSignal::kBusVoltage));
      PW_TRY_ASSIGN(float bus_voltage,
                    AdcReadingToFloat(AdcAnalogSignal::kBusVoltage, raw_value));

      if (bus_voltage <= kMinimumBusVoltage) {
        PW_LOG_ERROR(
            "Bus voltage too low. Bus voltage: %f, required minimum voltage: "
            "%f.",
            bus_voltage, kMinimumBusVoltage);

        // Set Blinky1 Led, (Red Led on Matraic) to indicate error.
        board_.EnableBlinky1Led();
        // Clear Blinky2 Led, (Yellow Led on Matraic) to indicate calibration
        // ended.
        board_.DisableBlinky2Led();
        return pw::Status::Aborted();
      }

      constexpr float setVoltage =
          15.0f *        // Target current.
          2.0 * 0.106f;  // Phase resistance.
                         // Volts = Desired amps (10.0A) * 2*phase ohms =
                         // Required voltage.

      if (p == 0) {  // U0, V+, W-   == align to 180 deg
        PW_TRY(gate_driver_.SetDutyCycles(
            -1.0f * (setVoltage / bus_voltage) / 2.0f + 0.5f,
            0.5 * (setVoltage / bus_voltage) / 2.0f + 0.5f,
            0.5 * (setVoltage / bus_voltage) / 2.0f + 0.5f));
      }

      if (p == 1) {  // U-, V+, W+  == align 90 Deg
        // Phase b/c voltages need to match PhaseOrder.
        // (assumed for now its swapped)
        // Phase order is not available when this is run. See Also b/226296767

        PhaseOrder phase_order = PhaseOrder::kAcb;

        if (phase_order == PhaseOrder::kAcb) {
          PW_TRY(gate_driver_.SetDutyCycles(
              0.0f * (setVoltage / bus_voltage) / 2.0f + 0.5f,
              -0.866f * (setVoltage / bus_voltage) / 2.0f + 0.5f,
              0.866f * (setVoltage / bus_voltage) / 2.0f + 0.5f));
        } else {
          PW_TRY(gate_driver_.SetDutyCycles(
              0.0f * (setVoltage / bus_voltage) / 2.0f + 0.5f,
              0.866f * (setVoltage / bus_voltage) / 2.0f + 0.5f,
              -0.866f * (setVoltage / bus_voltage) / 2.0f + 0.5f));
        }
      }
      controller_notification_.acquire();
      RotaryEncoderState encoder_state = encoder_.Update().value();
      (void)encoder_state;
    }

    if (p == 0) {
      // Read the encoder to get the offset.
      encoder_state_180 = encoder_.Update().value();
    }

    if (p == 1) {
      // Read the encoder to get the offset.
      encoder_state_90 = encoder_.Update().value();
    }
  }

  // Destroy the shutdown guard, which shuts down the gate driver.
  shutdown_guard = std::nullopt;

  // Double-check the gate driver has shut down properly in the guard, if not
  // then return an error.
  if (gate_driver_.CurrentState() != barkour::GateDriverState::PowerOff()) {
    board_.EnableBlinky1Led();   // Set Blinky1 Led, (Red Led on Matraic) to
                                 // indicate error
    board_.DisableBlinky2Led();  // Clear Blinky2 Led, (Yellow Led on Matraic)
                                 // to indicate calibration ended.
    return pw::Status::Internal();
  }

  // num_pole_pairs is not available when this is run. See Also b/226296767
  uint32_t num_pole_pairs = 21;  // Obj.parameter.num_pole_pairs

  // Calculate how much the motor actually moved between 90 Deg and 180 Deg
  // positions.
  int32_t count_delta = encoder_state_90.raw_encoder_counts -
                        encoder_state_180.raw_encoder_counts;
  int32_t counts_per_elect_turn = encoder_.GetCountsPerTurn() / num_pole_pairs;

  float dist_turned = 360.0f * static_cast<float>(count_delta) /
                      static_cast<float>(counts_per_elect_turn);

  PW_LOG_DEBUG(
      "Encoder Electrical Calibration Completed: ENC Init=%lu, ENC @ 180 "
      "Deg=%lu, ENC @ 90 Deg=%lu, Delta=%ld, Angle Turned=%5.2f Deg.",
      encoder_initial_state.raw_encoder_counts,
      encoder_state_180.raw_encoder_counts, encoder_state_90.raw_encoder_counts,
      count_delta, dist_turned);

  // Accept up to +-10 Deg angle error.
  if (fabsf((fabsf(dist_turned) - 90.0f)) > 10.0f) {
    PW_LOG_ERROR(
        "\nError: Electrical Angle Calibration failed to turn motor ~90 Deg, "
        "Actual angle was %5.2f Deg.\nElectrical angle calibration has not "
        "been changed!!!\n",
        dist_turned);

    board_.EnableBlinky1Led();   // Set Blinky1 Led, (Red Led on Matraic) to
                                 // indicate error
    board_.DisableBlinky2Led();  // Clear Blinky2 Led, (Yellow Led on Matraic)
                                 // to indicate calibration ended.
    return pw::Status::Internal();
  }

  EthercatFlagsMutex.lock();

  // motor is at 90 Deg, set 0 electrical angle position to 180 deg offset from
  // this
  Obj.SDO3000.electrical_angle_offset =
      ((int32_t)encoder_state_90.raw_encoder_counts) +
      counts_per_elect_turn / 2;

  Obj.SDO3000.electrical_angle_calibration_completed = 1;
  EthercatFlags.write_electrical_angle_calibration_value = true;
  EthercatFlagsMutex.unlock();

  ecat_device_.Restart();

  // Clear Blinky1 Led, (Red Led on Matraic) to indicate NO error.
  board_.DisableBlinky1Led();
  // Clear Blinky2 Led, (Yellow Led on Matraic) to indicate calibration ended.
  board_.DisableBlinky2Led();

  return pw::OkStatus();
}

// ===============================================================================================
pw::Status ControllerThread::CalibrateJointAngleAndRestartEthercat(void) {
  PW_LOG_INFO("Running joint angle calibration.");

  std::array<int32_t, kJointAngleCalibrationNumEncoderSamples>
      joint_angle_samples = {};

  // Get joint angle samples.
  for (int32_t& sample : joint_angle_samples) {
    PW_TRY_ASSIGN(RotaryEncoderState enc_state, encoder_.Update());

    sample = enc_state.multiturn_encoder_counts.value();

    // Wait for next controller timing signal.
    controller_notification_.acquire();
  }

  // Take the median sample by sorting the whole array and selecting the element
  // at index N / 2. This might be a little slow, but the control loop isn't
  // active at this point so it shouldn't matter.
  std::sort(joint_angle_samples.begin(), joint_angle_samples.end());

  int32_t joint_angle_offset =
      joint_angle_samples[kJointAngleCalibrationNumEncoderSamples / 2];

  PW_LOG_INFO("Joint angle calibration complete. Offset: %ld.",
              joint_angle_offset);

  EthercatFlagsMutex.lock();
  Obj.SDO3001.joint_angle_offset = joint_angle_offset;
  Obj.SDO3001.joint_angle_calibration_completed = 1;
  EthercatFlags.write_joint_angle_calibration_value = true;
  EthercatFlagsMutex.unlock();

  ecat_device_.Restart();

  return pw::OkStatus();
}

}  // namespace barkour
