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

// A self-contained example to control Barkour motors via EtherCAT.
//
// Expected behavior: Holds all joints in place and slowly wiggles one joint
// (the motor with device alias 1).
//
// Caveat: This example does not include safety features, so make sure to
// refine it before using it in a real system.
//
// The goal of this example is to be self-contained (connect to the EtherCAT
// bus, configure the motors, apply a basic control loop).
//
// You should have the Etherlab kernel module installed and loaded before
// running this example (i.e. `ethercat slaves` should list devices on the bus).
// In case you don't have the kernel module installed, you can talk to a "fake"
// EtherCAT bus by adding the `--fake_ethercat_host` flag.
//
// Usage:
// bazel build --cxxopt='-std=c++17' -c opt ethercat_host:ethercat_example
// bazel-bin/ethercat_host/ethercat_example --stderrthreshold=0
//
// To adapt this example for your own use-case:
// - Add safety features/error handling. In particular, consider
// timeouts/working counter errors and validate the torque output.
// - Update the MotorController class to accept external commands (connect it to
// your favorite middleware).
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/initialize.h"
#include "absl/log/flags.h"  // IWYU pragma: keep
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "canopen.h"
#include "ds402_state.h"
#include "custom_objects.h"
#include "ds301_objects.h"
#include "ethercat_host.h"
#include "etherlab_ethercat_host.h"
#include "fake_ethercat_host.h"
#include "types.h"
#include "ds402_objects.h"


ABSL_FLAG(bool, fake_ethercat_host, false, "Use a fake Ethercat host?");

struct MotorParameters {
  double rated_torque;        // Nm
  double encoder_resolution;  // encoder ticks/rad
  double gear_ratio;          // rotor turns (input)/shaft turns (output)
};

struct MotorState {
  // Received from the motor driver.
  double position = 0.0;  // radians
  double velocity = 0.0;  // radians/sec
  double torque = 0.0;    // Nm
  std::array<std::byte, barkour::kBinaryBlobNumBytes> blob_received_buffer;
  barkour::EthercatApplicationLayerState ethercat_al_state;
  uint32_t manufacturer_status_register;
  uint16_t status_word;
  barkour::Ds402State ds402_current_state;

  // Sent to the motor driver.
  double torque_setpoint = 0.0;  // Nm
  barkour::Ds402State ds402_target_state;
  uint16_t control_word = 0;
  // We can also write a blob to the motor driver, but this is not currently
  // used.
};

class MotorController {
 private:
  std::vector<MotorState> initial_motor_states_;
  absl::Time start_time_;
  absl::Time last_debug_print_time_;

 public:
  MotorController() = default;
  ~MotorController() = default;

  void ComputeMotorTorques(
      absl::Span<const barkour::EthercatDeviceInfo> device_infos,
      absl::Span<const MotorParameters> motor_parameters,
      absl::Span<MotorState> motor_states) {
    bool all_motors_enabled = true;
    for (size_t i = 0; i < motor_states.size(); ++i) {
      if (motor_states[i].ds402_current_state !=
          barkour::Ds402State::kOperationEnabled) {
        all_motors_enabled = false;
        break;
      }
    }

    // Until all motors are enabled, we don't apply any torque.
    if (!all_motors_enabled) {
      LOG_EVERY_N_SEC(INFO, 0.5) << "Waiting for all motors to be in operation "
                                    "enabled mode before applying torque.";
      // Don't accidentally jump back to a previous pose.
      initial_motor_states_.clear();
      for (size_t i = 0; i < motor_states.size(); ++i) {
        motor_states[i].torque_setpoint = 0;
      }
      return;
    }

    if (initial_motor_states_.empty()) {
      LOG(INFO) << "Enabling torque.";
      std::copy(motor_states.begin(), motor_states.end(),
                std::back_inserter(initial_motor_states_));
      start_time_ = absl::Now();
      last_debug_print_time_ = absl::UnixEpoch();
    }

    // The actual motor control loop.
    // Replace this example with something a little more advanced or accept
    // external commands.
    //
    // We just need to compute the torque setpoint for each motor: PD control.
    //
    // Note that this example doesn't have many safety features, so make sure to
    // refine this code before using it for a real system.
    const double kp = 30.0;         // Nm/rad. Don't exceed ~50 Nm/rad/s.
    const double kd = 0.5;          // Nm/rad/s. Don't exceed ~1 Nm/rad/s!
    const double max_torque = 4.0;  // Nm. Don't exceed ~20 Nm.
    for (size_t i = 0; i < motor_states.size(); ++i) {
      double target_position = initial_motor_states_[i].position;
      double target_velocity = 0.0;
      if (static_cast<int>(device_infos[i].station_alias) == 1) {
        // Wiggle the first joint on the bus a bit.
        target_position +=
            std::sin(absl::ToDoubleSeconds(absl::Now() - start_time_)) * 0.25;
      }
      double position_error =
          motor_states[i].position - target_position;
      double velocity_error = motor_states[i].velocity - target_velocity;
      motor_states[i].torque_setpoint =
          std::clamp(-kp * position_error - kd * velocity_error,
                     -max_torque, max_torque);
    }

    if (absl::Now() - last_debug_print_time_ > absl::Milliseconds(200)) {
      for (size_t i = 0; i < motor_states.size(); ++i) {
        LOG(INFO) << "Motor " << i << ": position " << motor_states[i].position
                  << ", velocity " << motor_states[i].velocity << ", torque "
                  << motor_states[i].torque << ", torque_setpoint "
                  << motor_states[i].torque_setpoint;
      }
      last_debug_print_time_ = absl::Now();
    }
  }
};

barkour::FakeEthercatHost* CreateFakeHost() {
  barkour::FakeEthercatHost* host = new barkour::FakeEthercatHost();
  std::vector<barkour::EthercatDeviceInfo> devices = {
      barkour::EthercatDeviceInfo{
          .bus_position = barkour::EthercatBusPosition(0),
          .station_alias = barkour::EthercatStationAlias(1),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 37,
          .name = "device_0",
      },
      barkour::EthercatDeviceInfo{
          .bus_position = barkour::EthercatBusPosition(1),
          .station_alias = barkour::EthercatStationAlias(2),
          .vendor_id = 8665,
          .product_code = 105689,
          .serial_number = 42,
          .name = "device_1",
      }};

  host->SetDevicesOnBus(devices).IgnoreError();

  return host;
}

ABSL_CONST_INIT absl::Mutex interrupted_mtx(absl::kConstInit);

bool interrupted = false;

// When the program is interrupted, we want to ensure that the driver is
// properly finalized. This function catches an interrupt signal and causes
// the main runloop to terminate on the next iteration.
void KeyboardInterruptHandler(sig_atomic_t signal) {
  absl::MutexLock lock(&interrupted_mtx);
  LOG(INFO) << "Received shutdown request";
  interrupted = true;
}

absl::StatusOr<MotorParameters> ConfigureMotorDriver(
    barkour::EthercatHost& ethercat_host,
    const barkour::EthercatDeviceInfo& device_info) {
  MotorParameters temp_parameters;

  // Make it possible to register the device for PDO transfer.
  {
    auto status =
        ethercat_host.RegisterDeviceForPdoTransfer(device_info.station_alias);
    if (!status.ok()) {
      return status;
    }
  }

  // Configure the device via SDO.
  // Rated torque.
  absl::StatusOr<uint32_t> raw_rated_torque = ethercat_host.ReadSdo<uint32_t>(
      device_info.bus_position, barkour::kRatedTorqueObjectInfo.address);
  if (!raw_rated_torque.ok()) {
    return raw_rated_torque.status();
  }
  temp_parameters.rated_torque = *raw_rated_torque * 0.001;  // nMm to Nm

  LOG(INFO) << "Rated torque (Nm): " << temp_parameters.rated_torque;
  if (temp_parameters.rated_torque < 1e-3) {
    return absl::InternalError(
        absl::StrCat("Invalid rated torque value read from motor driver: ",
                     *raw_rated_torque,
                     ". Has the motor driver's firmware been properly "
                     "configured? FYI, this is object ",
                     barkour::kRatedTorqueObjectInfo.address.ToString()));
  }

  absl::StatusOr<double> encoder_counts_per_revolution =
      ethercat_host.ReadSdo<uint32_t>(
          device_info.bus_position,
          barkour::kEncoderIncrementsObjectInfo.address);
  if (!encoder_counts_per_revolution.ok()) {
    return encoder_counts_per_revolution.status();
  }
  absl::StatusOr<double> encoder_motor_revolutions =
      ethercat_host.ReadSdo<uint32_t>(
          device_info.bus_position,
          barkour::kEncoderMotorRevolutionsObjectInfo.address);
  if (!encoder_motor_revolutions.ok()) {
    return encoder_motor_revolutions.status();
  }
  temp_parameters.encoder_resolution =
      (*encoder_counts_per_revolution / *encoder_motor_revolutions) /
      (2 * M_PI);
  LOG(INFO) << "Encoder resolution (ticks/rad): "
            << temp_parameters.encoder_resolution;

  absl::StatusOr<double> gear_motor_revolutions =
      ethercat_host.ReadSdo<uint32_t>(
          device_info.bus_position,
          barkour::kGearMotorRevolutionsObjectInfo.address);
  if (!gear_motor_revolutions.ok()) {
    return gear_motor_revolutions.status();
  }
  absl::StatusOr<double> gear_shaft_revolutions =
      ethercat_host.ReadSdo<uint32_t>(
          device_info.bus_position,
          barkour::kGearShaftRevolutionssObjectInfo.address);
  if (!gear_shaft_revolutions.ok()) {
    return gear_shaft_revolutions.status();
  }
  temp_parameters.gear_ratio =
      *gear_motor_revolutions / *gear_shaft_revolutions;
  LOG(INFO) << "Gear ratio: " << temp_parameters.gear_ratio;
  if (temp_parameters.gear_ratio < 1e-3) {
    return absl::InternalError(absl::StrCat(
        "Invalid gear ratio read from motor driver: ",
        temp_parameters.gear_ratio,
        ". Has the motor driver's firmware been properly "
        "configured? FYI, this is object ",
        barkour::kGearMotorRevolutionsObjectInfo.address.ToString(), " and ",
        barkour::kGearShaftRevolutionssObjectInfo.address.ToString()));
  }

  // Enable PDO transfer.
  // Tx PDOs (from device)
  {
    auto status = ethercat_host.RegisterPdoEntry(
      device_info.station_alias, barkour::kStatuswordObjectInfo,
      barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
      device_info.station_alias, barkour::kModesDisplayObjectInfo,
      barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
      device_info.station_alias, barkour::kPositionObjectInfo,
      barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
      device_info.station_alias, barkour::kVelocityObjectInfo,
      barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
      device_info.station_alias, barkour::kTorqueObjectInfo,
      barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias,
        barkour::kManufacturerStatusRegisterObjectInfo,
        barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias,
        barkour::kTxPdoBlobAddress, -1,
        barkour::PdoDirection::kTxDeviceToHost);
    if (!status.ok()) {
      return status;
    }
  }

  // Rx PDO (to device)
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias, barkour::kControlwordObjectInfo,
        barkour::PdoDirection::kRxHostToDevice);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias, barkour::kModesObjectInfo,
        barkour::PdoDirection::kRxHostToDevice);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias, barkour::kTorqueSetpointObjectInfo,
        barkour::PdoDirection::kRxHostToDevice);
    if (!status.ok()) {
      return status;
    }
  }
  {
    auto status = ethercat_host.RegisterPdoEntry(
        device_info.station_alias,
        barkour::kRxPdoBlobAddress, -1,
        barkour::PdoDirection::kRxHostToDevice);
    if (!status.ok()) {
      return status;
    }
  }

  return temp_parameters;
}

absl::StatusOr<bool> CheckAllDevicesInOperationalState(
    barkour::EthercatHost& ethercat_host,
    absl::Span<const barkour::EthercatDeviceAddress> device_aliases,
    absl::Span<MotorState> motor_states) {
  bool any_device_not_in_op = false;

  for (size_t i = 0; i < device_aliases.size(); ++i) {
    absl::StatusOr<barkour::EthercatApplicationLayerState> al_state =
        ethercat_host.GetDeviceApplicationLayerState(device_aliases[i]);
    if (!al_state.ok()) {
      return al_state.status();
    }

    motor_states[i].ethercat_al_state = *al_state;
    any_device_not_in_op = any_device_not_in_op ||
                           (al_state->base_state !=
                            barkour::BaseEthercatApplicationLayerState::kOp) ||
                           (al_state->error);
  }
  return any_device_not_in_op;
}

absl::Status ReadMotorStatesFromPdo(
    barkour::EthercatHost& ethercat_host,
    absl::Span<const barkour::EthercatDeviceInfo> device_infos,
    absl::Span<const MotorParameters> motor_parameters,
    absl::Span<MotorState> motor_states) {
  for (size_t i = 0; i < device_infos.size(); ++i) {
    absl::StatusOr<uint16_t> statusword = ethercat_host.ReadPdo<uint16_t>(
        device_infos[i].station_alias, barkour::kStatuswordObjectInfo.address);
    if (!statusword.ok()) {
      return statusword.status();
    }
    absl::StatusOr<uint32_t> manufacturer_statusword =
        ethercat_host.ReadPdo<uint32_t>(
            device_infos[i].station_alias,
            barkour::kManufacturerStatusRegisterObjectInfo.address);
    if (!manufacturer_statusword.ok()) {
      return manufacturer_statusword.status();
    }
    absl::StatusOr<int32_t> position = ethercat_host.ReadPdo<int32_t>(
        device_infos[i].station_alias, barkour::kPositionObjectInfo.address);
    if (!position.ok()) {
      return position.status();
    }
    absl::StatusOr<int32_t> velocity = ethercat_host.ReadPdo<int32_t>(
        device_infos[i].station_alias, barkour::kVelocityObjectInfo.address);
    if (!velocity.ok()) {
      return velocity.status();
    }
    absl::StatusOr<int16_t> torque = ethercat_host.ReadPdo<int16_t>(
        device_infos[i].station_alias, barkour::kTorqueObjectInfo.address);
    if (!torque.ok()) {
      return torque.status();
    }
    absl::StatusOr<barkour::EthercatHost::ByteSpan> blob_bytes =
        ethercat_host.GetPdoBytes(device_infos[i].station_alias,
                                  barkour::kTxPdoBlobAddress);
    if (!blob_bytes.ok()) {
      return blob_bytes.status();
    }

    motor_states[i].status_word = *statusword;
    if (!barkour::Ds402StateFromStatusWord(
            *statusword, motor_states[i].ds402_current_state)) {
      return absl::InternalError(
          absl::StrCat("Received invalid status word from the motor driver:",
                       device_infos[i].ToString()));
    }
    motor_states[i].manufacturer_status_register = *manufacturer_statusword;
    motor_states[i].position = static_cast<double>(*position) /
                               motor_parameters[i].encoder_resolution /
                               motor_parameters[i].gear_ratio;
    motor_states[i].velocity = static_cast<double>(*velocity) /
                               motor_parameters[i].encoder_resolution /
                               motor_parameters[i].gear_ratio;
    motor_states[i].torque = motor_parameters[i].rated_torque *
                             motor_parameters[i].gear_ratio *
                             static_cast<double>(*torque) / 1000.0;
    std::copy(motor_states[i].blob_received_buffer.begin(),
              motor_states[i].blob_received_buffer.end(), blob_bytes->begin());
  }
  return absl::OkStatus();
}

absl::Status WriteMotorStatesToPdo(
    barkour::EthercatHost& ethercat_host,
    absl::Span<const barkour::EthercatDeviceInfo> device_infos,
    absl::Span<const MotorParameters> motor_parameters,
    absl::Span<MotorState> motor_states) {
  for (size_t i = 0; i < device_infos.size(); ++i) {
    {
      absl::Status status = ethercat_host.WritePdo<uint16_t>(
          device_infos[i].station_alias,
          barkour::kControlwordObjectInfo.address,
          motor_states[i].control_word);
      if (!status.ok()) {
        return status;
      }
    }

    // Convert from SI to DS402 data type.
    int16_t target_torque = static_cast<int16_t>(
        motor_states[i].torque_setpoint * 1000.0 /
        motor_parameters[i].rated_torque / motor_parameters[i].gear_ratio);

    {
      absl::Status status = ethercat_host.WritePdo<int16_t>(
          device_infos[i].station_alias,
          barkour::kTorqueSetpointObjectInfo.address, target_torque);
      if (!status.ok()) {
        return status;
      }
    }
  }
  return absl::OkStatus();
}

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  absl::InitializeLog();

  std::vector<barkour::EthercatDeviceAddress> device_aliases;
  std::vector<barkour::EthercatDeviceInfo> device_infos;
  std::vector<MotorParameters> motor_parameters;
  std::vector<MotorState> motor_states;
  MotorController motor_controller;

  signal(SIGINT, KeyboardInterruptHandler);

  // Connect to the EtherCAT host (fake or real).
  LOG(INFO) << "Connecting to EtherCAT bus";
  barkour::EthercatHost* host;
  if (absl::GetFlag(FLAGS_fake_ethercat_host)) {
    host = CreateFakeHost();
  } else {
    host = new barkour::EtherlabEthercatHost();
  }
  absl::Status status = host->Connect();
  if (!status.ok()) {
    LOG(FATAL) << "Failed to connect to EtherCAT: " << status.message();
    return 1;
  }

  // Get the list of devices on the bus.
  LOG(INFO) << "Iterating devices on the EtherCAT bus";
  auto devices = host->GetDevicesOnBus();
  if (!devices.ok()) {
    LOG(FATAL) << "Failed to iterate devices on the bus: "
               << devices.status().message();
    return 1;
  }
  for (const auto& device : *devices) {
    LOG(INFO) << "Found device on the bus: " << device.ToString();
  }

  // Configure the devices on the bus that we'd like to control.
  LOG(INFO) << "Configuring EtherCAT devices";
  // Configure the PDOs.
  for (const auto& device : *devices) {
    // Configure each motor driver via SDO.
    // You might want to check that all the devices you care about are present.
    {
      auto parameters = ConfigureMotorDriver(*host, device);
      if (!parameters.ok()) {
        LOG(FATAL) << "Failed to configure motor driver: "
                   << parameters.status().message();
        return 1;
      }
      device_aliases.push_back(device.station_alias);
      device_infos.push_back(device);
      motor_parameters.push_back(*parameters);
      motor_states.emplace_back();
    }
  }

  {
    absl::Status status = host->ConfigurePdoTransfer();
    if (!status.ok()) {
      LOG(FATAL) << "Failed to configure PDO transfers: " << status.message();
      return 1;
    }
  }
  {
    absl::Status status = host->StartCyclicOperation();
    if (!status.ok()) {
      LOG(FATAL) << "Failed to start cyclic operation: " << status.message();
      return 1;
    }
  }

  // Main runloop.
  absl::Time last_cycle_time;
  while (true) {
    last_cycle_time = absl::Now();
    if (absl::MutexLock lock(&interrupted_mtx); interrupted) {
      signal(SIGINT, SIG_DFL);
      break;
    }
    {  // Reads PDO data from the bus into the object dictionary.
      auto status = host->PreCycle();
      if (!status.ok()) {
        LOG(ERROR) << "Failed to pre-cycle EtherCAT: " << status.message();
        break;
      }
    }
    // Wait until all devices are in the EtherCAT OP state.
    absl::StatusOr<bool> any_device_not_in_op =
        CheckAllDevicesInOperationalState(*host, device_aliases,
                                          absl::MakeSpan(motor_states));
    if (!any_device_not_in_op.ok()) {
      LOG(ERROR) << "Failed to check if all devices are in operational state: "
                 << any_device_not_in_op.status().message();
      break;
    }
    if (*any_device_not_in_op) {
      LOG_EVERY_N_SEC(INFO, 0.5)
          << "Waiting for all devices to be in operational state (AL).";
    }
    {
      {  // Reads PDO data from the object dictionary.
        absl::Status status =
            ReadMotorStatesFromPdo(*host, device_infos, motor_parameters,
                                   absl::MakeSpan(motor_states));
        if (!status.ok()) {
          LOG(ERROR) << "Failed to read motor states from PDOs: "
                     << status.message();
          break;
        }
      }
      // Request that all motors go into operation enabled state (CANopen).
      // To avoid confusion: the EtherCAT AL state refers to the state of the
      // EtherCAT bus. See
      // https://infosys.beckhoff.com/english.php?content=../content/1033/ek1310/1036980875.html&id=
      // The Ds402State is the CANopen (the higher level protocol on top of the
      // bare EtherCAT bus) state of the motor drivers:
      // https://webhelp.kollmorgen.com/AKD2G/English/Content/AKD2G%20CANopen/Device%20Control.htm
      for (size_t i = 0; i < motor_states.size(); ++i) {
        motor_states[i].ds402_target_state =
            barkour::Ds402State::kOperationEnabled;
        barkour::Ds402ControlWordForTargetState(
            motor_states[i].ds402_target_state,
            motor_states[i].ds402_current_state, motor_states[i].control_word,
            true);
      }
      // The actual control loop. Insert magic here.
      motor_controller.ComputeMotorTorques(device_infos, motor_parameters,
                                           absl::MakeSpan(motor_states));

      {  // Updates the PDO data (object dictionary).
        absl::Status status =
            WriteMotorStatesToPdo(*host, device_infos, motor_parameters,
                                   absl::MakeSpan(motor_states));
        if (!status.ok()) {
          LOG(ERROR) << "Failed to read motor states from PDOs: "
                     << status.message();
          break;
        }
      }
    }
    {  // Writes PDO data to the bus.
      auto status = host->PostCycle();
      if (!status.ok()) {
        LOG(ERROR) << "Failed to post-cycle EtherCAT: " << status.message();
        break;
      }
    }
    // Sleep until the next cycle (1kHz loop).
    // This is probably the worst way to sleep reliably.
    // Please replace this with something more robust to prevent drift etc.
    absl::Duration elapsed = absl::Now() - last_cycle_time;
    absl::Duration sleep_duration = absl::Milliseconds(1) - elapsed;
    if (sleep_duration < absl::ZeroDuration()) {
      LOG_EVERY_N_SEC(WARNING, 0.5) << "Cycle time exceeded";
    }
    absl::SleepFor(sleep_duration);
  }

  // Clean up
  LOG(INFO) << "Disconnecting from EtherCAT";
  {
    absl::Status status = host->StopCyclicOperation();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to stop cyclic operation: " << status.message();
    }
  }
  {
    absl::Status status = host->ClearRegisteredPdos();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to clear PDOs: " << status.message();
    }
  }
  {
    absl::Status status = host->ClearRegisteredDevices();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to clear registered devices: " << status.message();
    }
  }
  {
    absl::Status status = host->Disconnect();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to disconnect from EtherCAT: " << status.message();
    }
  }

  delete host;
  return 0;
}
