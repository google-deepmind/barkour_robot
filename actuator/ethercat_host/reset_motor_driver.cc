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

// Controls the RESET_OUT pin on the Trinamic TMC8462 to reset the STM32.
//
// This can be used to effectively power cycle motor drivers via EtherCAT.
// This script will cause the RESET_OUT on the TMC8462 to go high. This pin is
// connected to nRST on the STM32 and also to NRESET on the TMC8462 itself.
// Thus, RESET_OUT will cause both the STM32 and the TMC8462 to reset, which
// should be equivalent to physically pushing the reset button on the motor
// driver debugger (Mataric)
//
// Note that this will only work on the Holberton version of the motor driver,
// because the Cortes is missing a connection from the RESET_OUT pin to the
// Trinamic's own reset pin. Hence, using this script on a Cortes board, will
// hold it permanently in reset.
//
// Usage:
// This script requires raw socket access. You can enable this like so:
// sudo setcap CAP_NET_RAW+pe reset_motor_driver
// Make sure the Etherlab kernel module is not loaded as this script needs to
// talk directly to the ethernet interface: sudo systemctl stop ethercat
//
// Finally, you can run the script like this to reset all motor drivers:
// ./reset_motor_driver --stderrthreshold=0 --ethercat_interface=enx7cc2c6473b62
//
// If you'd like to reset a specific motor driver, use the --bus_position flag.
//
// To restart the EtherCAT kernel module afterwards, run
// sudo systemctl start ethercat

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <vector>

#include "SOEM/soem/ethercat.h"  // IWYU pragma: keep
#include "SOEM/soem/ethercatbase.h"
#include "SOEM/soem/ethercatconfig.h"
#include "SOEM/soem/ethercatmain.h"
#include "SOEM/soem/ethercattype.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/initialize.h"
#include "absl/log/flags.h"  // IWYU pragma: keep
#include "absl/log/log.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

ABSL_FLAG(std::string, ethercat_interface, "eth0", "Network interface to use.");
ABSL_FLAG(int, bus_position, -1,
          "Bus position of motor driver to reset. Set to -1 to reset all "
          "detected devices on bus.");
ABSL_FLAG(int, num_retries, 9,
          "How often to try to reset each device before bailing out");
ABSL_FLAG(absl::Duration, time_between_reset_attempts, absl::Seconds(2),
          "How long to wait between device reset attempts");
ABSL_FLAG(bool, reverse_reset_order, false,
          "When resetting all devices, reset them from the last bus position "
          "to the first?");

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  absl::InitializeLog();
  const int num_retries = absl::GetFlag(FLAGS_num_retries) + 1;
  const std::string ifname = absl::GetFlag(FLAGS_ethercat_interface);
  const absl::Duration time_between_reset_attempts =
      absl::GetFlag(FLAGS_time_between_reset_attempts);
  const int bus_position = absl::GetFlag(FLAGS_bus_position);
  int num_devices = ec_slavecount;

  if (num_retries <= 0) {
    LOG(FATAL) << "Invalid number of retries: " << num_retries;
    return 1;
  }

  LOG(INFO) << "Connecting to the bus once to iterate the devices.";
  LOG(INFO) << "Attempting ec_init on " << ifname;
  if (int ec_init_retv = ec_init(ifname.c_str())) {
    LOG(INFO) << "ec_init on " << ifname << " succeeded";

    if (int ec_config_init_retv = ec_config_init(false) > 0) {
      num_devices = ec_slavecount;
      LOG(INFO) << num_devices << " EtherCAT devices detected:";
      for (int i = 0; i < num_devices; ++i) {
        LOG(INFO) << "Device bus position: " << i
                  << " - configadr: " << ec_slave[i + 1].configadr
                  << " - alias: " << ec_slave[i + 1].aliasadr;
      }
    } else {
      LOG(FATAL) << "ec_config_init failed: " << ec_config_init_retv;
      return 1;
    }
    LOG(INFO) << "Closing EtherCAT.";
    ec_close();
  } else {
    LOG(ERROR) << "Could not initialize EtherCAT.";
    LOG(ERROR) << "Please check that the Etherlab kernel module is not loaded "
                  "(`sudo systemctl stop ethercat`), that the Ethernet port is "
                  "correct and that this binary has RAW network socket access "
                  "(`sudo setcap CAP_NET_RAW+pe reset_motor_driver`).";
    LOG(FATAL) << "ec_init on " << ifname << " failed: " << ec_init_retv;
    return 1;
  }

  LOG(INFO) << "Resetting requested devices.";
  std::vector<int> bus_positions_to_reset;
  std::vector<int> bus_positions_reset_success;
  if (bus_position < 0) {
    LOG(INFO) << "Resetting all devices on the bus.";
    if (absl::GetFlag(FLAGS_reverse_reset_order)) {
      for (int i = num_devices - 1; i >= 0; --i) {
        bus_positions_to_reset.push_back(i);
      }
    } else {
      for (int i = 0; i < num_devices; ++i) {
        bus_positions_to_reset.push_back(i);
      }
    }
  } else if (bus_position >= num_devices) {
    LOG(FATAL) << "Invalid bus position: " << bus_position;
    return 1;
  } else {
    bus_positions_to_reset.push_back(bus_position);
  }

  // Trinamic reset.
  //
  // Note that ec_slave indexing starts from 1 (bus position 0 == index 1)
  //
  // Arguments to ec_FPWR:
  // ADP: Address used by SOEM. Use ADP=ec_slave[bus_position+1].configadr.
  // ADO: Address offset (address of the register to write).
  // length: number of bytes to write.
  // data: pointer to data to write.
  // timeout: timeout in us.

  for (const int position_to_reset : bus_positions_to_reset) {
    absl::SleepFor(time_between_reset_attempts);
    if (int ec_init_retv = ec_init(ifname.c_str())) {
      LOG(INFO) << "ec_init on " << ifname << " succeeded";
      if (int ec_config_init_retv = ec_config_init(false) > 0) {
        if (ec_slavecount != num_devices) {
          LOG(INFO) << ec_slavecount << " EtherCAT devices detected:";
          for (int i = 0; i < ec_slavecount; ++i) {
            LOG(INFO) << "Device bus position: " << i
                      << " - configadr: " << ec_slave[i + 1].configadr
                      << " - alias: " << ec_slave[i + 1].aliasadr;
          }
          LOG(FATAL) << "Number of devices changed: " << ec_slavecount << " vs "
                     << num_devices;
          return 1;
        }
      } else {
        LOG(FATAL) << "ec_config_init failed: " << ec_config_init_retv;
        return 1;
      }
    } else {
      LOG(FATAL) << "ec_init on " << ifname << " failed: " << ec_init_retv;
    }
    bool reset_success = false;
    for (int attempt = 0; attempt < num_retries; ++attempt) {
      LOG(INFO) << "Resetting device at bus position " << position_to_reset
                << " attempt " << attempt + 1 << "/" << num_retries;
      absl::SleepFor(time_between_reset_attempts);
      ec_slavet* device = &ec_slave[position_to_reset + 1];
      int result;
      uint8_t data = 0x52;  // R
      result = ec_FPWR(device->configadr, 0x0040, 1, &data, EC_TIMEOUTRET);
      if (result == EC_NOFRAME) {
        LOG(ERROR) << "Failed to reset device at bus position (R, EC_NOFRAME) "
                   << position_to_reset;
        continue;
      }
      data = 0x45;  // E
      result = ec_FPWR(device->configadr, 0x0040, 1, &data, EC_TIMEOUTRET);
      if (result == EC_NOFRAME) {
        LOG(ERROR) << "Failed to reset device at bus position (E, EC_NOFRAME) "
                   << position_to_reset;
        continue;
      }
      data = 0x53;  // S
      result = ec_FPWR(device->configadr, 0x0040, 1, &data, EC_TIMEOUTRET);
      if (result == EC_NOFRAME) {
        LOG(ERROR) << "Failed to reset device at bus position (S, EC_NOFRAME) "
                   << position_to_reset;
        continue;
      }
      LOG(INFO) << "Successfully reset device at bus position "
                << position_to_reset;
      bus_positions_reset_success.push_back(position_to_reset);
      reset_success = true;
      break;
    }
    if (!reset_success) {
      LOG(ERROR) << "Failed to reset device at bus position "
                 << position_to_reset;
    }
    ec_close();
  }

  LOG(INFO) << "All done. Bye bye!";

  return 0;
}
