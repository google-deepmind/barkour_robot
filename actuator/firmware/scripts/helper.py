# Copyright 2024 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Script to assist flashing of the Cortes board."""

import argparse
import os
import re
import subprocess
import sys
import time
from absl import logging
from rules_python.python.runfiles import runfiles

_M4_ELF_PATH = '__main__/actuator/firmware/barkour/m4/m4.elf'
_M7_ELF_PATH = '__main__/actuator/firmware/barkour/m7/m7.elf'
_GDB_PATH = '__main__/external/gcc_arm_none_eabi_toolchain/bin/arm-none-eabi-gdb'


def jlink_flash_all():
  """Flash both cores using JLink."""
  r = runfiles.Create()
  m4_location_path = r.Rlocation(_M4_ELF_PATH)
  m7_location_path = r.Rlocation(_M7_ELF_PATH)
  gdb_location_path = r.Rlocation(_GDB_PATH)

  print(f'm4 Rlocation is: {m4_location_path}')
  print(f'm7 Rlocation is: {m7_location_path}')
  print(f'gdb Rlocation is: {gdb_location_path}')

  logging.info('Starting JLink GDB server.')
  subprocess.Popen([
      'JLinkGDBServerCLExe',
      '-device',
      'STM32H755II_M4',
      '-if',
      'SWD',
      '-vd',
      '-nogui',
      '-singlerun',
  ])

  logging.info('Starting GDB client.')
  fnull = open(os.devnull, 'w')
  gdb_client_args = [
      gdb_location_path,
      '-ex',
      'target remote localhost:2331',
      '-ex',
      'monitor flash device = STM32H755II_M4',
      '-ex',
      'monitor flash download = 1',
      '-ex',
      'monitor endian little',
      '-ex',
      'monitor reset 0',
      '-ex',
      f'load {m7_location_path}',
      '-ex',
      f'load {m4_location_path}',
      '-ex',
      'monitor go',
      '-ex',
      'disconnect',
      '-ex',
      'quit',
  ]

  subprocess.call(gdb_client_args, stdout=fnull)
  return


def update_firmware_for_device(
    m4_bin_path: str, m7_bin_path: str, device_alias: int
):
  """Flashes firmware over ethercat for a specific ethercat device.

  Args:
    m4_bin_path: Path to M4 binary.
    m7_bin_path: Path to M7 binary.
    device_alias: EtherCAT device alias.
  """
  try:
    # Change device's ethercat state (PREOP/OP->INIT->BOOT)
    subprocess.run(
        ['ethercat', 'states', '-a', str(device_alias), 'INIT'], check=True
    )
    subprocess.run(
        ['ethercat', 'states', '-a', str(device_alias), 'BOOT'], check=True
    )
    # Let the device erase FLASH memory
    time.sleep(10)
    # Send M7 firmware
    logging.info('Flashing M7.')
    subprocess.run(
        [
            'ethercat',
            'foe_write',
            '-a',
            str(device_alias),
            '-o',
            'm7.bin',
            m7_bin_path,
        ],
        check=True,
    )
    logging.info('Finished flashing M7.')
    # Send M4 firmware
    logging.info('Flashing M4.')
    subprocess.run(
        [
            'ethercat',
            'foe_write',
            '-a',
            str(device_alias),
            '-o',
            'm4.bin',
            m4_bin_path,
        ],
        check=True,
    )
    logging.info('Finished flashing M4.')
    # Move back to device's ethercat state PREOP (BOOT->INIT->PREOP)
    subprocess.run(
        ['ethercat', 'states', '-a', str(device_alias), 'INIT'], check=True
    )
    subprocess.run(
        ['ethercat', 'states', '-a', str(device_alias), 'PREOP'], check=True
    )
    time.sleep(1)
    # Send '0xDEADC0DE' to SDO index: 0x3030, subindex:0x00
    # to boot new firmware by resetting and swapping FLASH banks
    subprocess.run(
        [
            'ethercat',
            'download',
            '-a',
            str(device_alias),
            '-t',
            'uint32',
            '0x3030',
            '0x00',
            '0xDEADC0DE',
        ],
        check=False,
    )
  except subprocess.CalledProcessError:
    logging.error('Failed to flash firmware on device %s.', str(device_alias))


def firmware_over_ethercat(
    m4_bin_path: str, m7_bin_path: str, device_alias: int, flash_all: bool
) -> int:
  """Flashes firmware over ethercat.

  Args:
    m4_bin_path: Path to M4 binary.
    m7_bin_path: Path to M7 binary.
    device_alias: EtherCAT device alias.
    flash_all: If true, flash all devices. If false, only flash specified
      device.

  Returns:
    1 if error, 0 if success.
  """
  # Check if both m4 and m7 binaries exist
  if os.path.exists(m4_bin_path) and os.path.exists(m7_bin_path):
    if flash_all:
      logging.info('Flashing firmware over ethercat')
      logging.info('M4 binary path: %s', m4_bin_path)
      logging.info('M7 binary path: %s', m7_bin_path)
      # Find all devices and their aliases on the network
      result = subprocess.run(
          ['ethercat', 'slaves'], stdout=subprocess.PIPE, check=True
      )
      devices_detected = re.findall(r'(\d*):', str(result.stdout))
      if not devices_detected:
        logging.error('No devices detected.')
        return 1
      logging.info('Devices (by alias) to reflash: %s', devices_detected)
      # Iterate reversely over devices to prevent the 1st chip on the network
      # from resetting
      for each_device in reversed(devices_detected):
        logging.info('Flashing device %s.', each_device)
        update_firmware_for_device(m4_bin_path, m7_bin_path, each_device)
      logging.info('Power-cycle the network to boot new firmware.')
      return 0
    else:
      logging.info('Flashing firmware over ethercat')
      logging.info('M4 binary path: %s', m4_bin_path)
      logging.info('M7 binary path: %s', m7_bin_path)
      logging.info('device_alias: %s', str(device_alias))
      # Check if specified device alias exists in the network
      result = subprocess.run(
          ['ethercat', 'slaves'], stdout=subprocess.PIPE, check=True
      )
      devices = re.findall(r'(\d*):', str(result.stdout))
      if str(device_alias) in devices:
        update_firmware_for_device(m4_bin_path, m7_bin_path, device_alias)
        logging.info('Power-cycle the device to boot new firmware.')
        return 0
      else:
        logging.error(
            'Device alias %s not found on the network.', str(device_alias)
        )
        return 1
  else:
    logging.error('M4 or M7 binaries do not exist.')
    return 1


def swap_memory_banks(device_alias: int):
  """Swap the memory banks."""
  cmd = ['ethercat', 'rescan']
  subprocess.call(cmd, stdout=subprocess.PIPE)
  time.sleep(5)  # Time for the device to boot.
  cmd = [
      'ethercat',
      'download',
      '-a',
      str(device_alias),
      '-t',
      'uint32',
      '0x3030',
      '00',
      '0xDEADC0DE',
  ]
  subprocess.call(cmd, stdout=subprocess.PIPE)
  logging.info('Bank swap completed.')
  logging.info(
      "If there were any errors printed above, it's safest to flash the device"
      ' once more, followed by another bank swap.'
  )


def main() -> int:
  """Run a full build."""
  logging.set_verbosity(logging.INFO)

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      '--debug',
      action='store_true',
      help='Build for Debug. By default optimized targets will be built.',
  )
  parser.add_argument(
      '--flash', action='store_true', help='Flash the discovery board.'
  )
  parser.add_argument(
      '-j',
      '--jlink_gdb_server_path',
      default=None,
      help='The path of the JLink commander executable.',
  )
  parser.add_argument(
      '-t',
      '--tool',
      default='jlink',
      help='Flash software to use ("jlink", "ecat").',
  )
  parser.add_argument(
      '--device_alias',
      default=1,
      type=int,
      help='Alias of the EtherCAT device on the bus [1-12].',
  )
  parser.add_argument(
      '--swap_banks',
      action='store_true',
      help=(
          'Activates new firmware flashed via EtherCAT by swapping the '
          'memory banks and restarting the device. Can be used in combination '
          'with --flash --tool=ecat or independently.'
      ),
  )
  parser.add_argument(
      '--m4',
      type=str,
      help='Path to M4 Core firmware in .bin format.',
  )
  parser.add_argument(
      '--m7',
      type=str,
      help='Path to M7 Core firmware in .bin format.',
  )
  parser.add_argument(
      '--all',
      action='store_true',
      default=False,
      help='Flashes all ethercat chips detected on the network.',
  )

  args = parser.parse_args()

  if not (args.flash or args.swap_banks):
    logging.error('Please provide --flash flags or --swap_banks.')
    return 1

  if args.flash:
    if args.tool == 'jlink':
      jlink_flash_all()
    elif args.tool == 'ecat':
      # check if paths to binaries provided
      if not args.m4 or not args.m7:
        logging.error('Please provide --m4 and --m7 binaries.')
        return 1
      else:
        firmware_over_ethercat(args.m4, args.m7, args.device_alias, args.all)

  if args.swap_banks:
    logging.info(
        'Swapping memory banks on device with alias %d to activate new'
        ' firmware',
        args.device_alias,
    )
    swap_memory_banks(args.device_alias)

  return 0

if __name__ == '__main__':
  sys.exit(main())
