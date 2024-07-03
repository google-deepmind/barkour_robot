# Firmware

This page provides instructions to compile, flash, and calibrate the motor
drivers.

The source code can be found in the `actuator/firmware` folder and is based on
the [Pigweed](https://pigweed.dev) framework.

The motor driver is based on the
[STM32H755II](https://www.st.com/en/microcontrollers-microprocessors/stm32h755ii.html),
which is a dual-core (Arm M4 @ 240MHz and M7 @ 480MHz) microcontroller. We are
using the M4 core for the main motor control loop. The M7 core is available for
experimental use. Both cores can communicate with each other using shared
memory. The motor driver uses EtherCAT communication to exchange telemetry and
to receive motor current commands.

Note: The firmware works transparently with either the Holberton or Gebru +
Cortes motor driver variants (see [hardware overview](hardware_overview.md)). No
compilation flags required.

## Compiling the Firmware

Note: We are using [Bazel](https://bazel.build) as the build system. The
instructions below were tested with `bazel 7.1.2` on Debian and Ubuntu.

To build the firmware binaries and helper scripts, run:

```
# From the top-level directory:
bazel build //actuator/firmware/...
```

### Unit Tests

To run the firmware tests on the host, run:

```
# From the top-level directory:
bazel test //actuator/firmware/...
```

## Flashing

There are two ways to flash the motor drivers: using a J-Link connected to the
Mataric debugger board or via EtherCAT (bootloader). If you are flashing the
board for the first time, the J-Link option should be used.

### Flashing the EEPROM (SII)

When bringing up a brand new motor driver, the EtherCAT (TMC8462-BA) chip's
EEPROM has to be flashed to enable EtherCAT communication. Assuming that the
EtherLab library has been [installed](ethercat_config.md), flashing the EEPROM
can be done using the `ethercat` command:

```
cd actuator/ethecat_eeprom
ethercat sii_write -p 0  barkour_robot_sii.hex
```

After rescanning the bus:

```
ethercat rescan
ethercat slaves
```

The device should show up:

```
0   1:0  INIT  E  Cortes
```

Note: The board will be in the error (E) state if no firmware is present on the
microcontroller.

Note: The same SII binary can be used for either the Holberton or Cortes
variants. They will both show up as `Cortes` on the EtherCAT bus.

Important: The provided file uses Google's EtherCAT vendor ID. Update this to
your own [vendor ID](https://www.ethercat.org/en/vendor_id.html) provided by the
EtherCAT Technology Group.

### Flashing with J-Link + Mataric

Connect the Mataric board to the Cortes or Holberton board as shown below:

![Mataric PCBA](images/barkour_hardware/barkour_jlink_and_mataric.png)

Run the `helper` script to flash the board:

```
bazel run //actuator/firmware/scripts:helper -- --flash -jlink
```

### Flashing with the EtherCAT Bootloader (Single Device)

To flash a single device, invoke `helper` script providing paths to each
binary and specifying EtherCAT device alias of the device to be updated.

Note: `helper` script must be called from the same machine which built the
script. That is due to multiple dependencies being required at runtime including
external libraries.

```
bazel run //actuator/firmware/scripts:helper -- --flash --tool ecat --m4 <path to m4.bin> --m7 <path to m7.bin> --device_alias <device_alias>
```

An example of updating a single device with expected output:

```
user@ethercat-host:~/binaries_foe$ bazel run //actuator/firmware/scripts:helper -- --flash --tool ecat --m4 new_fw/m4.bin --m7 new_fw/m7.bin --device_alias 3
INFO:absl:Flashing firmware over ethercat
INFO:absl:M4 binary path: new_fw/m4.bin
INFO:absl:M7 binary path: new_fw/m7.bin
INFO:absl:device_alias: 3
INFO:absl:Flashing M7.
INFO:absl:Finished flashing M7.
INFO:absl:Flashing M4.
INFO:absl:Finished flashing M4.
INFO:absl:Power-cycle the device to boot new firmware.
```

Once binaries are successfully transferred to the device over EtherCAT, the
STM32H755 will boot new firmware after being restarted. The most reliable way to
achieve that is by power cycling the device.

Another option is to send a restarting command over EtherCAT directly using the
Etherlab command line:

```
ethercat download -a <device_alias> -t uint32 0x3030 00 0x01234567
```

### Flashing with the EtherCAT Bootloader (All Devices)

To flash all EtherCAT devices, `helper` can be invoked with `--all` flag as
below,

```
bazel run //actuator/firmware/scripts:helper -- --flash --tool ecat --m4 <path to m4.bin> --m7 <path to m7.bin> --all
```

An example of updating all devices with expected output:

```
user@ethercat-host:~/binaries_foe$ bazel run //actuator/firmware/scripts:helper -- --flash --tool ecat --m4 new_fw/m4.bin --m7 new_fw/m7.bin --all
INFO:absl:Flashing firmware over ethercat
INFO:absl:M4 binary path: new_fw/m4.bin
INFO:absl:M7 binary path: new_fw/m7.bin
INFO:absl:Devices (by alias) to reflash: ['1', '2', '3', '7', '8', '9', '10', '11', '12', '4', '5', '6']
INFO:absl:Flashing device 6.
INFO:absl:Flashing M7.
INFO:absl:Finished flashing M7.
INFO:absl:Flashing M4.
INFO:absl:Finished flashing M4.
INFO:absl:Flashing device 5.
INFO:absl:Flashing M7.
INFO:absl:Finished flashing M7.
INFO:absl:Flashing M4.
INFO:absl:Finished flashing M4.
...
INFO:absl:Flashing device 1.
INFO:absl:Flashing M7.
INFO:absl:Finished flashing M7.
INFO:absl:Flashing M4.
INFO:absl:Finished flashing M4.
INFO:absl:Power-cycle the network to boot new firmware.
```

After updating firmware on all devices it is highly recommended to power cycle
the network so that microcontrollers boot with the new firmware. That is because
restarting each device separately, as described in previous section, can disrupt
EtherCAT network and some devices might lose connection. If you are using the
Holberton motor driver, it is possible to power cycle the devices via EtherCAT
(see [here](ethercat_config.md)).

### How FoE Firmware Updates Work

File over EtherCAT is a method of exchanging large volumes of data between via
an EtherCAT bus. It is a mailbox protocol and as such available in all states
except INIT. However, in this implementation FoE is artificially limited to BOOT
state only, since it can cause heavy bus load and impede EtherCAT performance.

FoE is used here to flash new firmware onto the devices. Currently the firmware
is split into two binaries - `m4.bin` and `m7.bin`.

On the device side, FoE is handled by SOES, with the help from some callbacks.
On the host side, FoE is initiated by Etherlab commands. The following
approximate sequence of events is triggered:

1.  User sets device into BOOT (`ethercat state BOOT`).
2.  User initiates FoE data transfer (`ethercat foe_write -a <device_alias>
    m4.bin`).
3.  Interrupt is generated on M4 to inform that an EtherCAT exchange took place.
4.  EtherCAT task wakes up and reads AL event register 0x220. Flag SM0/1 event
    is active.
5.  `ESC_foeprocess()` is called, which in turn calls
    `foe_writefile_cfg_t.write_function()`, which is set to `flash_firmware()`.
6.  `flash_firmware()` processes the incoming data chunk by chunk. This method
    erases flash sectors and writes the new data to the flash memory.

Activation of the firmware is handled separately. A custom `uint32_t` object
dictionary at index `0x3030:00` is used to trigger a bank swap and a device
reboot when `0xDEADC0DE` is written to it. This activates the new firmware. Use
`ethercat download -a <device_alias> -t uint32 0x3030 00 0xDEADC0DE` to do this
manually. Do note that `ethercat rescan` might be needed after a firmware update
to ensure Etherlab is aware of the SDO entry.

### Firmware Organization

The STM32H755 has two identical memory banks of 1 megabyte each (flash bank 1
starting at address `0x08000000` and flash bank 2 starting at address
`0x08100000`). The banks each consist of 8 128 kB sectors.

The flash banks can be swapped, which is transparent to the microprocessor
cores. This means that after a bank swap, bank 1 is now bank 2 and vice versa
and all addresses that used to point to bank 1 now point to bank 2 and vice
versa. We use this mechanism to enable firmware updates. The current firmware is
always running from flash bank 1 and the update tool (`flash_firmware()`) always
writes to flash bank 2. A bank swap causes the new firmware to become active.

The firmware is organized as follows:

```
0x08000000 - 0x0807FFFF: M7 code (max 512 kilobyte).
0x08080000 - 0x080FFFFF: M4 code (max 512 kilobyte).
```

The firmware for each core is compiled separately, resulting in two ELFs and two
BINs. See the `helper` script for details. This is nothing more than a
byte-by-byte copy of the program data.

As the M7 core always boots first, we place its firmware at the start of the
bank (at the default address for the STM32). When the M7 boots up
(`main_m7.cc`), it checks that the boot addresses for the M7 and M4 are correct.
If not, it updates the relevant registers of the STM32 and reboots the device.
When the M4 boots up, it updates the address of the interrupt vector table to
point to the start of its flash memory (the default is located at the start of
the memory bank, so we have to make sure the M4 does not use the M7's interrupt
vector table).

The 50/50 split between the M7 and M4 is just an arbitrary choice and might not
be ideal. To change the boot addresses, update the following files and make sure
that the M4's code starts at the beginning of a flash sector (128 kB):

1.  `flash_firmware()` in `actuator/firmware/barkour/m4/ecat_device.cc`.
2.  The linker files in `actuator/firmware/targets/m4` and
    `actuator/firmware/targets/m7`.
3.  `actuator/firmware/barkour/m7/main_m7.cc` to set the correct boot addresses.

Note: Flash bank 2 can be used for data storage during normal operation. It will
only be overwritten when new firmware is flashed.

## Calibration

### Running Calibration Routines

To run the calibration routines, the Mataric programmer board must be attached
to the motor driver. There are two calibration routines, which are independent
of one another, and must be run separately.

Successful completion of either calibration routine involves a restart of the
EtherCAT stack, to force a write of the calibrated values to EEPROM / SII.
Depending on the state of the EtherCAT host and device stacks at the time of
calibration, this restart can sometimes cause the EtherCAT stack to enter an
error state. Power cycling the motor after calibration will usually resolve any
such issues.

### Electrical Angle Offset

The electrical angle offset is the difference between the encoder zero position,
and the nearest position where the electrical angle of the motor is zero. This
is critical for operation, as the motor cannot be commutated without this
information. It is calculated by applying a fixed set of voltage commands to the
motor to hold its electrical angle to 180 deg, then to 90 deg, and reading the
encoder angle. The final 0 position offset is then calculated and stored.

A validation check is performed to ensure that the motor turned ~90 electrical
degrees during the process, if the measured turn is with 10 Deg the calibration
is considered good, if the rotation was out by more than 10 Deg an error is
reported and the calibration is aborted.

The motor should have no significant load attached to it, and should be placed
in a position where it is able to rotate freely for at least ~15 (mechanical)
degrees in each direction. Ideally it should also be placed such that the axis
of rotation is parallel to gravity.

The routine is triggered by by holding the `nButton` on the Mataric board while
the power is turned on or the `nRST` button is pushed to reset the STM. This
will run the electrical angle calibration, store the calibration results to
EEPROM (as part of the EtherCAT SII), and restart the EtherCAT stack.

During the calibration process the LEDs on the Mataric board will indicate the
status. At the start of calibration the orange Blinky2 LED will be lit. If the
calibration is successful the Blinky2 LED will go out and the board can be
operated normally. If there is an error during calibration the Red Blinky1 LED
will light and the board will enter a hung error state.

Additional debug info is also reported on the pw console output during the
calibration.

The electrical angle offset can be read via SDO, with index `0x3000`, subindex
`0x1`, as an `int32`, with units of encoder counts. The SDO at index `0x3000`,
subindex `0x2`, type `uint8` should read `1` if the calibration has been
completed successfully, and any other value otherwise.

### Joint Angle Offset

The joint angle offset is the difference between a negative-side joint endstop
and a joint zero position.

To run this routine, the robot joint should be held against its negative
endstop, with a Mataric board attached. Then the following sequence of button
presses is needed on the Mataric board: - The reset button pressed and held
(i.e. not let go yet!) - The `Blinky1` button pressed and held. - The reset
button released. - (A few seconds later) the `Blinky1` button released.

This results in the `Blinky1` button being held as the firmware starts up, which
will trigger a series of encoder reads. The median of these will be used as the
zero offset, and stored to EEPROM (as part of the EtherCAT SII). The EtherCAT
stack will then be restarted.

If the calibration routine has been successful, the position of the motor (as
reported over DS402) will be relative to this recorded position.

The joint angle offset can be read via SDO, with index `0x3001`, subindex `0x1`,
as an `int32`, with units of encoder counts. The SDO at index `0x3001`, subindex
`0x2`, type `uint8` should read `1` if the calibration has been completed
successfully, and any other value otherwise.

### Error Reporting

Error codes corresponding to certain error conditions are be written to the
CANopen "manufacturer status register", index `0x1002`, subindex `0x0`. This can
be mapped to a PDO to allow for monitoring errors in the cyclic control loop on
the host side.

This contains a `uint32` value, where the bits are set according to the
following table. The bit indices are given in LSB order.

| Bit Index | Error Condition               | Result                          |
| --------- | ----------------------------- | ------------------------------- |
| 0         | Electrical angle calibration  | Controller loop stops           |
:           : routine failed                :                                 :
| 1         | Joint angle calibration       | Controller loop stops           |
:           : routine failed                :                                 :
| 2         | Calibrated electrical angle   | Controller loop stops           |
:           : offset not found              :                                 :
| 3         | Calibrated joint angle offset | Joint angle offset of `0` used. |
:           : not found                     :                                 :
| 4         | Invalid motor parameters set  | Controller loop stops           |
| 5         | Invalid current limit set     | Current limit set to infinity   |
| 6         | Failed to configure one or    | Controller loop stops           |
:           : more controller components    :                                 :
| 7         | Failed to get sensor readings | Controller loop stops, if this  |
:           : / derived quantities          : is the first read, else CiA 402 :
:           :                               : fault                           :
| 8         | STO enabled                   | CiA 402 fault or failure to     |
:           :                               : initialize                      :
| 9         | Unexpected gate driver state  | CiA 402 fault                   |
:           : change while voltage is       :                                 :
:           : applied to motor              :                                 :
| 10        | EtherCAT not in "operational" | CiA 402 fault                   |
:           : state while CiA 402 state     :                                 :
:           : machine initialized           :                                 :
| 11        | Bus voltage too low           | CiA 402 fault                   |
| 12        | Overheat detected             | CiA 402 fault                   |
| 13        | STO line 1 down while voltage | CiA 402 fault                   |
:           : is applied to motor           :                                 :
| 14        | STO line 2 down while voltage | CiA 402 fault                   |
:           : is applied to motor           :                                 :

The error bits which result in a CiA 402 fault will be reset upon a CiA 402
fault recovery state transition.

Manufacturer status register can be read over EtherCAT using following command

```
ethercat upload -a <alias> -t uint32 0x1002 0x0
```

## Shared Memory Block

The primary mechanism for inter-core communications is a block of shared memory
which both cores can access. We use the SRAM4 block in Domain 3 for this
purpose, which has address range `0x38000000` - `0x3800FFFF`.

This block is recommended by the reference manual to be used for this purpose.
It can also be seen from the linker files that the M4 core uses SRAM1, SRAM2 and
SRAM3 for its working memory, whereas the M7 core uses the AXI SRAM. This means
that SRAM4 is not used by either core for working memory, and so is safe to use
for inter-core communications without the risk of e.g. corrupting the stack.

### Shared Variables

The shared variables, addresses and expected use patterns are listed below. In
code, they are described in the
`actuator/firmware/barkour/common/shared_memory.h` header.

<!-- mdformat off(Multiline tables not rendered by gitiles) -->
| Address      | Address Alias in Code | Type       | Expected Use Pattern        |
| ------------ | --------------------- | ---------- | --------------------------- |
| `0x38000000` | `kUidWord0Address`    | `uint32_t` | Written to by the M7 core on startup, before the M4 core starts. Read-only thereafter. Contains a copy of the STM variable unique identifier word 0, `UID[31\:0]`, which can only be read by the M7 core. |
| `0x38000004` | `kUidWord1Address`    | `uint32_t` | Written to by the M7 core on startup, before the M4 core starts. Read-only thereafter. Contains a copy of the STM variable unique identifier word 1, `UID[63\:32]`, which can only be read by the M7 core. |
| `0x38000008` | `kUidWord2Address`    | `uint32_t` | Written to by the M7 core on startup, before the M4 core starts. Read-only thereafter. Contains a copy of the STM variable unique identifier word 2, `UID[95\:64]`, which can only be read by the M7 core. |
<!-- mdformat on -->

## EtherCAT Implementation Details

This section describes the implementation details of the EtherCAT stack.

Terminology:

-   ESC: EtherCAT Slave Controller (TMC8462)
-   SII: Slave Information Interface
-   ETG: EtherCAT Technology Group
-   ESCReg: EtherCAT Slave Controller register (see TMC8462 data sheet)

## Implementation of EtherCAT on Cortes using SOES.

SOES is an open source EtherCAT device stack implementation. The kernel state
machine and general device functionality is contained in the following files,
together with their corresponding headers:

```
esc.c
esc.h
esc_coe.c
esc_coe.h
esc_foe.c
esc_foe.h
ecat_slv.c
ecat_slv.h
```

Using SOES requires the following steps:

1.  Implement an interface to the hardware
2.  Specify general configuration options for SOES
3.  Merge everything into your application

### Interface to Hardware

The communication between the ESC and SOES is implemented in these functions:

-   `void ESC_init(const esc_cfg_t *config)`: Initialize hardware
-   `void ESC_read(uint16_t addr, void *buf, uint16_t len)`: Read contents of a
    register
-   `void ESC_write(uint16_t addr, void *buf, uint16_t len)`: Write data to a
    register

The ESC is connected via an SPI bus clocked at 30 MHz.

### Configuration of SOES

The device application is configured in the following header files:

-   `actuator/firmware/barkour/m4/ecat/ecat_options.h`: General options
    configuring the device
-   `actuator/firmware/barkour/m4/ecat/utypes.h`: struct definition for all data
    to be exchanged between the SOES kernel and application.

### Object Dictionary Definition

The EtherCAT object dictionary is defined in
`actuator/firmware/barkour/m4/ecat/objectlist.c`.

## SOES Callbacks and Required Functions

SOES uses a callback mechanism as well as mandatory functions where the user can
implement custom behaviour. These functions are defined in
`actuator/firmware/barkour/m4/ecat_device.cc`. Take a look, it is extensively
documented in source code.

Mandatory are: * `cb_set_outputs()`: Called just after a RxPDO (SM2 event) is
received (host to device) in order to set outputs. Ideally most of the work is
done * here because it is synchronous to RxPDO reception. * `cb_get_inputs()`:
Called just before data is transferred into the output buffer (SM3).

In this application the following optional callbacks are implemented: *
`set_defaults()`: Called in transition INIT->PREOP * `pre_state_change()`:
Called just before a state change. Can be used to perform exit instructions or
to reject the change * `post_state_change()`: Called just after state has
changed, but just before it is acknowledged in the ALstatus register (0x0130) *
`safeoutput()`: Called upon exit of OP state, Error State, etc; used by the
application to ensure outputs are switch to safe state.

These optional callbacks are configured in `struct EthercatDevice` that is
passed to SOES during stack initialization.

## EtherCAT Main Loop

The EtherCAT stack runs in a separate thread on the M4. There is a single entry
point `EthercatDevice::run()`, implemented in
`actuator/firmware/barkour/m4/ecat_device.cc`, which runs forever. First the
stack is configured with `ecat_slv_init()` after which it goes into an infinite
loop, waiting on a semaphore, calling `ecat_slv_poll()` to get EtherCAT status
information and `DIG_process()` to process PDOs.

The loop is interrupt driven. Whenever an EtherCAT packet is received, it
generates an interrupt that releases a semaphore from
`EthercatDevice::EofInterrupt()`. This implementation is thus called
SM-Synchronous.

## EtherCAT Communication

In this section various typical EtherCAT scenarios are described to show how the
implementation reacts.

### SDO

Service Data Objects (SDO) is mailbox communication, similar to remote procedure
calls. Mailbox communication is available in all states except INIT.

Every EtherCAT object defined in the object list can be accessed (read or
written) via SDO.

SDO is handled completely by SOES, with help from some callbacks. The following
approximate sequence of events is triggered:

1.  Server initiates a request (`ethercat upload -p0 0x1000 0`)
2.  Interrupt is generated on M4 to inform that an EtherCAT exchange took place
3.  ECat task wakes up and reads AL event register 0x220. Flag SM0/1 event is
    active
4.  `ESC_coeprocess()` is called, which in turn calls
    `pre_object_download_hook()` and `post_object_download_hook()` where a
    non-zero return value calls SDO abort.
5.  SOES prepares an answer and sends it back to the host.

### PDO

Process Data Objects (PDO) are typically process variables that are exchanged
between host and device on a continuous basis, usually with every EtherCAT
cycle.

As a PDO is an EtherCAT object, it is also a SDO but not the other way round.
Whether an EtherCAT object is a PDO or not is defined in the object dictionary.

During the setup phase in PREOP, the host configures, or maps, PDOs into a
SyncManager using SDO transactions. In fact SyncManager configuration simply
sets up objects 0x1C12 (SM2, Outputs, RxPDO) and 0x1C13 (SM3, Inputs, TxPDO) in
a special way.

PDOs are exchanged during SAFEOP and OP only

PDO is handled completely by SOES, with help from some callbacks. The following
rough sequence of events is triggered:

1.  Server initiates a PDO exchange 1. Interrupt is generated on M4 to inform
    that an EtherCAT exchange took place
2.  ECat task wakes up and reads AL event register 0x220. Flag SM2/3 event is
    active
3.  User calls `DIG_process()` 1. If SM2 event flag is set (outputs, RxPDO), the
    values are read into `struct _Object` struct by SOES. Thereafter
    `cb_set_outputs()` (a user function) is called to inform of the arrival of
    new data 1. User callback `application_hook()` is called
4.  User callback `cb_get_inputs()` is called and SOES copies data out of
    `struct _Object` into the EtherCAT output area to get ready for a TxPDO
    frame.
