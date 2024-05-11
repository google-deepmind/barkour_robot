# EtherCAT Configuration (Host Side)

[TOC]

This document explains how to install and troubleshoot the EtherCAT bus.

## EtherCAT Drivers

The EtherLab library must be installed on the computer connected to the EtherCAT
bus, which is typically on the robot's onboard computer. We use EtherLab, which
is a Linux kernel-based library that implements an EtherCAT host. See
[here](https://www.etherlab.org/en/ethercat/index.php) for details.

### EtherLab Installation Instructions

The following outlines how to install the EtherLab host from source. For more
comprehensive instructions, see the
[EtherLab documentation](https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf).

1.  Install the required libraries and kernel headers.

    ```
    sudo apt install autoconf libtool build-essential git pkg-config vim ethtool cpuset
    sudo apt-get install linux-headers-`uname -r`
    ```

1.  Clone the repository and enter the directory.

    ```
    git clone https://gitlab.com/etherlab.org/ethercat.git
    cd ethercat
    ```

1.  Check out the stable version.

    ```
    git checkout stable-1.5
    ```

1.  Bootstrap and configure the toolchain; disable the `8139too` driver, which
    is not required/supported, and enable the generic driver.

    ```
    ./bootstrap
    ./configure --disable-8139too --enable-generic
    ```

    `bootstrap` might give an error, asking you to run `autoupdate`. In this
    case, you should run `autoupdate` followed by `bootstrap` again.

1.  Build the userland library and kernel modules.

    ```
    make && make modules
    ```

1.  Install the user land library and kernel modules.

    ```
    sudo make install && sudo make modules_install
    sudo depmod
    ```

1.  Create symlinks for the EtherCAT configuration. The current version of
    EtherLab also tries to use `sh`, which is problematic on many machines. We
    use `bash` instead:

    ```
    sudo mkdir -p /etc/sysconfig/
    sudo ln -s /usr/local/etc/init.d/ethercat /etc/init.d/ethercat
    sudo cp /usr/local/etc/sysconfig/ethercat /etc/sysconfig/ethercat
    sed '1 s/^.*$/#!\/bin\/bash/' /etc/init.d/ethercat  | sudo tee /etc/init.d/ethercat > /dev/null
    ```

1.  Edit the configuration file to point it to the correct NIC and enable the
    generic driver module. First, find the MAC address of the NIC you would like
    to use for EtherCAT, e.g., using `ip link`. Then with your favourite editor,
    edit the file `/etc/sysconfig/ethercat` to set up the configuration. For
    example, if the NIC you're using has MAC address `aa:bb:cc:dd:ee:ff`, edit
    the file to look like:

    ```
    ...

    # Main Ethernet devices
    ...

    MASTER0_DEVICE="aa:bb:cc:dd:ee:ff"
    #MASTER1_DEVICE=""

    ...

    # Backup Ethernet devices

    ...

    #MASTER0_BACKUP=""

    ...

    # Ethernet driver modules to use for EtherCAT operation

    ...

    DEVICE_MODULES="generic"

    ```

1.  (Re)start the host. This reloads the kernel modules and starts the kernel
    threads that process EtherCAT data.

    ```
    sudo /etc/init.d/ethercat restart
    ```

    Tip: If you get an error like `FATAL: Module ec_master not found in
    directory /lib/modules/5.13.0-44-generic`, your kernel has likely been
    updated since the last install and the kernel module version no longer
    matches your kernel version. Delete the `ethercat` folder, go back to step 1
    and try again. If the problem persists, reboot and try running `sudo
    /etc/init.d/ethercat restart` once more.

1.  Allow non-root access to the EtherCAT port:

    ```
    sudo sh -c "echo 'KERNEL==\"EtherCAT[0-9]*\", MODE=\"0666\"' > /etc/udev/rules.d/99-EtherCAT.rules"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

To check if the install has worked, you should be able to see a summary of the
host status by running `ethercat master`, which gives an output like:

```
$ ethercat master

Master0
  Phase: Idle
  Active: no
  Slaves: 0
  Ethernet devices:
    Main: aa:bb:cc:dd:ee:ff (attached)
      Link: UP
      Tx frames:   0
      Tx bytes:    0
      Rx frames:   0
      Rx bytes:    0
      Tx errors:   0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
    Common:
      Tx frames:   388038
      Tx bytes:    65393294
      Rx frames:   388038
      Rx bytes:    65393294
      Lost frames: 0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.1
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.1
      Loss rate [1/s]:          0      0      0
      Frame loss [%]:         0.0  100.0    1.7
  Distributed clocks:
    Reference clock:   None
    DC reference time: 0
    Application time:  0
                       2000-01-01 00:00:00.000000000
```

If you attach a motor module to the EtherCAT bus and power it on, then you can
also check the output of `ethercat slaves`, which should show something like
this:

```
$ ethercat slaves
0  1:0  PREOP  +  Cortes
```

## Configuring EtherCAT Devices (Motor Drivers) On the Bus

When installing new motor drivers, it's important to assign the correct EtherCAT
device alias to each driver to ensure the correct joint order in software. In
our typical configuration, the physical EtherCAT motor order is different from
the logical one for historical reasons. The physical to logical mapping is shown
below.

| **Joint | **Position on ECAT | **ECAT device | **Joint name**          |
: index** : bus**              : alias**       :                         :
| :-----: | :----------------: | :-----------: | :---------------------: |
| 0       | 0                  | 1             | abduction\_front\_left  |
| 1       | 1                  | 2             | hip\_front\_left        |
| 2       | 2                  | 3             | knee\_front\_left       |
| 3       | 9                  | 4             | abduction\_hind\_left   |
| 4       | 10                 | 5             | hip\_hind\_left         |
| 5       | 11                 | 6             | knee\_hind\_left        |
| 6       | 3                  | 7             | abduction\_front\_right |
| 7       | 4                  | 8             | hip\_front\_right       |
| 8       | 5                  | 9             | knee\_front\_right      |
| 9       | 6                  | 10            | abduction\_hind\_right  |
| 10      | 7                  | 11            | hip\_hind\_right        |
| 11      | 8                  | 12            | knee\_hind\_right       |

Note: Bus positions use 0-based indexing (0-11), whereas aliases use 1-based
indexing (1-12) 🤷.

Use the `ethercat alias` command to set the correct device aliases:

```bash
ethercat alias -p0 1
ethercat alias -p1 2
ethercat alias -p2 3
ethercat alias -p9 4
ethercat alias -p10 5
ethercat alias -p11 6
ethercat alias -p3 7
ethercat alias -p4 8
ethercat alias -p5 9
ethercat alias -p6 10
ethercat alias -p7 11
ethercat alias -p8 12
```

## Flashing New Firmware

See [this page](firmware.md) for instructions.

## Restarting Motor Drivers Via EtherCAT

Important: This feature is only supported on the Holberton version of
the motor drivers. If you run this tool on the Gebru + Cortes
version, the motor driver will be held in reset until you (manually) power cycle
the motors.

It is possible to send a sequence of low-level EtherCAT commands to reset motor
drivers on the bus. This will toggle the reset pins on both the EtherCAT chip
(Trinamic) and the main microcontroller (STM32H7). This is equivalent to
pressing the reset button (power cycle) on the board or the Mataric debugger.

This is implemented as a standalone binary in the
[reset_motor_driver](../actuator/ethercat_host/reset_motor_driver.cc) example.

This script uses the [SOEM](https://github.com/OpenEtherCATsociety/SOEM) library
instead of EtherLab in order to send individual EtherCAT frames. To run it on
the robot, stop the Etherlab kernel module, run the script, then restart the
Etherlab kernel module. The `reset_motor_driver` binary should have permission
to access raw sockets.

```bash
sudo setcap CAP_NET_RAW+pe reset_motor_driver
sudo systemctl stop ethercat
./reset_motor_driver --alsologtostderr --ethercat_interface=enx7cc2c6473b62
sudo systemctl start ethercat
```

## Common Error Conditions

Below are details of some common error messages that may be encountered when
using the hardware.

**Boards come up in error state, for example: `(3 7:0 INIT E Cortes)`**

When running `ethercat slaves`, one or more motor drivers show up as faulted:

```
3   7:0  INIT   E  Cortes
```

or have incorrect information:

```
11   6:0  INIT   +  0x00600613:0x00001000
```

This usually indicates that either the EEPROM on the EtherCAT chip or the flash
memory on the microcontroller is corrupted. We have noticed the former in case
of brownout conditions. See the
[actuator assembly and setup](actuator_assembly_and_setup.md) documentation to
reflash the EEPROM (SII) and the [firmware](firmware.md) page to reflash the
embedded software.

**Unexpected working counter received from the EtherCAT bus**

When an EtherCAT frame is sent around the bus by the host, each device (e.g. a
motor driver) on the bus increments the working counter of each datagram in the
frame that it interacts with. The increment amounts depend on the action that
the device is commanded to carry out, but in the usual case (device is commanded
to read and write from the datagram), the working counter is incremented by +1
for a successful read and by +2 for a successful write. Hence, the working
counter should be equal to 3 times the number of devices on the bus. A mismatch
indicates a communication failure with at least 1 device on the bus.

There are various reasons this can happen, including:

-   A lost or unreliable physical connection to a device
-   Incorrect behaviour of device firmware
-   Devices not having enough time to process datagrams, due to an overly high
    EtherCAT cycle frequency or excessive jitter on the bus

Depending on the characteristics of the host system (e.g. if it's running an RT
kernel, or CPU isolation for the userland process), some working counter errors
may be expected due to jitter.

**More than one device found on EtherCAT bus matching device address**

The most likely cause of this is that a device alias is incorrectly configured,
causing two devices on the bus to have the same alias. This might occur after
swapping a motor or reflashing the SII.

You can use the `ethercat slaves` command on the computer connected to the
EtherCAT bus to verify the device order. Once you identify the incorrectly
configured device, use the `ethercat alias` command to fix it.

Example incorrect configuration, followed by a fix:

```
barkour@barkour:~$ ethercat slaves
 0   1:0  PREOP  +  Cortes
 1   2:0  PREOP  +  Cortes
 2   3:0  PREOP  +  Cortes
 3   7:0  PREOP  +  Cortes
 4   8:0  PREOP  +  Cortes
 5   9:0  PREOP  +  Cortes
 6  10:0  PREOP  +  Cortes
 7  11:0  PREOP  +  Cortes
 8  12:0  PREOP  +  Cortes
 9   4:0  PREOP  +  Cortes
10   5:0  PREOP  +  Cortes
11   1:0  PREOP  +  Cortes  # Duplicate alias 1 (should be alias 6).
barkour@barkour:~$ ethercat alias -p11 6  # Set motor driver at position 11 to use alias 6
barkour@barkour:~$ ethercat slaves
 0   1:0  PREOP  +  Cortes
 1   2:0  PREOP  +  Cortes
 2   3:0  PREOP  +  Cortes
 3   7:0  PREOP  +  Cortes
 4   8:0  PREOP  +  Cortes
 5   9:0  PREOP  +  Cortes
 6  10:0  PREOP  +  Cortes
 7  11:0  PREOP  +  Cortes
 8  12:0  PREOP  +  Cortes
 9   4:0  PREOP  +  Cortes
10   5:0  PREOP  +  Cortes
11   6:0  PREOP  +  Cortes  # Correct alias.
```

