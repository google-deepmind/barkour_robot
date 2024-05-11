# Motor Control

We provide a C++ example to connect to the motors via EtherCAT and send
torque/position commands to the actuators.

The implementation can be found at `actuator/ethercat_host/ethercat_example.cc`.

Important: This example does not include safety features, so make sure to refine
it before using it in a real system. Please also review the
[basic robot instructions](getting_started.md).

## Running the EtherCAT Host Side Example

Note: To connect to hardware, make sure the Etherlab kernel module has been
installed and the motor drivers are showing up on the bus. See
[EtherCAT config](ethercat_config.md) for instructions. Make sure you are using
the same version for the kernel module and the user mode code below.

Note: Instructions to compile and flash the firmware can be found
[here](firmware.md).

Note: We are using [Bazel](https://bazel.build) as the build system. The
instructions below were tested with `bazel 7.1.2` on Debian and Ubuntu.

As we are using Etherlab to communicate via the EtherCAT bus, we clone the
Etherlab repo and configure it (only needed once):

```bash
cd actuator/ethercat_host

# Download and configure the EtherLab library.
sudo apt install autoconf libtool build-essential git pkg-config vim ethtool cpuset
sudo apt-get install linux-headers-`uname -r`
rm -Rf etherlab
git clone https://gitlab.com/etherlab.org/ethercat.git etherlab
cd etherlab
git checkout stable-1.5
./bootstrap
# We run ./configure to generate the config.h file.
./configure --disable-8139too --enable-generic
```

Now you can compile the example and run it:

```bash
# Compile and run the example
cd ../..
bazel build --cxxopt='-std=c++17' -c opt ethercat_host:ethercat_example
bazel-bin/ethercat_host/ethercat_example --stderrthreshold=0
```

The expected behavior is that all actuators are held in place and that one joint
slowly wiggles (the actuator with device alias 1).

To give the example a try without hardware, just add the `fake_ethercat_host`
flag:

```bash
bazel-bin/ethercat_host/ethercat_example --stderrthreshold=0 --fake_ethercat_host
```
