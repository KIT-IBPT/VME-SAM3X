VME-SAM3X firmware
==================

Tools required for building on Ubuntu 24.04 LTS:
- gcc-arm-none-eabi
- libnewlib-arm-none-eabi
- make

When installing the tools needed for building, you might want to specify
`--no-install-recommends` in order to avoid also installing the C++ standard
library for ARM, which is very large and not needed by this project. For
example:

```sh
apt-get install --no-install-recommends \
  gcc-arm-none-eabi libnewlib-arm-none-eabi make
```

Tools required for installing the firmware image:
- bossa

Building
--------

```sh
make clean; make all
```

Installation
------------

- Power up VME-EVM-300 or VME-EVR-300 with jumper installed in SAM Erase
  header.
- Remove jumper and power cycle.
- Connect USB cable to USB port in front panel.
- Run `bossac -e -w -v -b -p /dev/ttyUSB1 build/EVM_300_UDP.bin`
- Power cycle.

There is an alternative method for installing the firmware image, but it is
only easier when one has already setup debugging (OpenOCD must be running
and connected to the target device):

```sh
make install
```


Debugging
---------

GDB and OpenOCD can be used in order to debug (and install) the firmware. For
this, you have to install two additional packages:

- gdb-arm-none-eabi
- openocd

Connect a USB cable to the USB port in the front panel and then run:

```sh
openocd -f OpenOCD/mrf-vme-usb.cfg -f OpenOCD/vme-evm-300.cfg
```

(for the VME-EVM-300) or

```sh
openocd -f OpenOCD/mrf-vme-usb.cfg -f OpenOCD/vme-evr-300.cfg
```

(for the VME-EVR-300).

Open a separate terminal and ensure that the correct firmware image is loaded
(GDB cannot work correctly if the running image and the image used by GDB
differ). In case of doubt, run `make install` to install the built image.
Subsequently, run the following command to launch GDB and connect it to the
device.

```sh
make debug
```

This assumes that OpenOCD is running on the same host as GDB and listening for
a connection from GDB on port 3333. If this does not apply, you have to modify
`.gdbinit`.
