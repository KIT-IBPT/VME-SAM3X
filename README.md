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
