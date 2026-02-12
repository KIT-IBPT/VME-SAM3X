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


Network Protocol
----------------

The firmware for the SAM3X implements a UDP/IP server that listens on UDP
port 2000. There are two versions of the protocol: The older version
(version 1) is identical to the one that is supported by older devices from the
VME-EVG-230 and VME-EVR-230 series that use an IP2022 instead of a SAM3X. The
newer version (version 2) is exclusively supported by recent versions of the
firmware for the SAM3X and adds support for accessing 32-bit registers in a
single round-trip.

Packets for protocol version 1 have the following structure:

|Field type|Field length (in bytes)|Description|
|----------|-----------------------|-----------|
|uint8     |                      1|access type|
|int8      |                      1|status     |
|uint16    |                      2|data       |
|uint32    |                      4|address    |
|uint32    |                      4|reference  |

Packets for protocol version 2 have the following structure:

|Field type|Field length (in bytes)|Description|
|----------|-----------------------|-----------|
|uint8     |                      1|access type|
|int8      |                      1|status     |
|uint16    |                      2|reserved   |
|uint32    |                      4|address    |
|uint32    |                      4|reference  |
|uint32    |                      4|data       |

All fields use network byte-order (big endian).

The *access type* field specifies the action that is taken. Protocol version 1
supports two access types:

- `1`: read from 16-bit register
- `2`: write to 16-bit register

Protocol version 2 supports two additional access types:

- `3`: read from 32-bit register
- `4`: write to 32-bit register

It is still possible to read from and write to 32-bit registers using version 1
of the protocol, but this requires two round-trips: For read operations, the
low word (starting at the register address plus two bytes) should be read
first, and the high word (the two bytes starting at the register address)
should be read second. For write operations, the high word should be written
first, and the low word should be written second (after the first write
operation has succeeded).

The *status* field is only used in replies to requests and should be set to
zero in requests. It can have the following values:

- `0`: Operation succeeded.
- `-1` (`0xFF`): Invalid address specified (this is never returned by the
  current version of the firmware).
- `-2` (`0xFE`): FPGA did not respond in time (timeout).
- `-3` (`0xFD`): Invalid command (the *access type*  field contains a value
  that is not supported for the respective version of the protocol).

The *reserved* field in version 2 of the protocol is not used at the moment and
should be set to zero in requests.

The *address* field specifies the address of the requested register inside the
FPGA. Depending on the address space that shall be accessed the following
offsets need to be applied (please note that these values are only valid for
the VME-EVM-300 and VME-EVR-300, other devices may use different offsets):

Offset    |Address space
----------|------------------------
0x00000000|VME CR/CSR address space
0x80000000|EVM or EVR address space

The *reference* field in a request can contain an arbitrary number chosen by
the client. Each response contains the *reference*  value from the respective
request, so clients can use this field to associate responses with requests.

The *data* field contains the value that is supposed to be written (for
write requests) or the value that has been read (for responses). For read
requests, the value in this field is ignored. For responses to write requests,
this is not the value that has been written but rather the value that was read
back from the register after a successful write operation. If the *status*
field of a response is non-zero, the value in the *data* field is invalid and
should not be used.
