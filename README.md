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

- `3`: write to 16-bit register and do not read back
- `4`: read from 32-bit register
- `5`: write to 32-bit register
- `6`: write to 32-bit register and do not read back

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
back from the register after a successful write operation, unless the “no
readback” variant of the write operation was used. If the *status* field of a 
response is non-zero, the value in the *data* field is invalid and should not
be used.


JTAG XVC Server
---------------

The firmware implements an embedded Xilinx Virtual Cable (XVC) server that can
be used to access the FPGA’s JTAG interface in order to install firmware
updates for the FPGA. This server is available on TCP port 2542.

While the SAM3X microcontroller appears on the same JTAG chain, **the XVC
server cannot be used to update the firmware on the SAM3X**. The reason is that
the XVC server runs on the SAM3X, so the JTAG connection would be lost the
moment the SAM3X is stopped in order to install the new firmware.

On power-up, the XVC server is disabled. In order to enable it, connect to the
device via Telnet (TCP port 23) and issue the `j` command in the console in
order to enable the XVC server. When it is no longer needed, the XVC server can
also be disabled by issuing this command.

The XVC server cannot be enabled when a JTAG adapter is connected to the JTAG
header on the PCB or when the device is connected to a host system via the USB
port on the front panel. The reason is that there is only a single JTAG
interface, and this interface cannot be driven by the microcontroller when it
is already driven via an external JTAG adapter or via the internal JTAG USB
adapter.

### Using openFPGALoader to Install Firmware Updates

[openFPGALoader](https://github.com/trabucayre/openFPGALoader) can be used to
install firmware updates on the FPGA’s SPI flash EEPROM via XVC. In order for
this to work, you need at least version 1.1.0 of openFPGALoader, and you most
likely want to use a version where a
[patch significantly improving the XVC performance](
https://github.com/trabucayre/openFPGALoader/pull/632) is also included.
Without this patch, the process is extremely slow (taking many hours).

You can first check that JTAG communication is working by detecting the chain:

```sh
openFPGALoader --cable xvc-client --ip 192.0.2.1 --port 2542 --detect
```

The IP address 192.0.2.1 has to be replaced with the actual IP address of the
device where the update is supposed to be installed. At the time of writing, an
IP adress must be specified as openFPGALoader does not support the use of DNS
names.

For the VME-EVR-300, running this command should result in an output like this:

```
empty
detected xvcServer version v1.0 packet size 512
freq 6000000 166.666667 166 0
b2 2 0 0
index 0:
	idcode   0x4ba00477
	type     ARM cortex A9
	irlength 4
index 1:
	idcode 0x3647093
	manufacturer xilinx
	family kintex7
	model  xc7k70t
	irlength 6
```

Next, you can install the firmware image:

```sh
openFPGALoader --cable xvc-client --ip 192.0.2.1 --port 2542 \
  --index-chain 1 --fpga-part xc7k70tfbg676 --write-flash --bulk-erase \
  VME-EVR-300-12160207.bit
```

The name of the `.bit` file has to be adjusted to match the name of the
firmware file that shall actually be installed, of course.

At the time of writing, SPI over JTAG support for the XC7K325TFBG900, which is
used on the VME-EVM-300 is not available in openFPGALoader yet. However, there
is a [pull request](https://github.com/trabucayre/openFPGALoader/pull/635)
adding support. When support is available, the following command should work:

```sh
openFPGALoader --cable xvc-client --ip 192.0.2.1 --port 2542 \
  --index-chain 1 --fpga-part xc7k325tfbg900 --write-flash --bulk-erase \
  VME-EVM-300-22110207.bit
```

### Connecting with Xilinx ISE iMPACT

When using Xilinx ISE iMPACT to connect to the XVC server, you have to select
“Open Cable Plug-in” in the cable setup dialog and enter the following string:

```
xilinx_xvc host=192.1.2.0 disableversioncheck=true maxpacketsize=512
```

Like for openFPGALOader, 192.1.2.0 has to be replaced with the actual IP
address of the device.
