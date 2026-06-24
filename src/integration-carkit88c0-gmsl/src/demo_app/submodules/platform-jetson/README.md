# NVIDIA Jetson platform implementation

## Contents

This repository contains the platform implementation for accessing hardware on Nvidia Jetson platform.

## Usage

Use this repository as a submodule in other projects and use the functionality as described below in the details. For some functionality additional libraries have to be installed. See the details below.

## Detailed Description

### Util

Utilities used for platform implementation. This contains the _DEBUG_ flag which, when defined, activates verbose debug logging inside the platform functions.

### Logging

Logging is currently implemented just using _printf_. Some implementations suggest this should be implemented in an async way, which is currently not the case.

### Time

Timing functionality is implemented using Linux's _CLOCK_MONOTONIC_ clock, which is recommended for measuring elapsed time.

### GPIO

GPIO access from userspace was implemented using _libgpiod_ version 1.6. Currently, version 2 is already available but it made great API changes compared to version 1, and is additionally not available in the official repositories of any supported Ubuntu distribution (20.04 on Jetpack 5, 22.04 on Jetpack 6). Hence, version 1.6 was selected for implementation.

As the program manages the reserved GPIO lines, resources should be properly freed by calling _PlatfromGpio_deinit()_ before exiting.

#### Installation on Ubuntu 20.04 (Jetpack 5)

Ubuntu 20.04 only provides _libgpiod_ version 1.4 via the repository, so version 1.6 needs to be build from source using the following procedure:

```
# Update package repository
sudo apt update

# Install prerequisites
sudo apt install git build-essential autoconf automake autoconf-archive libtool pkg-config

# Clone libgpiod sources at correct branch
git clone --branch v1.6.x https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git

cd libgpiod

# Configure installation
./autogen.sh --enable-tools=yes

# Build
make -j$(nproc)

# Install
sudo make install
```

#### Installation on Ubuntu 22.04 (Jetpack 6)

Install _libgpiod_ (v1.6.3) from the repository:

```
sudo apt update
sudo apt install libgpiod-dev
```

### SPI

SPI is implemented using the _spidev_ Linux driver to access the bus from userspace. A speciality here is that chip-selects are, where possible, controlled as GPIO pins. This is required as the iRFE driver requires the processing of responses within one SPI frame (i.e.: keeping CS asserted). This can be seen in the code snippet [here](https://gitlab.intra.infineon.com/ifx/pss/fwswgrz/projects/ctrx/ctrx-irfe-driver/-/blob/develop/src/IfxRfe_Spi.c?ref_type=heads#L52-83). The Linux driver although treats each transaction (read, write or both) as atomic operation and does not to keep CS asserted afterwards. Hence, the workaround was to control the CS manually as GPIO.

As the program manages SPI devices internally, resources should be properly freed by calling _PlatfromSpi_deinit()_ before exiting.

### I2C

I2C functionality is provided via SMBus. Requires the package _libi2c-dev_ to work:

```
sudo apt update
sudo apt install libi2c-dev
```
