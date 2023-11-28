# tlns-rak2287

Silent LoRaWAN listener.

It prints received LoRaWAN messages to the stdout.

You need RAK2287 USB gateway to listen LoRaWAN messages on air. 

## Build

You can use

- Automake
- CMake

build system.

Clone repository:

```
git clone https://github.com/commandus/tlns-rak2287.git
```

### Prerequisites

Make sure automake/CMake, Git, essential build tools are installed:

```
apt install autoconf libtool build-essential
```

You need RAK2287 library (loragw).

Before you start, first you need clone [loragw library](https://github.com/commandus/loragw.git).

Check path to RAK2287 libloragw lib sources in the CMakeLists.txt:

```
vi CMakeLists.txt
```

Find line

```
set(LIBLORAGW_ROOT ../../libloragw)
```

and replace with actual path of libloragw root directory.

Build loragw library:

```
cd libloragw
mkdir build
cd build
cmake ..
make
```

Optionally make custom config/gateway_usb_conf.h header file using gateway-config2cpp tool:

```
./gateway-config2cpp -h ~/src/rak_common_for_gateway/lora/rak2287/global_conf_usb/* > gateway_usb_conf.
```

or edit config/gateway_usb_conf.h manually.

Dependencies:

- (https://github.com/commandus/libloragw)[https://github.com/commandus/libloragw.git]
- (https://github.com/RAKWireless/rak_common_for_gateway)[https://github.com/RAKWireless/rak_common_for_gateway.git]

### Build using Autotools

Generate automake files, configure and make (gcc):

```
cd tlns-rak2287
./autogen.sh
./configure
make
strip tlns-rak2287
```

For clang:

```
./configure CC=clang CXX=clang++
```

### Build using CMake

```
cd tlns-rak2287
mkdir -p build
cd build
cmake ..
make
strip tlns-rak2287
```

You can use Clang instead of gcc:

```
mkdir -p build
cd build
export CC=/usr/bin/clang;export CXX=/usr/bin/clang++;cmake ..
make
strip tlns-rak2287
```
