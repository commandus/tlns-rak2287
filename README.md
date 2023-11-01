# tlns-rak2287

## Build

Make sure CMake, Git, essential build tools are installed.

Clone repository:
```
git clone https://github.com/commandus/tlns-rak2287.git
```

Check path to RAK2287 libloragw lib sources in the CMakeLists.txt:
```
vi CMakeLists.txt
```

Find line

```
set(LIBLORAGW_ROOT ../../libloragw)
```

and replace with actual path of libloragw root directory.

Build library:

```
cd libloragw
mkdir build
cd build
cmake ..
make
```

Dependencies:

- (https://github.com/commandus/libloragw])[https://github.com/commandus/libloragw.git]
- (https://github.com/RAKWireless/rak_common_for_gateway])[https://github.com/RAKWireless/rak_common_for_gateway.git]