# WiCAN J2534 Pass-Thru Driver

A SAE J2534-1 compliant pass-thru driver for the WiCAN Pro device.

## Overview

This driver allows J2534-compatible diagnostic software to communicate with vehicles through the WiCAN Pro device over WiFi.

## Supported Features

- **Protocols:**
  - CAN (Controller Area Network)
  - ISO 15765 (CAN with OBD-II)

- **Baud Rates:**
  - 125 kbps
  - 250 kbps
  - 500 kbps
  - 1000 kbps (1 Mbps)

- **Filter Types:**
  - Pass filters
  - Block filters
  - Flow control filters (ISO 15765)

## Building the Driver

### Prerequisites

1. **Windows 10/11**
2. **Visual Studio 2022** with C++ Desktop Development workload
   - Or Visual Studio Build Tools
3. **CMake 3.15+**

### Build Steps

1. Open Command Prompt or PowerShell
2. Navigate to the driver directory:
   ```cmd
   cd tools\j2534_driver
   ```
3. Run the build script:
   ```cmd
   build.bat all
   ```

This will create:
- `build32\Release\wican_j2534_32.dll` (32-bit driver)
- `build64\Release\wican_j2534_64.dll` (64-bit driver)

### Manual Build with CMake

```cmd
# 32-bit build
mkdir build32
cd build32
cmake -G "Visual Studio 17 2022" -A Win32 ..
cmake --build . --config Release

# 64-bit build
mkdir build64
cd build64
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release
```

## Installation

### Automatic Installation

1. Create installation directory:
   ```cmd
   mkdir "C:\Program Files\WiCAN"
   ```

2. Copy DLL files:
   ```cmd
   copy build32\Release\wican_j2534_32.dll "C:\Program Files\WiCAN\"
   copy build64\Release\wican_j2534_64.dll "C:\Program Files\WiCAN\"
   ```

3. Register the driver (run as Administrator):
   ```cmd
   regedit /s register_driver.reg
   ```

### Manual Registration

If you need to customize the installation path, modify `register_driver.reg` before running it.

## Usage

### Connection Setup

1. **Power on the WiCAN Pro device**
2. **Connect your PC to the WiCAN WiFi network**
   - SSID: `WiCAN_xxxxxx` (device-specific)
   - Default IP: 192.168.80.1
3. **Launch your J2534-compatible software**
4. **Select "WiCAN J2534" as the interface**

### PassThruOpen Parameters

When calling `PassThruOpen()`, you can optionally specify the device IP address:

```c
// Default connection (192.168.80.1:3333)
PassThruOpen(NULL, &deviceID);

// Custom IP address
PassThruOpen("192.168.1.100", &deviceID);
```

### Supported IOCTLs

| IOCTL ID | Description |
|----------|-------------|
| GET_CONFIG | Read configuration parameters |
| SET_CONFIG | Set configuration parameters |
| READ_VBATT | Read battery voltage (nominal) |
| CLEAR_TX_BUFFER | Clear transmit buffer |
| CLEAR_RX_BUFFER | Clear receive buffer |
| CLEAR_MSG_FILTERS | Clear all message filters |

## Compatible Software

This driver should work with any SAE J2534-1 compliant software, including:

- Ford IDS/FDRS
- GM GDS2/TIS2Web
- Toyota Techstream
- Honda HDS
- FCA wiTECH
- ScanTool OBD-II Software
- PyOBD
- ELM327 compatible tools (with J2534 mode)

## Troubleshooting

### Driver Not Found

1. Verify the DLL is in `C:\Program Files\WiCAN\`
2. Run `register_driver.reg` as Administrator
3. Check registry entries exist at:
   - `HKEY_LOCAL_MACHINE\SOFTWARE\PassThruSupport.04.04\WiCAN`

### Connection Failed

1. Ensure WiCAN device is powered on
2. Verify PC is connected to WiCAN WiFi
3. Ping the device: `ping 192.168.80.1`
4. Check Windows Firewall settings

### Message Timeout

1. Verify vehicle is turned on (ignition)
2. Check CAN bus connections
3. Try different baud rates (500k is common for OBD-II)

### 32-bit vs 64-bit

- Use the 32-bit DLL for 32-bit applications
- Use the 64-bit DLL for 64-bit applications
- Both can be installed simultaneously

## Technical Details

### Communication Protocol

The driver communicates with WiCAN Pro using a binary protocol over TCP:

```
Packet Format:
[SYNC1][SYNC2][CMD][LEN_H][LEN_L][DATA...][CHECKSUM]

SYNC1: 0x55
SYNC2: 0xAA
CMD:   Command ID
LEN:   Data length (big endian)
DATA:  Variable length payload
CHECKSUM: XOR of all bytes from CMD to end of DATA
```

### Thread Safety

The driver is thread-safe and uses critical sections to protect shared resources. Multiple threads can call J2534 functions concurrently.

## Version History

### 1.0.0

- Initial release
- CAN and ISO 15765 protocol support
- Basic filter functionality
- IOCTL support

## License

This driver is part of the WiCAN Pro firmware project.

## Support

For issues and feature requests, please open an issue on the project repository.
