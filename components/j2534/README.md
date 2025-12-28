# J2534 Component

SAE J2534 Pass-Thru Protocol Implementation for WiCAN.

## File Structure

| File | Lines | Description |
|------|-------|-------------|
| `j2534_core.c` | 429 | Global state, initialization, device management, helpers |
| `j2534_channel.c` | 477 | Connect/disconnect, filter management |
| `j2534_msg.c` | 424 | Read/write messages, TX indication buffering |
| `j2534_isotp.c` | 483 | ISO-TP transport layer, CAN frame conversion |
| `j2534_ioctl.c` | 269 | IOCTL operations (GET/SET_CONFIG, etc.) |
| `j2534_cmd.c` | 482 | Binary protocol parser, command dispatch |

## Headers

- `include/j2534.h` - Public API (error codes, protocol IDs, message structures)
- `include/j2534_internal.h` - Internal types and shared state declarations

## Key Features

- Full J2534-1 API support (Open, Close, Connect, Disconnect, ReadMsgs, WriteMsgs, etc.)
- ISO 15765-2 (ISO-TP) transport layer with multi-frame support
- CAN/CAN-FD and legacy protocol support (ISO9141, ISO14230, J1850)
- Flow control filter handling for ECU programming
- Functional and physical addressing modes
- TX indication echo for programming operations

## Bug Fixes in This Version

1. **TX Indication Buffer Overflow** - Fixed data size calculation in echo messages
2. **Filter Permissive Mode** - Only bypass filters when no explicit PASS filters exist

## Migration Status

This component was refactored from the original monolithic `main/j2534.c` (2856 lines)
into smaller, more maintainable files (all under 600 lines each).
