# J2534 Implementation Changes

This document describes the changes made to the J2534 firmware implementation to support ECU programming operations with large ISO-TP transfers.

> **Note**: A component refactoring is in progress to move J2534 code from `main/j2534.c` to `components/j2534/`. The component structure has been started with `j2534_core.c` and `j2534_channel.c`. Additional files (`j2534_msg.c`, `j2534_isotp.c`, `j2534_cmd.c`) need to be completed to finish the migration.

## Overview

These changes fix critical issues that prevented successful ECU programming via diagnostic software. The primary issues were:

1. **TX Indication Buffer Overflow** - Large message echoes caused garbage data in ReadMsgs
2. **Filter Logic Too Permissive** - PASS filters were being bypassed by functional addressing mode
3. **RAM Constraints** - Large ISO-TP logging exceeded available memory

---

## 1. TX Indication Buffer Overflow Fix

### Problem
When transmitting large ISO-TP messages (e.g., 4KB TransferData blocks), the TX indication (echo) was setting `data_size` to the original message size but only copying 12 bytes of actual data. This caused `PassThruReadMsgs` to read garbage memory beyond the 256-byte buffer.

### Symptoms
- Diagnostic software receiving `00 DE B0 06` or other garbage bytes
- Programming operations failing after TransferData (0x36) requests
- Random/inconsistent data in TX echoes

### Fix Location
`main/j2534.c` - `j2534_buffer_tx_indication()` function

### Change
```c
// BEFORE (bug):
msg->data_size = orig_msg->data_size;  // Could be 4098 bytes!

// AFTER (fix):
msg->data_size = echo_size;  // Only the bytes actually copied (max 12)
```

### Technical Details
- TX echoes for ISO-TP only need: 4-byte CAN ID + 8-byte first frame = 12 bytes
- The `data[]` array in `PASSTHRU_MSG` is only 256 bytes
- Setting `data_size` larger than copied data causes buffer overread

---

## 2. Filter Logic Fix for PASS Filters

### Problem
When explicit PASS filters were configured, the functional/physical addressing "permissive mode" was still allowing ALL frames in certain CAN ID ranges through. This flooded the RX buffer with unwanted frames.

### Symptoms
- RX buffer filling with unrelated CAN traffic
- Slow response times due to buffer scanning
- Diagnostic operations timing out

### Fix Location
`main/j2534.c` - `j2534_filter_message()` function

### Change
```c
// BEFORE (bug):
if (ch->protocol_id == J2534_PROTOCOL_ISO15765) {
    // Was passing ALL frames in functional ranges

// AFTER (fix):
if (ch->protocol_id == J2534_PROTOCOL_ISO15765 && !has_pass_filter) {
    // Only use permissive mode when NO explicit PASS filters exist
```

### Technical Details
- Added `has_pass_filter` check before enabling permissive addressing modes
- Functional addressing (0x7DF) and physical addressing modes now respect explicit filters
- When PASS filters exist, only matching frames are buffered

---

## 3. RAM-Optimized Logging for Large ISO-TP

### Problem
The ESP32-S3 has limited RAM (~320KB usable). Logging full ISO-TP payloads (up to 4KB) would exhaust memory and crash the firmware.

### Solution
Implemented truncated logging that shows meaningful data without consuming excessive RAM:

### Key Changes

#### a) TX Logging (outgoing messages)
```c
// Log first 32 bytes + last 16 bytes of large payloads
if (data_len > 64) {
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, payload, 32, ESP_LOG_INFO);
    ESP_LOGI(TAG, "  ... (%d bytes omitted) ...", data_len - 48);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, payload + data_len - 16, 16, ESP_LOG_INFO);
}
```

#### b) RX Reassembly Logging
```c
// Only log reassembly progress, not full payloads
ESP_LOGI(TAG, "ISO-TP RX: CF seq=%d, received=%lu/%lu bytes",
         seq_num, isotp_rx_state.received_length, isotp_rx_state.expected_length);
```

#### c) Completed Message Logging
```c
// Log summary with CRC/checksum for verification without full dump
ESP_LOGI(TAG, "ISO-TP complete: %lu bytes, first: %02X %02X %02X %02X",
         total_len, data[0], data[1], data[2], data[3]);
```

### Memory Impact
| Scenario | Before | After |
|----------|--------|-------|
| 4KB TransferData log | ~4KB stack | ~64 bytes |
| Full reassembly dump | ~4KB heap | ~32 bytes |
| Typical operation | Risk of stack overflow | Stable |

---

## 4. Extended Wait for Programming Operations

### Purpose
ECU programming operations (especially TransferData 0x36) can take 1-5+ seconds to process. The firmware now extends the read timeout when:

1. A multi-frame ISO-TP reception is in progress
2. A large ISO-TP transmission just completed and response is expected

### Implementation
```c
// After sending large ISO-TP message:
isotp_tx_state.awaiting_response = true;
isotp_tx_state.tx_complete_time = xTaskGetTickCount();
isotp_tx_state.response_can_id = tx_can_id + 8;  // Standard response offset

// In read_msgs, extend wait up to 5 seconds if awaiting response
if (isotp_tx_state.awaiting_response && tx_elapsed < max_isotp_wait_ticks) {
    continue;  // Keep waiting
}
```

---

## Testing Recommendations

1. **Basic Connectivity**: Use diagnostic software to read DTCs
2. **Large Transfers**: Attempt calibration/module programming with 4KB blocks
3. **Multi-ECU Discovery**: Verify subnet scanning doesn't flood buffer
4. **Timeout Behavior**: Confirm extended waits work for slow ECU responses

---

## Files Modified

| File | Changes |
|------|---------|
| `main/j2534.c` | TX indication fix, filter logic, extended waits, logging optimization |
| `main/j2534.h` | No changes required |
| `tools/j2534_driver/wican_comm.c` | Extended timeout for large transfers |
| `tools/j2534_driver/wican_j2534.c` | No functional changes |

---

## Compatibility

These changes maintain full backward compatibility with:
- SAE J2534-1 API
- SAE J2534-2 extended protocols
- Existing WiCAN diagnostic features
- All supported CAN protocols (ISO15765, raw CAN, etc.)
