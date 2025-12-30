/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2025 Matt Deering
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file j2534_core.c
 * @brief J2534 Core - Global variables, initialization, device management
 *
 * This file contains:
 * - Global state variables (defined here, extern in j2534_internal.h)
 * - Initialization and reset functions
 * - Device open/close functions
 * - Helper functions used across the component
 * - Periodic message task
 * - PSRAM buffer allocation for improved ECU programming performance
 */

#include "j2534_internal.h"
#include "can.h"
#include "stn_j2534.h"

#define TAG J2534_TAG

/* ============================================================================
 * Global Variable Definitions
 * These are declared extern in j2534_internal.h
 * ============================================================================ */

// Response callback and queue
void (*j2534_response)(char*, uint32_t, QueueHandle_t *q, char* cmd_str) = NULL;
QueueHandle_t *j2534_rx_queue = NULL;

// Track last TX CAN ID for functional addressing detection
uint32_t j2534_last_tx_id = 0;

// Device state
bool j2534_device_open = false;
uint32_t j2534_device_id = 1;
j2534_channel_t j2534_channels[J2534_MAX_CHANNELS];
uint32_t j2534_active_channel = 0;
j2534_error_t j2534_last_error = J2534_STATUS_NOERROR;
char j2534_last_error_desc[80] = {0};

// Packet parsing state
uint8_t j2534_rx_buffer[J2534_MAX_PACKET_SIZE];
uint32_t j2534_rx_index = 0;
uint8_t j2534_parse_state = 0;
uint16_t j2534_expected_len = 0;

/* ============================================================================
 * PSRAM Buffer Management
 * 
 * WiCAN Pro has 8MB PSRAM at 80MHz which enables much larger buffers for
 * ECU reprogramming operations. Buffers are dynamically allocated in PSRAM
 * if available, with fallback to static allocation in internal RAM.
 * ============================================================================ */

// PSRAM availability flag
bool j2534_psram_available = false;

// PSRAM-allocated RX message buffer (pointer, dynamically allocated)
j2534_msg_t *j2534_rx_msg_buffer = NULL;

// Fallback static buffer for when PSRAM isn't available (smaller size)
#define J2534_RX_MSG_BUFFER_SIZE_FALLBACK 32
static j2534_msg_t j2534_rx_msg_buffer_static[J2534_RX_MSG_BUFFER_SIZE_FALLBACK];

// Actual buffer size (depends on PSRAM availability)
uint32_t j2534_rx_msg_buffer_actual_size = 0;

// RX message buffer indices
volatile uint32_t j2534_rx_msg_head = 0;
volatile uint32_t j2534_rx_msg_tail = 0;

// Flow Control state for multi-frame TX
isotp_fc_state_t isotp_fc_state = {0};

// Periodic message support
j2534_periodic_msg_t j2534_periodic_msgs[J2534_MAX_PERIODIC_MSGS_ACTIVE];
uint32_t j2534_next_periodic_id = 1;
TaskHandle_t j2534_periodic_task_handle = NULL;

// ISO-TP state - data buffer is PSRAM-allocated in init
volatile isotp_rx_state_t isotp_rx_state = {0};
volatile isotp_tx_state_t isotp_tx_state = {0};

// Fallback static ISO-TP buffer (4KB if PSRAM unavailable)
static uint8_t isotp_rx_data_static[4128];

/* ============================================================================
 * PSRAM Allocation Helper
 * ============================================================================ */

/**
 * @brief Allocate PSRAM buffers for J2534
 * 
 * Attempts to allocate large buffers in PSRAM for improved ECU programming
 * performance. Falls back to smaller static buffers if PSRAM unavailable.
 */
static void j2534_alloc_psram_buffers(void)
{
    // Check if PSRAM is available
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    
    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM detected: %zu bytes total", psram_size);
        
        // Allocate RX message buffer in PSRAM (128 messages * ~280 bytes = ~36KB)
        size_t rx_buf_size = J2534_RX_MSG_BUFFER_SIZE * sizeof(j2534_msg_t);
        j2534_rx_msg_buffer = (j2534_msg_t*)heap_caps_malloc(rx_buf_size, MALLOC_CAP_SPIRAM);
        
        if (j2534_rx_msg_buffer) {
            memset(j2534_rx_msg_buffer, 0, rx_buf_size);
            j2534_rx_msg_buffer_actual_size = J2534_RX_MSG_BUFFER_SIZE;
            ESP_LOGI(TAG, "PSRAM: Allocated %zu bytes for RX message buffer (%d msgs)", 
                     rx_buf_size, J2534_RX_MSG_BUFFER_SIZE);
        } else {
            ESP_LOGW(TAG, "PSRAM: Failed to allocate RX buffer, using static fallback");
            j2534_rx_msg_buffer = j2534_rx_msg_buffer_static;
            j2534_rx_msg_buffer_actual_size = J2534_RX_MSG_BUFFER_SIZE_FALLBACK;
        }
        
        // Allocate ISO-TP reassembly buffer in PSRAM (8KB for large responses)
        uint8_t *isotp_data = (uint8_t*)heap_caps_malloc(J2534_ISOTP_RX_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
        
        if (isotp_data) {
            memset(isotp_data, 0, J2534_ISOTP_RX_BUFFER_SIZE);
            isotp_rx_state.data = isotp_data;
            isotp_rx_state.data_buffer_size = J2534_ISOTP_RX_BUFFER_SIZE;
            ESP_LOGI(TAG, "PSRAM: Allocated %d bytes for ISO-TP RX buffer", J2534_ISOTP_RX_BUFFER_SIZE);
        } else {
            ESP_LOGW(TAG, "PSRAM: Failed to allocate ISO-TP buffer, using static fallback");
            isotp_rx_state.data = isotp_rx_data_static;
            isotp_rx_state.data_buffer_size = sizeof(isotp_rx_data_static);
        }
        
        j2534_psram_available = (j2534_rx_msg_buffer != j2534_rx_msg_buffer_static);
        
        // Log remaining PSRAM
        size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "PSRAM: %zu bytes remaining after J2534 allocation", psram_free);
        
    } else {
        ESP_LOGI(TAG, "PSRAM not available, using static buffers");
        j2534_rx_msg_buffer = j2534_rx_msg_buffer_static;
        j2534_rx_msg_buffer_actual_size = J2534_RX_MSG_BUFFER_SIZE_FALLBACK;
        isotp_rx_state.data = isotp_rx_data_static;
        isotp_rx_state.data_buffer_size = sizeof(isotp_rx_data_static);
        j2534_psram_available = false;
    }
    
    ESP_LOGI(TAG, "J2534 buffers: RX msgs=%lu (PSRAM=%s), ISO-TP=%lu bytes",
             j2534_rx_msg_buffer_actual_size,
             j2534_psram_available ? "yes" : "no",
             isotp_rx_state.data_buffer_size);
}

/* ============================================================================
 * ISO-TP State Reset Functions
 * 
 * These functions reset ISO-TP state while PRESERVING the PSRAM buffer pointers.
 * Direct memset() on these structs would zero out the data pointer, causing crashes.
 * ============================================================================ */

/**
 * @brief Reset ISO-TP RX state while preserving PSRAM buffer allocation
 * 
 * This resets the RX state machine to idle WITHOUT destroying the data buffer pointer.
 * Use this instead of memset(&isotp_rx_state, 0, ...) 
 */
void j2534_reset_isotp_rx_state(void)
{
    // Preserve buffer allocation
    uint8_t *saved_data = isotp_rx_state.data;
    uint32_t saved_size = isotp_rx_state.data_buffer_size;
    
    // Clear all fields
    isotp_rx_state.active = false;
    isotp_rx_state.can_id = 0;
    isotp_rx_state.expected_length = 0;
    isotp_rx_state.received_length = 0;
    isotp_rx_state.next_seq_num = 0;
    isotp_rx_state.flow_control_id = 0;
    isotp_rx_state.is_extended = false;
    isotp_rx_state.last_frame_time = 0;
    isotp_rx_state.block_size = 0;
    isotp_rx_state.block_count = 0;
    
    // Restore buffer pointers
    isotp_rx_state.data = saved_data;
    isotp_rx_state.data_buffer_size = saved_size;
}

/**
 * @brief Reset ISO-TP TX state
 */
void j2534_reset_isotp_tx_state(void)
{
    isotp_tx_state.awaiting_response = false;
    isotp_tx_state.tx_complete_time = 0;
    isotp_tx_state.response_can_id = 0;
}

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Check if a protocol ID is a legacy (non-CAN) protocol
 * @return true if J1850 or ISO9141/ISO14230
 */
bool j2534_is_legacy_protocol(uint32_t protocol_id)
{
    switch (protocol_id) {
        case J2534_PROTOCOL_J1850VPW:
        case J2534_PROTOCOL_J1850PWM:
        case J2534_PROTOCOL_ISO9141:
        case J2534_PROTOCOL_ISO14230:
        case J2534_PROTOCOL_J1850VPW_PS:
        case J2534_PROTOCOL_J1850PWM_PS:
        case J2534_PROTOCOL_ISO9141_PS:
        case J2534_PROTOCOL_ISO14230_PS:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Check if a protocol ID is a J1850 protocol (VPW or PWM)
 * @return true if J1850 VPW/PWM
 */
bool j2534_is_j1850_protocol(uint32_t protocol_id)
{
    switch (protocol_id) {
        case J2534_PROTOCOL_J1850VPW:
        case J2534_PROTOCOL_J1850PWM:
        case J2534_PROTOCOL_J1850VPW_PS:
        case J2534_PROTOCOL_J1850PWM_PS:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Check if a protocol ID is a K-line protocol (ISO 9141 or ISO 14230)
 * @return true if ISO 9141 or ISO 14230 (KWP2000)
 */
bool j2534_is_kline_protocol(uint32_t protocol_id)
{
    switch (protocol_id) {
        case J2534_PROTOCOL_ISO9141:
        case J2534_PROTOCOL_ISO14230:
        case J2534_PROTOCOL_ISO9141_PS:
        case J2534_PROTOCOL_ISO14230_PS:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Get the header size for a legacy protocol
 * @param protocol_id J2534 protocol ID
 * @return Header size in bytes (3 for J1850/ISO, can vary for KWP with length byte)
 * 
 * Note: K-line header format depends on the format byte:
 * - ISO 9141-2: Format + Target + Source (3 bytes), length in format byte
 * - ISO 14230: Can have additional length byte if format byte bit 6 = 0
 */
uint8_t j2534_get_legacy_header_size(uint32_t protocol_id, const uint8_t *data, uint32_t data_len)
{
    // J1850 always has 3-byte header: Priority + Target + Source
    if (j2534_is_j1850_protocol(protocol_id)) {
        return 3;
    }
    
    // K-line protocols: analyze format byte to determine header size
    if (j2534_is_kline_protocol(protocol_id) && data != NULL && data_len >= 3) {
        uint8_t format_byte = data[0];
        
        // ISO 14230 format byte:
        // Bit 7: 1 = length in format byte, 0 = separate length byte
        // Bits 5-6: Address mode (10=physical, 11=functional)
        // Bits 0-5 (if bit 7=1): Data length
        
        // If format byte has bit 7 set and bits 0-5 are non-zero, length is in format byte
        // Header = 3 bytes (format + target + source)
        if ((format_byte & 0xC0) != 0) {
            // Length in format byte or functional addressing
            return 3;
        }
        
        // If format byte == 0x80 exactly, there's a separate length byte
        // Header = 4 bytes (format + target + source + length)
        if ((format_byte & 0x3F) == 0 && data_len >= 4) {
            return 4;
        }
        
        // Default to 3-byte header
        return 3;
    }
    
    // Default: 3-byte header
    return 3;
}

uint8_t j2534_calc_checksum(uint8_t *data, uint32_t len)
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void j2534_set_error(j2534_error_t error, const char *desc)
{
    j2534_last_error = error;
    if (desc) {
        strncpy(j2534_last_error_desc, desc, sizeof(j2534_last_error_desc) - 1);
        j2534_last_error_desc[sizeof(j2534_last_error_desc) - 1] = '\0';
    } else {
        j2534_last_error_desc[0] = '\0';
    }
}

j2534_channel_t* j2534_get_channel(uint32_t channel_id)
{
    for (int i = 0; i < J2534_MAX_CHANNELS; i++) {
        if (j2534_channels[i].active && j2534_channels[i].channel_id == channel_id) {
            return &j2534_channels[i];
        }
    }
    return NULL;
}

j2534_channel_t* j2534_alloc_channel(void)
{
    for (int i = 0; i < J2534_MAX_CHANNELS; i++) {
        if (!j2534_channels[i].active) {
            return &j2534_channels[i];
        }
    }
    return NULL;
}

uint32_t j2534_baudrate_to_can(uint32_t baudrate)
{
    switch (baudrate) {
        case 5000:   return CAN_5K;
        case 10000:  return CAN_10K;
        case 20000:  return CAN_20K;
        case 25000:  return CAN_25K;
        case 33333:  return CAN_33K;   // SWCAN / GMLAN normal-speed
        case 50000:  return CAN_50K;
        case 83333:  return CAN_83K;   // SWCAN / GMLAN high-speed
        case 100000: return CAN_100K;
        case 125000: return CAN_125K;
        case 250000: return CAN_250K;
        case 500000: return CAN_500K;
        case 800000: return CAN_800K;
        case 1000000: return CAN_1000K;
        default: return CAN_500K;
    }
}

void j2534_send_response(uint8_t cmd, j2534_error_t status,
                         uint8_t *data, uint16_t data_len, QueueHandle_t *q)
{
    static uint8_t resp_buf[J2534_MAX_PACKET_SIZE];
    uint32_t idx = 0;

    resp_buf[idx++] = J2534_SYNC1;
    resp_buf[idx++] = J2534_SYNC2;
    resp_buf[idx++] = cmd | J2534_RESPONSE_FLAG;
    resp_buf[idx++] = (uint8_t)status;
    resp_buf[idx++] = (data_len >> 8) & 0xFF;
    resp_buf[idx++] = data_len & 0xFF;

    if (data && data_len > 0) {
        memcpy(&resp_buf[idx], data, data_len);
        idx += data_len;
    }

    resp_buf[idx] = j2534_calc_checksum(resp_buf, idx);
    idx++;

    if (j2534_response) {
        j2534_response((char*)resp_buf, idx, q, NULL);
    }
}

/* ============================================================================
 * Periodic Message Task
 * ============================================================================ */

static void j2534_periodic_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Periodic message task started");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < J2534_MAX_PERIODIC_MSGS_ACTIVE; i++) {
            if (!j2534_periodic_msgs[i].active) continue;

            TickType_t elapsed = now - j2534_periodic_msgs[i].last_send_time;
            if (elapsed >= pdMS_TO_TICKS(j2534_periodic_msgs[i].interval_ms)) {
                j2534_channel_t *ch = j2534_get_channel(j2534_periodic_msgs[i].channel_id);
                if (ch && ch->active) {
                    twai_message_t can_frame = {0};

                    // Extract CAN ID from first 4 bytes
                    can_frame.identifier = (j2534_periodic_msgs[i].msg.data[0] << 24) |
                                          (j2534_periodic_msgs[i].msg.data[1] << 16) |
                                          (j2534_periodic_msgs[i].msg.data[2] << 8) |
                                          (j2534_periodic_msgs[i].msg.data[3]);

                    if (j2534_periodic_msgs[i].msg.tx_flags & J2534_CAN_29BIT_ID) {
                        can_frame.extd = 1;
                    } else {
                        can_frame.identifier &= 0x7FF;
                    }

                    // Copy payload (skip first 4 bytes which are CAN ID)
                    uint32_t payload_len = j2534_periodic_msgs[i].msg.data_size - 4;
                    if (payload_len > 8) payload_len = 8;
                    can_frame.data_length_code = payload_len;
                    memcpy(can_frame.data, &j2534_periodic_msgs[i].msg.data[4], payload_len);

                    twai_transmit(&can_frame, pdMS_TO_TICKS(10));
                    j2534_periodic_msgs[i].last_send_time = now;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

j2534_error_t j2534_start_periodic_msg(uint32_t channel_id, j2534_msg_t *msg,
                                       uint32_t interval_ms, uint32_t *msg_id)
{
    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    // Find free slot
    int free_slot = -1;
    for (int i = 0; i < J2534_MAX_PERIODIC_MSGS_ACTIVE; i++) {
        if (!j2534_periodic_msgs[i].active) {
            free_slot = i;
            break;
        }
    }

    if (free_slot < 0) {
        return J2534_ERR_EXCEEDED_LIMIT;
    }

    // Setup periodic message
    j2534_periodic_msgs[free_slot].active = true;
    j2534_periodic_msgs[free_slot].msg_id = j2534_next_periodic_id++;
    j2534_periodic_msgs[free_slot].channel_id = channel_id;
    j2534_periodic_msgs[free_slot].interval_ms = interval_ms;
    memcpy(&j2534_periodic_msgs[free_slot].msg, msg, sizeof(j2534_msg_t));
    j2534_periodic_msgs[free_slot].last_send_time = xTaskGetTickCount();

    *msg_id = j2534_periodic_msgs[free_slot].msg_id;

    // Start task if not running
    if (j2534_periodic_task_handle == NULL) {
        xTaskCreate(j2534_periodic_task, "j2534_periodic", 4096, NULL, 5, &j2534_periodic_task_handle);
    }

    ESP_LOGI(TAG, "Started periodic msg ID=%lu, interval=%lu ms", *msg_id, interval_ms);
    return J2534_STATUS_NOERROR;
}

j2534_error_t j2534_stop_periodic_msg(uint32_t channel_id, uint32_t msg_id)
{
    for (int i = 0; i < J2534_MAX_PERIODIC_MSGS_ACTIVE; i++) {
        if (j2534_periodic_msgs[i].active &&
            j2534_periodic_msgs[i].msg_id == msg_id &&
            j2534_periodic_msgs[i].channel_id == channel_id) {
            j2534_periodic_msgs[i].active = false;
            ESP_LOGI(TAG, "Stopped periodic msg ID=%lu", msg_id);
            return J2534_STATUS_NOERROR;
        }
    }
    return J2534_ERR_INVALID_MSG_ID;
}

/* ============================================================================
 * J2534 API - Device Management
 * ============================================================================ */

void j2534_init(void (*send_callback)(char*, uint32_t, QueueHandle_t *q, char* cmd_str),
                QueueHandle_t *rx_queue)
{
    ESP_LOGI(TAG, "Initializing J2534 module");

    j2534_response = send_callback;
    j2534_rx_queue = rx_queue;

    // Allocate PSRAM buffers if available
    j2534_alloc_psram_buffers();

    // Initialize channels
    memset(j2534_channels, 0, sizeof(j2534_channels));
    for (int i = 0; i < J2534_MAX_CHANNELS; i++) {
        j2534_channels[i].channel_id = i + 1;
    }

    // Initialize periodic messages
    memset(j2534_periodic_msgs, 0, sizeof(j2534_periodic_msgs));
    j2534_next_periodic_id = 1;

    // Initialize FC state
    memset((void*)&isotp_fc_state, 0, sizeof(isotp_fc_state));

    // Reset state
    j2534_device_open = false;
    j2534_active_channel = 0;
    j2534_rx_index = 0;
    j2534_parse_state = 0;
    j2534_rx_msg_head = 0;
    j2534_rx_msg_tail = 0;

    j2534_set_error(J2534_STATUS_NOERROR, NULL);

    ESP_LOGI(TAG, "J2534 module initialized (PSRAM=%s)", j2534_psram_available ? "enabled" : "disabled");
}

j2534_error_t j2534_open(uint32_t *device_id)
{
    if (j2534_device_open) {
        ESP_LOGW(TAG, "Device already open, auto-resetting for new session");
        j2534_reset();
    }

    j2534_device_open = true;
    *device_id = j2534_device_id;

    ESP_LOGI(TAG, "Device opened, ID: %lu (PSRAM buffers: RX=%lu msgs, ISO-TP=%lu bytes)",
             j2534_device_id, j2534_rx_msg_buffer_actual_size, isotp_rx_state.data_buffer_size);
    return J2534_STATUS_NOERROR;
}

j2534_error_t j2534_close(uint32_t device_id)
{
    if (!j2534_device_open) {
        j2534_set_error(J2534_ERR_DEVICE_NOT_CONNECTED, "Device not open");
        return J2534_ERR_DEVICE_NOT_CONNECTED;
    }

    if (device_id != j2534_device_id) {
        j2534_set_error(J2534_ERR_INVALID_DEVICE_ID, "Invalid device ID");
        return J2534_ERR_INVALID_DEVICE_ID;
    }

    // Close all channels
    for (int i = 0; i < J2534_MAX_CHANNELS; i++) {
        if (j2534_channels[i].active) {
            j2534_disconnect(j2534_channels[i].channel_id);
        }
    }

    j2534_device_open = false;
    ESP_LOGI(TAG, "Device closed");
    return J2534_STATUS_NOERROR;
}

bool j2534_is_active(void)
{
    return j2534_device_open && j2534_active_channel > 0;
}

uint32_t j2534_get_protocol(void)
{
    j2534_channel_t *ch = j2534_get_channel(j2534_active_channel);
    if (ch) {
        return ch->protocol_id;
    }
    return 0;
}

void j2534_reset(void)
{
    ESP_LOGI(TAG, "J2534 reset - closing all channels and device");

    // Close all channels
    for (int i = 0; i < J2534_MAX_CHANNELS; i++) {
        if (j2534_channels[i].active) {
            j2534_disconnect(j2534_channels[i].channel_id);
        }
    }

    // Reset device state
    j2534_device_open = false;
    j2534_active_channel = 0;
    j2534_set_error(J2534_STATUS_NOERROR, NULL);

    // Reset parser state
    j2534_parse_state = 0;
    j2534_rx_index = 0;
    j2534_expected_len = 0;

    ESP_LOGI(TAG, "J2534 reset complete");
}

j2534_error_t j2534_get_last_error(char *error_description)
{
    if (error_description) {
        strncpy(error_description, j2534_last_error_desc, 80);
    }
    return j2534_last_error;
}

j2534_error_t j2534_read_version(uint32_t device_id,
                                 char *firmware_version,
                                 char *dll_version,
                                 char *api_version)
{
    if (!j2534_device_open || device_id != j2534_device_id) {
        return J2534_ERR_INVALID_DEVICE_ID;
    }

    if (firmware_version) strncpy(firmware_version, J2534_FW_VERSION, 32);
    if (dll_version) strncpy(dll_version, J2534_DLL_VERSION, 32);
    if (api_version) strncpy(api_version, J2534_API_VERSION, 32);

    return J2534_STATUS_NOERROR;
}
