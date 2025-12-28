/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
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

// RX message buffer
j2534_msg_t j2534_rx_msg_buffer[J2534_RX_MSG_BUFFER_SIZE];
volatile uint32_t j2534_rx_msg_head = 0;
volatile uint32_t j2534_rx_msg_tail = 0;

// Flow Control state for multi-frame TX
isotp_fc_state_t isotp_fc_state = {0};

// Periodic message support
j2534_periodic_msg_t j2534_periodic_msgs[J2534_MAX_PERIODIC_MSGS_ACTIVE];
uint32_t j2534_next_periodic_id = 1;
TaskHandle_t j2534_periodic_task_handle = NULL;

// ISO-TP state
volatile isotp_rx_state_t isotp_rx_state = {0};
volatile isotp_tx_state_t isotp_tx_state = {0};

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
        case 50000:  return CAN_50K;
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

    ESP_LOGI(TAG, "J2534 module initialized");
}

j2534_error_t j2534_open(uint32_t *device_id)
{
    if (j2534_device_open) {
        ESP_LOGW(TAG, "Device already open, auto-resetting for new session");
        j2534_reset();
    }

    j2534_device_open = true;
    *device_id = j2534_device_id;

    ESP_LOGI(TAG, "Device opened, ID: %lu", j2534_device_id);
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
