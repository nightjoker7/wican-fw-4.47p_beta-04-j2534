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
 * @file j2534_internal.h
 * @brief Internal definitions and state structures for J2534 component
 * 
 * This header contains internal types, state structures, and shared variables
 * used across the J2534 component files. Not intended for external use.
 */

#ifndef __J2534_INTERNAL_H__
#define __J2534_INTERNAL_H__

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/twai.h"
#include "j2534.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Internal Constants
 * ============================================================================ */

#define J2534_TAG "J2534"

// RX message buffer for received CAN frames
// 32 messages × ~280 bytes = ~9KB - reasonable for ESP32-S3
#define J2534_RX_MSG_BUFFER_SIZE 32

// Maximum periodic messages active at once
#define J2534_MAX_PERIODIC_MSGS_ACTIVE 8

/* ============================================================================
 * Flow Control State (for multi-frame TX synchronization with RX task)
 * ============================================================================ */

typedef struct {
    volatile bool waiting_for_fc;        // True when we've sent FF and waiting for FC
    volatile bool fc_received;           // True when FC has been received
    volatile uint8_t fc_flow_status;     // FC Flow Status: 0=CTS, 1=Wait, 2=Overflow
    volatile uint8_t fc_block_size;      // FC Block Size (0 = no limit)
    volatile uint8_t fc_stmin;           // FC STmin (separation time)
    volatile uint32_t fc_rx_id;          // CAN ID we're expecting FC from
} isotp_fc_state_t;

/* ============================================================================
 * ISO-TP Multi-frame RX State
 * NOTE: Accessed from multiple tasks (CAN RX task and J2534 command task)
 * ============================================================================ */

typedef struct {
    volatile bool active;               // Multi-frame reception in progress
    uint32_t can_id;                    // CAN ID of the message
    volatile uint32_t expected_length;  // Total expected data length
    volatile uint32_t received_length;  // Data received so far
    uint8_t next_seq_num;               // Next expected sequence number
    uint8_t data[4128];                 // ISO-TP max data size (4095 + padding)
    uint32_t flow_control_id;           // CAN ID for Flow Control response
    bool is_extended;                   // Extended CAN ID flag
    TickType_t last_frame_time;         // Timestamp of last frame
    uint8_t block_size;                 // BS from our FC (0 = no limit)
    uint8_t block_count;                // Frames received in current block
} isotp_rx_state_t;

/* ============================================================================
 * ISO-TP TX State (for extended wait on programming operations)
 * ============================================================================ */

typedef struct {
    volatile bool awaiting_response;     // True after sending ISO-TP multi-frame
    volatile TickType_t tx_complete_time; // When the TX completed
    volatile uint32_t response_can_id;   // Expected response CAN ID
} isotp_tx_state_t;

/* ============================================================================
 * Periodic Message State
 * ============================================================================ */

typedef struct {
    bool active;
    uint32_t msg_id;
    uint32_t channel_id;
    uint32_t interval_ms;
    j2534_msg_t msg;
    TickType_t last_send_time;
} j2534_periodic_msg_t;

/* ============================================================================
 * Extern Declarations for Shared State Variables
 * These are defined in j2534_core.c
 * ============================================================================ */

// Response callback and queue
extern void (*j2534_response)(char*, uint32_t, QueueHandle_t *q, char* cmd_str);
extern QueueHandle_t *j2534_rx_queue;

// Device state
extern bool j2534_device_open;
extern uint32_t j2534_device_id;
extern j2534_channel_t j2534_channels[J2534_MAX_CHANNELS];
extern uint32_t j2534_active_channel;
extern j2534_error_t j2534_last_error;
extern char j2534_last_error_desc[80];

// Packet parsing state
extern uint8_t j2534_rx_buffer[J2534_MAX_PACKET_SIZE];
extern uint32_t j2534_rx_index;
extern uint8_t j2534_parse_state;
extern uint16_t j2534_expected_len;

// RX message buffer
extern j2534_msg_t j2534_rx_msg_buffer[J2534_RX_MSG_BUFFER_SIZE];
extern volatile uint32_t j2534_rx_msg_head;
extern volatile uint32_t j2534_rx_msg_tail;

// Last TX CAN ID for functional addressing detection
extern uint32_t j2534_last_tx_id;

// Flow control state
extern isotp_fc_state_t isotp_fc_state;

// Periodic message state
extern j2534_periodic_msg_t j2534_periodic_msgs[J2534_MAX_PERIODIC_MSGS_ACTIVE];
extern uint32_t j2534_next_periodic_id;
extern TaskHandle_t j2534_periodic_task_handle;

// ISO-TP state
extern volatile isotp_rx_state_t isotp_rx_state;
extern volatile isotp_tx_state_t isotp_tx_state;

/* ============================================================================
 * Internal Helper Function Declarations
 * ============================================================================ */

// Core helpers (j2534_core.c)
uint8_t j2534_calc_checksum(uint8_t *data, uint32_t len);
void j2534_set_error(j2534_error_t error, const char *desc);
j2534_channel_t* j2534_get_channel(uint32_t channel_id);
j2534_channel_t* j2534_alloc_channel(void);
uint32_t j2534_baudrate_to_can(uint32_t baudrate);
void j2534_send_response(uint8_t cmd, j2534_error_t status, uint8_t *data, uint16_t data_len, QueueHandle_t *q);
bool j2534_is_legacy_protocol(uint32_t protocol_id);

// TX indication (j2534_msg.c)
void j2534_buffer_tx_indication(j2534_msg_t *orig_msg, uint32_t protocol_id);

// Periodic messages (j2534_core.c)
j2534_error_t j2534_start_periodic_msg(uint32_t channel_id, j2534_msg_t *msg, uint32_t interval_ms, uint32_t *msg_id);
j2534_error_t j2534_stop_periodic_msg(uint32_t channel_id, uint32_t msg_id);

// Filter functions (j2534_filter.c)
bool j2534_filter_message(j2534_channel_t *ch, twai_message_t *frame, uint32_t *flow_control_id);
void j2534_clear_all_filters(j2534_channel_t *ch);

// ISO-TP functions (j2534_isotp.c)
uint32_t isotp_get_flow_control_id(j2534_channel_t *ch, uint32_t rx_id);
void isotp_send_flow_control(j2534_channel_t *ch, uint32_t tx_id, bool is_extended);
void isotp_buffer_complete_message(j2534_channel_t *ch, uint32_t can_id, uint8_t *data, uint32_t length, bool is_extended);
void j2534_process_can_frame(twai_message_t *frame);
j2534_error_t j2534_send_isotp_message(j2534_channel_t *ch, uint32_t can_id, uint8_t *data, uint32_t data_len, bool is_extended, bool padding);

// Command handler (j2534_cmd.c)
void j2534_handle_command(uint8_t cmd, uint8_t *data, uint16_t len, QueueHandle_t *q);

#ifdef __cplusplus
}
#endif

#endif /* __J2534_INTERNAL_H__ */
