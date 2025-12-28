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
 * @file stn_j2534.h
 * @brief Bridge layer between J2534 and STN OBD chip
 *
 * This module routes J1850, ISO9141, and ISO14230 J2534 protocol requests
 * through the STN21xx OBD interpreter chip using ST commands for optimal
 * J2534 compliance (raw message mode, precise timing control).
 */

#ifndef __STN_J2534_H__
#define __STN_J2534_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Protocol Types for STN OBD Chip
 * ============================================================================ */

typedef enum {
    STN_PROTO_AUTO          = 0,    // Automatic protocol detection
    STN_PROTO_J1850PWM      = 1,    // SAE J1850 PWM (41.6 kbaud)
    STN_PROTO_J1850VPW      = 2,    // SAE J1850 VPW (10.4 kbaud)
    STN_PROTO_ISO9141       = 3,    // ISO 9141-2 (5 baud init, 10.4 kbaud)
    STN_PROTO_ISO14230_5BAUD = 4,   // ISO 14230-4 KWP (5 baud init)
    STN_PROTO_ISO14230_FAST  = 5,   // ISO 14230-4 KWP (fast init)
    STN_PROTO_CAN_11B_500K  = 6,    // ISO 15765-4 CAN (11 bit ID, 500 kbaud)
    STN_PROTO_CAN_29B_500K  = 7,    // ISO 15765-4 CAN (29 bit ID, 500 kbaud)
    STN_PROTO_CAN_11B_250K  = 8,    // ISO 15765-4 CAN (11 bit ID, 250 kbaud)
    STN_PROTO_CAN_29B_250K  = 9,    // ISO 15765-4 CAN (29 bit ID, 250 kbaud)
    STN_PROTO_CAN_11B_USER1 = 0xA,  // User CAN (11 bit ID, user baud)
    STN_PROTO_CAN_29B_USER1 = 0xB,  // User CAN (29 bit ID, user baud)
    STN_PROTO_GM_UART       = 0xC,  // GM Class 2 UART (8192 baud)
} stn_protocol_t;

/* ============================================================================
 * Status Codes
 * ============================================================================ */

typedef enum {
    STN_J2534_STATUS_OK = 0,
    STN_J2534_STATUS_TIMEOUT,
    STN_J2534_STATUS_NO_DATA,
    STN_J2534_STATUS_BUS_ERROR,
    STN_J2534_STATUS_CHIP_ERROR,
    STN_J2534_STATUS_PROTOCOL_ERROR,
} stn_j2534_status_t;

/* ============================================================================
 * Message Structure for Legacy Protocols
 * ============================================================================ */

typedef struct {
    uint8_t data[256];      // Message data (header + payload)
    uint32_t data_len;      // Data length
    uint32_t timestamp;     // Timestamp in microseconds
    uint8_t rx_status;      // RX status flags
} stn_j2534_msg_t;

/* ============================================================================
 * Configuration
 * ============================================================================ */

typedef struct {
    stn_protocol_t protocol;        // Active protocol
    uint32_t baudrate;              // Baud rate (for user-defined CAN)
    uint8_t tx_header[4];           // TX header bytes
    uint8_t tx_header_len;          // Header length (2-4 bytes)
    uint8_t rx_filter[4];           // RX filter
    uint8_t rx_mask[4];             // RX mask
    uint32_t timeout_ms;            // Response timeout
    bool headers_enabled;           // Show headers in response
    bool echo_enabled;              // Echo TX messages
    bool adaptive_timing;           // Use adaptive timing
} stn_j2534_config_t;

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/**
 * @brief Initialize the STN-J2534 bridge
 * @return ESP_OK on success
 */
esp_err_t stn_j2534_init(void);

/**
 * @brief Select protocol on STN chip
 * @param protocol Protocol to select (STN_PROTO_xxx)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_select_protocol(stn_protocol_t protocol);

/**
 * @brief Send a message via legacy protocol
 * @param data Message data to send (payload only, header set separately)
 * @param data_len Length of data
 * @param responses Buffer for response messages
 * @param max_responses Maximum number of responses to receive
 * @param num_responses Actual number of responses received
 * @param timeout_ms Timeout in milliseconds
 * @return Status code
 */
stn_j2534_status_t stn_j2534_send_message(
    const uint8_t *data, 
    uint32_t data_len,
    stn_j2534_msg_t *responses,
    uint32_t max_responses,
    uint32_t *num_responses,
    uint32_t timeout_ms
);

/**
 * @brief Set TX header for legacy protocols
 * @param header Header bytes
 * @param header_len Length (2-4 bytes depending on protocol)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_set_header(const uint8_t *header, uint8_t header_len);

/**
 * @brief Set RX filter/mask for legacy protocols
 * @param filter Filter bytes
 * @param mask Mask bytes
 * @param len Length (2-4 bytes)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_set_filter(const uint8_t *filter, const uint8_t *mask, uint8_t len);

/**
 * @brief Perform 5-baud initialization (ISO9141/ISO14230)
 * @param target_address Target ECU address (typically 0x33)
 * @param key_bytes Output buffer for key bytes (2 bytes)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_five_baud_init(uint8_t target_address, uint8_t *key_bytes);

/**
 * @brief Perform fast initialization (ISO14230)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_fast_init(void);

/**
 * @brief Set KWP2000/ISO9141 timing parameters
 * @param p1_max P1 max time in ms (inter-byte ECU response)
 * @param p2_max P2 max time in ms (request to response)
 * @param p3_max P3 max time in ms (response to next request)  
 * @param p4_min P4 min time in ms (inter-byte tester request)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_set_timing(uint32_t p1_max, uint32_t p2_max, uint32_t p3_max, uint32_t p4_min);

/**
 * @brief Send TesterPresent message to keep ECU session alive
 * @return Status code
 */
stn_j2534_status_t stn_j2534_tester_present(void);

/**
 * @brief Start periodic TesterPresent (keep-alive)
 * @param interval_ms Interval between messages (typically 2000ms)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_start_keep_alive(uint32_t interval_ms);

/**
 * @brief Stop periodic TesterPresent
 * @return Status code
 */
stn_j2534_status_t stn_j2534_stop_keep_alive(void);

/**
 * @brief Read battery voltage
 * @param voltage_mv Output voltage in millivolts
 * @return Status code
 */
stn_j2534_status_t stn_j2534_read_voltage(uint32_t *voltage_mv);

/**
 * @brief Check if legacy protocol is currently active
 * @return true if a non-CAN protocol is active on the STN chip
 */
bool stn_j2534_is_legacy_active(void);

/**
 * @brief Get current protocol
 * @return Current protocol type
 */
stn_protocol_t stn_j2534_get_protocol(void);

/**
 * @brief Reset STN chip to defaults
 * @return Status code
 */
stn_j2534_status_t stn_j2534_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __STN_J2534_H__ */
