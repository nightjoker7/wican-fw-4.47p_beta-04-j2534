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
 * @file stn_j2534_usdt.h
 * @brief J1850 VPW USDT (Unacknowledged Segmented Data Transfer) for J2534
 *
 * Implements ISO 15765-2 style segmentation for J1850 VPW protocol:
 * - Single Frame (SF): Messages up to 6 data bytes
 * - First Frame (FF): Start of multi-frame message
 * - Consecutive Frame (CF): Continuation frames
 * - Flow Control (FC): Flow control from receiver
 *
 * This enables GM SPS programming over J1850 VPW (GM Class 2) by
 * properly handling large TransferData ($36) blocks.
 *
 * @note J1850 VPW frame format:
 *   [Priority] [Target] [Source] [PCI] [Data...] [CRC]
 *   - 3-byte header is handled by ATSH command
 *   - PCI byte indicates frame type
 *   - Max 7 bytes after header (excluding CRC)
 */

#ifndef STN_J2534_USDT_H
#define STN_J2534_USDT_H

#include <stdint.h>
#include <stdbool.h>
#include "stn_j2534.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ============================================================================ */

// J1850 frame limits (after 3-byte header)
#define J1850_MAX_FRAME_DATA        7       // Max data bytes per frame (including PCI)
#define J1850_SF_MAX_DATA           6       // Single Frame max data (7 - 1 PCI byte)
#define J1850_FF_MAX_DATA           4       // First Frame data (7 - 1 PCI - 2 length bytes)
#define J1850_CF_MAX_DATA           6       // Consecutive Frame data (7 - 1 PCI byte)

// USDT buffer sizes
#define USDT_TX_BUFFER_SIZE         4096    // Max TX message size
#define USDT_RX_BUFFER_SIZE         4096    // Max RX message size

// Timing constants (milliseconds)
#define USDT_CF_TIMEOUT_MS          250     // Timeout waiting for next CF
#define USDT_FC_TIMEOUT_MS          250     // Timeout waiting for FC
#define USDT_STMIN_DEFAULT_MS       10      // Default separation time
#define USDT_BS_DEFAULT             0       // Default block size (0 = no limit)

// PCI byte definitions (upper nibble)
#define PCI_SF                      0x00    // Single Frame
#define PCI_FF                      0x10    // First Frame
#define PCI_CF                      0x20    // Consecutive Frame
#define PCI_FC                      0x30    // Flow Control

// Flow Control status (lower nibble of FC PCI)
#define FC_CTS                      0x00    // Clear To Send
#define FC_WAIT                     0x01    // Wait
#define FC_OVFLW                    0x02    // Overflow (abort)

// Frame type extraction macros
#define PCI_TYPE(pci)               ((pci) & 0xF0)
#define PCI_SF_LEN(pci)             ((pci) & 0x0F)
#define PCI_FF_LEN_HI(pci)          ((pci) & 0x0F)
#define PCI_CF_SEQ(pci)             ((pci) & 0x0F)
#define PCI_FC_STATUS(pci)          ((pci) & 0x0F)

/* ============================================================================
 * Types
 * ============================================================================ */

/**
 * @brief USDT frame type enumeration
 */
typedef enum {
    USDT_FRAME_SF = 0,              // Single Frame
    USDT_FRAME_FF = 1,              // First Frame
    USDT_FRAME_CF = 2,              // Consecutive Frame
    USDT_FRAME_FC = 3,              // Flow Control
    USDT_FRAME_UNKNOWN = 0xFF       // Unknown/invalid
} usdt_frame_type_t;

/**
 * @brief USDT TX state machine
 */
typedef struct {
    bool active;                    // TX session active
    uint8_t *data;                  // Pointer to data being sent
    uint32_t total_len;             // Total message length
    uint32_t sent_len;              // Bytes sent so far
    uint8_t sequence;               // Next CF sequence number (0-F)
    uint8_t header[3];              // J1850 header bytes
    uint8_t block_count;            // Frames sent in current block
    uint8_t block_size;             // BS from FC (0 = unlimited)
    uint8_t stmin;                  // STmin from FC (milliseconds)
    bool waiting_fc;                // Waiting for Flow Control
    uint32_t timestamp;             // Last frame timestamp
} usdt_tx_state_t;

/**
 * @brief USDT RX state machine
 */
typedef struct {
    bool active;                    // RX session active
    uint8_t buffer[USDT_RX_BUFFER_SIZE];  // Reassembly buffer
    uint32_t expected_len;          // Expected total length
    uint32_t received_len;          // Bytes received so far
    uint8_t next_sequence;          // Expected next CF sequence
    uint8_t header[3];              // J1850 header of sender
    uint32_t timestamp;             // Last frame timestamp
    bool complete;                  // Message complete flag
} usdt_rx_state_t;

/**
 * @brief USDT configuration
 */
typedef struct {
    uint8_t bs;                     // Block Size we send in FC (0 = no limit)
    uint8_t stmin;                  // STmin we send in FC (ms)
    uint32_t cf_timeout_ms;         // CF timeout
    uint32_t fc_timeout_ms;         // FC timeout
} usdt_config_t;

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/**
 * @brief Initialize USDT subsystem
 * @return ESP_OK on success
 */
esp_err_t usdt_init(void);

/**
 * @brief Deinitialize USDT subsystem
 */
void usdt_deinit(void);

/**
 * @brief Reset USDT state (both TX and RX)
 */
void usdt_reset(void);

/**
 * @brief Configure USDT parameters
 * @param config Configuration structure
 */
void usdt_configure(const usdt_config_t *config);

/**
 * @brief Detect frame type from PCI byte
 * @param pci PCI byte (first data byte after header)
 * @return Frame type enumeration
 */
usdt_frame_type_t usdt_get_frame_type(uint8_t pci);

/**
 * @brief Check if message needs segmentation
 * @param data_len Total data length (excluding header)
 * @return true if multi-frame needed
 */
bool usdt_needs_segmentation(uint32_t data_len);

/**
 * @brief Send a complete message with automatic segmentation
 * 
 * Handles Single Frame for short messages, or First Frame + Consecutive
 * Frames for longer messages. Handles Flow Control from receiver.
 *
 * @param header 3-byte J1850 header
 * @param data Message data (UDS payload, no header)
 * @param data_len Data length
 * @param timeout_ms Timeout for complete transfer
 * @return STN_J2534_STATUS_OK on success
 */
stn_j2534_status_t usdt_send_message(
    const uint8_t *header,
    const uint8_t *data,
    uint32_t data_len,
    uint32_t timeout_ms);

/**
 * @brief Process a received frame
 * 
 * Handles frame type detection and reassembly:
 * - SF: Complete message, return immediately
 * - FF: Start reassembly, send FC
 * - CF: Continue reassembly
 * - FC: Update TX state
 *
 * @param frame Complete frame including header
 * @param frame_len Frame length
 * @param[out] complete_msg Buffer for complete message (if done)
 * @param[out] complete_len Length of complete message
 * @return true if a complete message is available
 */
bool usdt_process_rx_frame(
    const uint8_t *frame,
    uint32_t frame_len,
    uint8_t *complete_msg,
    uint32_t *complete_len);

/**
 * @brief Check if RX reassembly is in progress
 * @return true if waiting for more CF frames
 */
bool usdt_rx_in_progress(void);

/**
 * @brief Check if TX is in progress
 * @return true if sending multi-frame message
 */
bool usdt_tx_in_progress(void);

/**
 * @brief Get TX state (for debugging)
 * @return Pointer to TX state
 */
const usdt_tx_state_t *usdt_get_tx_state(void);

/**
 * @brief Get RX state (for debugging)
 * @return Pointer to RX state
 */
const usdt_rx_state_t *usdt_get_rx_state(void);

/**
 * @brief Send Flow Control frame
 * 
 * Used when we receive a First Frame and need to tell sender to continue.
 *
 * @param header 3-byte header (swap target/source from FF)
 * @param status FC status (FC_CTS, FC_WAIT, or FC_OVFLW)
 * @param bs Block size
 * @param stmin Separation time minimum (ms)
 * @return STN_J2534_STATUS_OK on success
 */
stn_j2534_status_t usdt_send_flow_control(
    const uint8_t *header,
    uint8_t status,
    uint8_t bs,
    uint8_t stmin);

#ifdef __cplusplus
}
#endif

#endif /* STN_J2534_USDT_H */
