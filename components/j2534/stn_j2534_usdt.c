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
 * @file stn_j2534_usdt.c
 * @brief J1850 VPW USDT Implementation for J2534
 *
 * Implements ISO 15765-2 style segmented data transfer over J1850 VPW.
 * This enables GM SPS/DPS programming by handling large diagnostic messages
 * that exceed single-frame capacity.
 *
 * J1850 VPW Frame Structure (after ATSH header):
 *   Single Frame:     [0L] [D0] [D1] [D2] [D3] [D4] [D5]     L=1-6
 *   First Frame:      [1L] [LL] [D0] [D1] [D2] [D3]          L=length high nibble
 *   Consecutive:      [2N] [D0] [D1] [D2] [D3] [D4] [D5]     N=sequence 0-F
 *   Flow Control:     [3S] [BS] [STmin]                       S=status
 */

#include "stn_j2534_usdt.h"
#include "stn_j2534.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "USDT"

/* ============================================================================
 * External References
 * ============================================================================ */

extern QueueHandle_t uart1_queue;
extern SemaphoreHandle_t xuart1_semaphore;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static usdt_tx_state_t s_tx_state;
static usdt_rx_state_t s_rx_state;
static usdt_config_t s_config = {
    .bs = USDT_BS_DEFAULT,
    .stmin = USDT_STMIN_DEFAULT_MS,
    .cf_timeout_ms = USDT_CF_TIMEOUT_MS,
    .fc_timeout_ms = USDT_FC_TIMEOUT_MS,
};
static bool s_initialized = false;

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Format data as hex string for STN chip
 */
static void format_hex_string(const uint8_t *data, uint32_t len, char *out)
{
    for (uint32_t i = 0; i < len; i++) {
        sprintf(out + (i * 2), "%02X", data[i]);
    }
    out[len * 2] = '\0';
}

/**
 * @brief Send a raw frame via STN chip (data only, header set via ATSH)
 */
static stn_j2534_status_t send_raw_frame(const uint8_t *data, uint32_t len, uint32_t timeout_ms)
{
    char cmd[64];
    char response[256];
    
    // Format data as hex
    format_hex_string(data, len, cmd);
    strcat(cmd, "\r");
    
    ESP_LOGI(TAG, "TX frame: %s", cmd);
    
    return stn_j2534_send_raw_cmd(cmd, response, sizeof(response), timeout_ms);
}

/**
 * @brief Set J1850 header via ATSH command
 */
static stn_j2534_status_t set_header(const uint8_t *header)
{
    char cmd[32];
    char response[64];
    
    snprintf(cmd, sizeof(cmd), "ATSH %02X %02X %02X\r", header[0], header[1], header[2]);
    ESP_LOGI(TAG, "Setting header: %02X %02X %02X", header[0], header[1], header[2]);
    
    return stn_j2534_send_raw_cmd(cmd, response, sizeof(response), 200);
}

/* ============================================================================
 * Public Functions - Initialization
 * ============================================================================ */

esp_err_t usdt_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    
    memset(&s_tx_state, 0, sizeof(s_tx_state));
    memset(&s_rx_state, 0, sizeof(s_rx_state));
    
    s_initialized = true;
    ESP_LOGI(TAG, "USDT initialized");
    
    return ESP_OK;
}

void usdt_deinit(void)
{
    usdt_reset();
    s_initialized = false;
    ESP_LOGI(TAG, "USDT deinitialized");
}

void usdt_reset(void)
{
    memset(&s_tx_state, 0, sizeof(s_tx_state));
    memset(&s_rx_state, 0, sizeof(s_rx_state));
    ESP_LOGI(TAG, "USDT state reset");
}

void usdt_configure(const usdt_config_t *config)
{
    if (config) {
        memcpy(&s_config, config, sizeof(usdt_config_t));
        ESP_LOGI(TAG, "USDT configured: BS=%u STmin=%u", config->bs, config->stmin);
    }
}

/* ============================================================================
 * Public Functions - Frame Detection
 * ============================================================================ */

usdt_frame_type_t usdt_get_frame_type(uint8_t pci)
{
    switch (PCI_TYPE(pci)) {
        case PCI_SF: return USDT_FRAME_SF;
        case PCI_FF: return USDT_FRAME_FF;
        case PCI_CF: return USDT_FRAME_CF;
        case PCI_FC: return USDT_FRAME_FC;
        default:     return USDT_FRAME_UNKNOWN;
    }
}

bool usdt_needs_segmentation(uint32_t data_len)
{
    return data_len > J1850_SF_MAX_DATA;
}

/* ============================================================================
 * Public Functions - TX Operations
 * ============================================================================ */

stn_j2534_status_t usdt_send_message(
    const uint8_t *header,
    const uint8_t *data,
    uint32_t data_len,
    uint32_t timeout_ms)
{
    stn_j2534_status_t status;
    uint8_t frame[J1850_MAX_FRAME_DATA];
    
    if (!s_initialized) {
        ESP_LOGE(TAG, "USDT not initialized");
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    if (data_len == 0) {
        ESP_LOGW(TAG, "Empty message");
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Set the J1850 header
    status = set_header(header);
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to set header");
        return status;
    }
    
    // Single Frame - fits in one frame
    if (data_len <= J1850_SF_MAX_DATA) {
        // Build SF: [0L][D0][D1]...[D5]
        frame[0] = PCI_SF | (data_len & 0x0F);
        memcpy(&frame[1], data, data_len);
        
        ESP_LOGI(TAG, "TX SF: len=%lu PCI=%02X", data_len, frame[0]);
        return send_raw_frame(frame, data_len + 1, timeout_ms);
    }
    
    // Multi-Frame - need FF + CFs
    ESP_LOGI(TAG, "TX Multi-Frame: total_len=%lu", data_len);
    
    // Initialize TX state
    s_tx_state.active = true;
    s_tx_state.total_len = data_len;
    s_tx_state.sent_len = 0;
    s_tx_state.sequence = 1;  // First CF will be sequence 1
    s_tx_state.block_count = 0;
    s_tx_state.block_size = 0;  // Will be set by FC
    s_tx_state.stmin = USDT_STMIN_DEFAULT_MS;
    s_tx_state.waiting_fc = false;
    memcpy(s_tx_state.header, header, 3);
    
    // Send First Frame: [1L][LL][D0][D1][D2][D3]
    frame[0] = PCI_FF | ((data_len >> 8) & 0x0F);
    frame[1] = data_len & 0xFF;
    
    uint32_t ff_data_len = (data_len < J1850_FF_MAX_DATA) ? data_len : J1850_FF_MAX_DATA;
    memcpy(&frame[2], data, ff_data_len);
    
    ESP_LOGI(TAG, "TX FF: total=%lu ff_data=%lu PCI=%02X%02X", 
             data_len, ff_data_len, frame[0], frame[1]);
    
    status = send_raw_frame(frame, ff_data_len + 2, timeout_ms);
    if (status != STN_J2534_STATUS_OK) {
        s_tx_state.active = false;
        return status;
    }
    
    s_tx_state.sent_len = ff_data_len;
    
    // J1850 VPW USDT is "Unacknowledged" - typically no FC required
    // GM Class 2 modules expect CFs to follow immediately after FF
    // However, some modules may send FC - we'll use a short wait and check
    
    s_tx_state.waiting_fc = false;  // Don't wait for FC by default
    s_tx_state.block_size = 0;       // No block limit
    s_tx_state.stmin = 5;            // Small separation time between frames
    
    // Short delay after FF to allow module to prepare (10ms typical)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Send Consecutive Frames
    const uint8_t *remaining_data = data + s_tx_state.sent_len;
    uint32_t remaining_len = data_len - s_tx_state.sent_len;
    
    while (remaining_len > 0) {
        // Wait STmin between frames
        if (s_tx_state.stmin > 0) {
            vTaskDelay(pdMS_TO_TICKS(s_tx_state.stmin));
        }
        
        // Build CF: [2N][D0][D1]...[D5]
        frame[0] = PCI_CF | (s_tx_state.sequence & 0x0F);
        
        uint32_t cf_data_len = (remaining_len < J1850_CF_MAX_DATA) ? remaining_len : J1850_CF_MAX_DATA;
        memcpy(&frame[1], remaining_data, cf_data_len);
        
        ESP_LOGI(TAG, "TX CF[%u]: %lu bytes", s_tx_state.sequence, cf_data_len);
        
        status = send_raw_frame(frame, cf_data_len + 1, timeout_ms);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "CF[%u] failed", s_tx_state.sequence);
            s_tx_state.active = false;
            return status;
        }
        
        // Update state
        s_tx_state.sequence = (s_tx_state.sequence + 1) & 0x0F;
        s_tx_state.sent_len += cf_data_len;
        s_tx_state.block_count++;
        remaining_data += cf_data_len;
        remaining_len -= cf_data_len;
        
        // J1850 USDT is unacknowledged - no FC wait needed between CFs
        // Just use STmin for inter-frame timing
    }
    
    ESP_LOGI(TAG, "TX complete: %lu bytes sent", data_len);
    s_tx_state.active = false;
    
    return STN_J2534_STATUS_OK;
}

stn_j2534_status_t usdt_send_flow_control(
    const uint8_t *header,
    uint8_t status_code,
    uint8_t bs,
    uint8_t stmin)
{
    stn_j2534_status_t status;
    uint8_t frame[3];
    
    // Set header (typically swap target/source from received frame)
    status = set_header(header);
    if (status != STN_J2534_STATUS_OK) {
        return status;
    }
    
    // Build FC frame: [3S][BS][STmin]
    frame[0] = PCI_FC | (status_code & 0x0F);
    frame[1] = bs;
    frame[2] = stmin;
    
    ESP_LOGI(TAG, "TX FC: status=%u BS=%u STmin=%u", status_code, bs, stmin);
    
    return send_raw_frame(frame, 3, 200);
}

/* ============================================================================
 * Public Functions - RX Operations
 * ============================================================================ */

bool usdt_process_rx_frame(
    const uint8_t *frame,
    uint32_t frame_len,
    uint8_t *complete_msg,
    uint32_t *complete_len)
{
    *complete_len = 0;
    
    if (frame_len < 4) {  // Minimum: 3-byte header + 1 PCI
        return false;
    }
    
    // Extract header and PCI
    const uint8_t *header = frame;
    uint8_t pci = frame[3];
    const uint8_t *payload = &frame[4];
    uint32_t payload_len = frame_len - 4;
    
    usdt_frame_type_t type = usdt_get_frame_type(pci);
    
    switch (type) {
        case USDT_FRAME_SF: {
            // Single Frame - complete message
            uint8_t sf_len = PCI_SF_LEN(pci);
            
            if (sf_len > payload_len || sf_len > J1850_SF_MAX_DATA) {
                ESP_LOGW(TAG, "Invalid SF length: %u", sf_len);
                return false;
            }
            
            // Copy header + data to output
            memcpy(complete_msg, header, 3);
            memcpy(complete_msg + 3, payload, sf_len);
            *complete_len = 3 + sf_len;
            
            ESP_LOGI(TAG, "RX SF: len=%u", sf_len);
            return true;
        }
        
        case USDT_FRAME_FF: {
            // First Frame - start reassembly
            if (payload_len < 1) {
                return false;
            }
            
            uint16_t total_len = ((uint16_t)PCI_FF_LEN_HI(pci) << 8) | payload[0];
            
            if (total_len > USDT_RX_BUFFER_SIZE) {
                ESP_LOGE(TAG, "Message too large: %u", total_len);
                // Send FC-OVFLW
                uint8_t fc_header[3] = {header[0], header[2], header[1]};  // Swap target/source
                usdt_send_flow_control(fc_header, FC_OVFLW, 0, 0);
                return false;
            }
            
            // Initialize RX state
            s_rx_state.active = true;
            s_rx_state.expected_len = total_len;
            s_rx_state.received_len = 0;
            s_rx_state.next_sequence = 1;
            s_rx_state.complete = false;
            memcpy(s_rx_state.header, header, 3);
            s_rx_state.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
            
            // Copy FF data (skip length byte)
            uint32_t ff_data_len = payload_len - 1;
            if (ff_data_len > total_len) {
                ff_data_len = total_len;
            }
            
            memcpy(s_rx_state.buffer, &payload[1], ff_data_len);
            s_rx_state.received_len = ff_data_len;
            
            ESP_LOGI(TAG, "RX FF: total=%u got=%lu", total_len, ff_data_len);
            
            // Send Flow Control - Clear To Send
            uint8_t fc_header[3] = {header[0], header[2], header[1]};  // Swap target/source
            usdt_send_flow_control(fc_header, FC_CTS, s_config.bs, s_config.stmin);
            
            // Check if already complete (small message in FF)
            if (s_rx_state.received_len >= s_rx_state.expected_len) {
                memcpy(complete_msg, s_rx_state.header, 3);
                memcpy(complete_msg + 3, s_rx_state.buffer, s_rx_state.received_len);
                *complete_len = 3 + s_rx_state.received_len;
                s_rx_state.active = false;
                return true;
            }
            
            return false;  // Need more CFs
        }
        
        case USDT_FRAME_CF: {
            // Consecutive Frame - continue reassembly
            if (!s_rx_state.active) {
                ESP_LOGW(TAG, "CF received without active RX session");
                return false;
            }
            
            // Check sequence number
            uint8_t seq = PCI_CF_SEQ(pci);
            if (seq != s_rx_state.next_sequence) {
                ESP_LOGE(TAG, "CF sequence error: expected %u got %u", 
                         s_rx_state.next_sequence, seq);
                s_rx_state.active = false;
                return false;
            }
            
            // Calculate how much data to copy
            uint32_t remaining = s_rx_state.expected_len - s_rx_state.received_len;
            uint32_t copy_len = (payload_len < remaining) ? payload_len : remaining;
            
            // Copy data to buffer
            memcpy(&s_rx_state.buffer[s_rx_state.received_len], payload, copy_len);
            s_rx_state.received_len += copy_len;
            s_rx_state.next_sequence = (s_rx_state.next_sequence + 1) & 0x0F;
            s_rx_state.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
            
            ESP_LOGI(TAG, "RX CF[%u]: got=%lu total=%lu/%lu", 
                     seq, copy_len, s_rx_state.received_len, s_rx_state.expected_len);
            
            // Check if complete
            if (s_rx_state.received_len >= s_rx_state.expected_len) {
                memcpy(complete_msg, s_rx_state.header, 3);
                memcpy(complete_msg + 3, s_rx_state.buffer, s_rx_state.received_len);
                *complete_len = 3 + s_rx_state.received_len;
                s_rx_state.active = false;
                s_rx_state.complete = true;
                
                ESP_LOGI(TAG, "RX complete: %lu bytes", *complete_len);
                return true;
            }
            
            return false;  // Need more CFs
        }
        
        case USDT_FRAME_FC: {
            // Flow Control - update TX state
            if (!s_tx_state.active || !s_tx_state.waiting_fc) {
                ESP_LOGW(TAG, "Unexpected FC received");
                return false;
            }
            
            uint8_t fc_status = PCI_FC_STATUS(pci);
            
            switch (fc_status) {
                case FC_CTS:
                    s_tx_state.block_size = (payload_len > 0) ? payload[0] : 0;
                    s_tx_state.stmin = (payload_len > 1) ? payload[1] : USDT_STMIN_DEFAULT_MS;
                    s_tx_state.waiting_fc = false;
                    s_tx_state.block_count = 0;
                    ESP_LOGI(TAG, "FC-CTS: BS=%u STmin=%u", s_tx_state.block_size, s_tx_state.stmin);
                    break;
                    
                case FC_WAIT:
                    ESP_LOGI(TAG, "FC-WAIT received");
                    // Keep waiting_fc = true, reset timeout
                    s_tx_state.timestamp = (uint32_t)(esp_timer_get_time() / 1000);
                    break;
                    
                case FC_OVFLW:
                    ESP_LOGE(TAG, "FC-OVFLW received - aborting");
                    s_tx_state.active = false;
                    s_tx_state.waiting_fc = false;
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown FC status: %u", fc_status);
                    break;
            }
            
            return false;  // FC doesn't produce a complete message
        }
        
        default:
            ESP_LOGW(TAG, "Unknown frame type: PCI=%02X", pci);
            return false;
    }
}

/* ============================================================================
 * Public Functions - State Queries
 * ============================================================================ */

bool usdt_rx_in_progress(void)
{
    return s_rx_state.active;
}

bool usdt_tx_in_progress(void)
{
    return s_tx_state.active;
}

const usdt_tx_state_t *usdt_get_tx_state(void)
{
    return &s_tx_state;
}

const usdt_rx_state_t *usdt_get_rx_state(void)
{
    return &s_rx_state;
}
