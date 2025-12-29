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
 * @file stn_j2534.c
 * @brief Bridge layer between J2534 and STN OBD chip
 *
 * Routes J1850, ISO9141, and ISO14230 J2534 protocol requests through
 * the STN21xx OBD interpreter chip. Uses ST commands where possible
 * for better J2534 compliance (raw message mode, precise timing).
 * 
 * STN-specific commands used:
 * - STPX: Protocol transmit with options (raw mode, timeout, responses)
 * - STPTO: Set protocol timeout in milliseconds
 * - STPRS: Protocol receive start (async receive)
 * - STDI: Display device ID
 * - Standard AT commands for basic configuration
 */

#include "stn_j2534.h"
#include "elm327.h"
#include "hw_config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#if HARDWARE_VER == WICAN_PRO

#define TAG "STN_J2534"

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define STN_J2534_UART_NUM          UART_NUM_1
#define STN_J2534_BUFFER_SIZE       512
#define STN_J2534_CMD_TIMEOUT_MS    1000
#define STN_J2534_MUTEX_TIMEOUT_MS  5000
#define STN_J2534_READ_TIMEOUT_MS   10

/* ============================================================================
 * External References (from elm327.c)
 * ============================================================================ */

extern QueueHandle_t uart1_queue;
extern SemaphoreHandle_t xuart1_semaphore;

/* ============================================================================
 * Static Variables
 * ============================================================================ */

static bool s_initialized = false;
static stn_j2534_config_t s_config = {
    .protocol = STN_PROTO_AUTO,
    .baudrate = 0,
    .tx_header = {0x68, 0x6A, 0xF1},  // Default J1850/ISO header
    .tx_header_len = 3,
    .timeout_ms = 1000,
    .headers_enabled = true,
    .echo_enabled = false,
    .adaptive_timing = true,
};

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Read from UART until pattern found or timeout
 */
static int stn_j2534_read_until(char* buffer, size_t buffer_size, 
                                const char* pattern, int timeout_ms)
{
    int total_len = 0;
    int64_t start_time = esp_timer_get_time() / 1000;
    
    while (total_len < buffer_size - 1) {
        int len = uart_read_bytes(STN_J2534_UART_NUM, buffer + total_len, 
                                  buffer_size - total_len - 1, 
                                  pdMS_TO_TICKS(STN_J2534_READ_TIMEOUT_MS));
        
        if (len > 0) {
            total_len += len;
            buffer[total_len] = '\0';
            
            if (strstr(buffer, pattern)) {
                return total_len;
            }
        }
        
        if ((esp_timer_get_time() / 1000 - start_time) >= timeout_ms) {
            ESP_LOGW(TAG, "Read timeout after %d ms", timeout_ms);
            break;
        }
    }
    
    return total_len;
}

/**
 * @brief Send command and wait for response
 * @note Works with both AT and ST commands
 */
static stn_j2534_status_t stn_j2534_send_cmd(const char* cmd, char* response, 
                                              size_t response_size, int timeout_ms)
{
    if (!xuart1_semaphore) {
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    if (xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take UART semaphore");
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Flush any stale data
    uart_flush_input(STN_J2534_UART_NUM);
    
    // Send command
    ESP_LOGD(TAG, "TX: %s", cmd);
    uart_write_bytes(STN_J2534_UART_NUM, cmd, strlen(cmd));
    
    // Read response
    int len = stn_j2534_read_until(response, response_size, "\r>", timeout_ms);
    
    xSemaphoreGive(xuart1_semaphore);
    
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGD(TAG, "RX: %s", response);
        
        // Check for error responses
        if (strstr(response, "?")) {
            return STN_J2534_STATUS_CHIP_ERROR;
        }
        if (strstr(response, "NO DATA")) {
            return STN_J2534_STATUS_NO_DATA;
        }
        // Note: "BUS INIT: OK" is success, "BUS INIT: ...ERROR" is failure
        if (strstr(response, "BUS INIT") && strstr(response, "ERROR")) {
            return STN_J2534_STATUS_BUS_ERROR;
        }
        if (strstr(response, "BUS ERROR")) {
            return STN_J2534_STATUS_BUS_ERROR;
        }
        if (strstr(response, "UNABLE TO CONNECT")) {
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        return STN_J2534_STATUS_OK;
    }
    
    return STN_J2534_STATUS_TIMEOUT;
}

/**
 * @brief Parse hex string to bytes
 */
static int stn_j2534_parse_hex_response(const char* response, uint8_t* data, size_t max_len)
{
    int data_len = 0;
    const char* p = response;
    
    // Skip whitespace and echo
    while (*p && (*p == ' ' || *p == '\r' || *p == '\n' || *p == '>')) {
        p++;
    }
    
    // Parse hex bytes
    while (*p && data_len < (int)max_len) {
        // Skip non-hex characters
        while (*p && !isxdigit((unsigned char)*p)) {
            if (*p == '\r' || *p == '>') {
                // End of line
                p++;
                continue;
            }
            p++;
        }
        
        if (!*p) break;
        
        // Parse hex byte
        char hex[3] = {0};
        if (isxdigit((unsigned char)p[0]) && isxdigit((unsigned char)p[1])) {
            hex[0] = p[0];
            hex[1] = p[1];
            data[data_len++] = (uint8_t)strtol(hex, NULL, 16);
            p += 2;
        } else {
            p++;
        }
    }
    
    return data_len;
}

/**
 * @brief Format bytes as hex string
 */
static void stn_j2534_format_hex(const uint8_t* data, size_t len, char* out)
{
    for (size_t i = 0; i < len; i++) {
        sprintf(out + (i * 2), "%02X", data[i]);
    }
    out[len * 2] = '\0';
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t stn_j2534_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing STN chip for J2534...");
    
    // First, ensure the chip is awake by setting SLEEP pin high
    gpio_set_level(OBD_SLEEP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Check if chip is ready
    if (elm327_chip_get_status() != ELM327_READY) {
        ESP_LOGI(TAG, "STN chip not ready, performing hardware reset...");
        
        // Hardware reset: pull RESET low then high
        gpio_set_level(OBD_RESET_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(OBD_RESET_PIN, 1);
        
        // Wait for chip to come up (up to 3 seconds)
        int retry_count = 0;
        const int max_retries = 30;  // 30 x 100ms = 3 seconds
        
        while (elm327_chip_get_status() != ELM327_READY && retry_count < max_retries) {
            vTaskDelay(pdMS_TO_TICKS(100));
            retry_count++;
            if (retry_count % 10 == 0) {
                ESP_LOGI(TAG, "Waiting for STN chip... (%d/%d)", retry_count, max_retries);
            }
        }
        
        if (elm327_chip_get_status() != ELM327_READY) {
            ESP_LOGE(TAG, "STN chip not ready after reset, GPIO7=%d", gpio_get_level(OBD_READY_PIN));
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    ESP_LOGI(TAG, "STN chip ready, GPIO7=%d", gpio_get_level(OBD_READY_PIN));
    
    // Small delay to let chip stabilize after wake
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Reset to defaults
    stn_j2534_status_t status = stn_j2534_reset();
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to reset STN chip");
        return ESP_FAIL;
    }
    
    s_initialized = true;
    ESP_LOGI(TAG, "STN-J2534 bridge initialized successfully");
    
    return ESP_OK;
}

stn_j2534_status_t stn_j2534_reset(void)
{
    char response[128];
    stn_j2534_status_t status;
    
    // Send ATZ (reset)
    status = stn_j2534_send_cmd("ATZ\r", response, sizeof(response), 2000);
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGW(TAG, "ATZ failed, trying ATD");
    }
    
    // Send ATD (defaults)
    status = stn_j2534_send_cmd("ATD\r", response, sizeof(response), 500);
    
    // Disable echo
    stn_j2534_send_cmd("ATE0\r", response, sizeof(response), 500);
    
    // Enable headers (required for J2534 - need full message)
    stn_j2534_send_cmd("ATH1\r", response, sizeof(response), 500);
    
    // Disable spaces (cleaner parsing)
    stn_j2534_send_cmd("ATS0\r", response, sizeof(response), 500);
    
    // Enable adaptive timing
    stn_j2534_send_cmd("ATAT1\r", response, sizeof(response), 500);
    
    // STN-specific: Check if STDI works (confirms STN chip)
    status = stn_j2534_send_cmd("STDI\r", response, sizeof(response), 500);
    if (status == STN_J2534_STATUS_OK) {
        ESP_LOGI(TAG, "STN chip detected: %s", response);
    } else {
        ESP_LOGW(TAG, "STDI not supported - may be ELM327 clone");
    }
    
    s_config.protocol = STN_PROTO_AUTO;
    
    return STN_J2534_STATUS_OK;
}

stn_j2534_status_t stn_j2534_select_protocol(stn_protocol_t protocol)
{
    char cmd[32];
    char response[128];
    stn_j2534_status_t status;
    
    ESP_LOGI(TAG, "Selecting protocol %d", protocol);
    
    // Special handling for GM Class 2 UART (8192 baud)
    if (protocol == STN_PROTO_GM_UART) {
        ESP_LOGI(TAG, "Initializing GM Class 2 UART protocol");
        
        // Reset to defaults first
        status = stn_j2534_send_cmd("ATD\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK) return status;
        
        // Set J1850 VPW as base protocol (closest to Class 2 timing)
        status = stn_j2534_send_cmd("ATSP2\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK) return status;
        
        // Configure for 8192 baud (GM Class 2)
        // ATIB 96 sets ISO baud rate to 9600 (closest standard rate)
        // Some STN chips support ATIB 82 for 8192 baud directly
        status = stn_j2534_send_cmd("ATIB 96\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        // Ignore error - not all chips support this
        
        // Set slower timing for Class 2
        status = stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        
        // Set longer timeout for Class 2 responses
        status = stn_j2534_send_cmd("ATST FF\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        
        // Enable headers in response
        status = stn_j2534_send_cmd("ATH1\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "GM Class 2 UART initialized");
        return STN_J2534_STATUS_OK;
    }
    
    // J1850 VPW specific setup (Class 2 Serial)
    if (protocol == STN_PROTO_J1850VPW) {
        ESP_LOGI(TAG, "Initializing J1850 VPW (Class 2 Serial) protocol");
        
        // Reset to defaults first
        stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Select protocol 2 (J1850 VPW 10.4 kbaud)
        status = stn_j2534_send_cmd("ATSP2\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "ATSP2 failed: %s", response);
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        // Enable headers - J2534 needs full message with header bytes
        stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Disable spaces for cleaner parsing
        stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Disable echo
        stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Allow long messages (up to 256 bytes)
        stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Set default timeout (ATST in 4ms units, 0xFF = 1020ms)
        stn_j2534_send_cmd("ATST FF\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Try STN-specific command for millisecond timeout
        stn_j2534_send_cmd("STPTO 1000\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Disable adaptive timing - use fixed timing for J2534 compliance
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Set default J1850 header (functional broadcast to BCM)
        stn_j2534_send_cmd("ATSH686AF1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Try to wake up the bus by sending a test message (TesterPresent)
        // This helps ensure the bus is actually active
        ESP_LOGI(TAG, "Attempting bus initialization...");
        status = stn_j2534_send_cmd("3F00\r", response, sizeof(response), 5000);
        if (status == STN_J2534_STATUS_NO_DATA) {
            // No response is OK for TesterPresent - bus may be active
            ESP_LOGI(TAG, "No response to bus init (expected for some ECUs)");
        } else if (status != STN_J2534_STATUS_OK) {
            ESP_LOGW(TAG, "Bus init response: %s", response);
        } else {
            ESP_LOGI(TAG, "Bus init got response: %s", response);
        }
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "J1850 VPW protocol initialized successfully");
        return STN_J2534_STATUS_OK;
    }
    
    // J1850 PWM specific setup
    if (protocol == STN_PROTO_J1850PWM) {
        ESP_LOGI(TAG, "Initializing J1850 PWM protocol");
        
        stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        status = stn_j2534_send_cmd("ATSP1\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK || !strstr(response, "OK")) {
            ESP_LOGE(TAG, "ATSP1 failed: %s", response);
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATST FF\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "J1850 PWM protocol initialized successfully");
        return STN_J2534_STATUS_OK;
    }
    
    // Build ATSP command for standard protocols
    if (protocol <= 9) {
        snprintf(cmd, sizeof(cmd), "ATSP%d\r", protocol);
    } else if (protocol <= 0xB) {
        snprintf(cmd, sizeof(cmd), "ATSP%c\r", 'A' + (protocol - 10));
    } else {
        return STN_J2534_STATUS_PROTOCOL_ERROR;
    }
    
    status = stn_j2534_send_cmd(cmd, response, sizeof(response), 
                                 STN_J2534_CMD_TIMEOUT_MS);
    
    if (status == STN_J2534_STATUS_OK && strstr(response, "OK")) {
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "Selected protocol %d", protocol);
        return STN_J2534_STATUS_OK;
    }
    
    ESP_LOGE(TAG, "Protocol selection failed: %s", response);
    return status;
}

stn_j2534_status_t stn_j2534_set_header(const uint8_t *header, uint8_t header_len)
{
    char cmd[32];
    char response[128];
    
    if (header_len < 2 || header_len > 4) {
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Copy header
    memcpy(s_config.tx_header, header, header_len);
    s_config.tx_header_len = header_len;
    
    // Format AT SH command based on length
    if (header_len == 3) {
        snprintf(cmd, sizeof(cmd), "ATSH%02X%02X%02X\r", 
                 header[0], header[1], header[2]);
    } else if (header_len == 4) {
        // Extended addressing
        snprintf(cmd, sizeof(cmd), "ATSH%02X%02X%02X%02X\r", 
                 header[0], header[1], header[2], header[3]);
    } else {
        // 2-byte header for some protocols
        snprintf(cmd, sizeof(cmd), "ATSH%02X%02X\r", header[0], header[1]);
    }
    
    return stn_j2534_send_cmd(cmd, response, sizeof(response), 
                               STN_J2534_CMD_TIMEOUT_MS);
}

stn_j2534_status_t stn_j2534_set_filter(const uint8_t *filter, const uint8_t *mask, uint8_t len)
{
    char cmd[32];
    char response[128];
    stn_j2534_status_t status;
    
    if (len < 1 || len > 4) {
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Save filter/mask
    memcpy(s_config.rx_filter, filter, len);
    memcpy(s_config.rx_mask, mask, len);
    
    // Set receive address (CRA for filtering)
    // For J1850 VPW, only 1 byte header is used
    if (len == 1) {
        snprintf(cmd, sizeof(cmd), "ATCRA%02X\r", filter[0]);
    } else if (len == 2) {
        snprintf(cmd, sizeof(cmd), "ATCRA%02X%02X\r", filter[0], filter[1]);
    } else if (len == 3) {
        snprintf(cmd, sizeof(cmd), "ATCRA%02X%02X%02X\r", 
                 filter[0], filter[1], filter[2]);
    } else {
        snprintf(cmd, sizeof(cmd), "ATCRA%02X%02X%02X%02X\r", 
                 filter[0], filter[1], filter[2], filter[3]);
    }
    
    status = stn_j2534_send_cmd(cmd, response, sizeof(response), 
                                 STN_J2534_CMD_TIMEOUT_MS);
    
    return status;
}

stn_j2534_status_t stn_j2534_five_baud_init(uint8_t target_address, uint8_t *key_bytes)
{
    char cmd[32];
    char response[256];
    stn_j2534_status_t status;
    
    // ISO9141/ISO14230 5-baud init
    // Set target address
    snprintf(cmd, sizeof(cmd), "ATIIA%02X\r", target_address);
    status = stn_j2534_send_cmd(cmd, response, sizeof(response), 
                                 STN_J2534_CMD_TIMEOUT_MS);
    if (status != STN_J2534_STATUS_OK) {
        return status;
    }
    
    // Perform slow init
    status = stn_j2534_send_cmd("ATSI\r", response, sizeof(response), 10000);
    
    if (status == STN_J2534_STATUS_OK) {
        // Parse key bytes from response (format: "BUS INIT: OK" or "BUS INIT: ...XX XX")
        if (key_bytes) {
            key_bytes[0] = 0;
            key_bytes[1] = 0;
            // Try to parse key bytes if present
            char* kb_start = strstr(response, "BUS INIT:");
            if (kb_start) {
                kb_start += 9;  // Skip "BUS INIT:"
                while (*kb_start == ' ' || *kb_start == '.') kb_start++;
                // Parse two hex bytes if present
                if (isxdigit((unsigned char)kb_start[0]) && isxdigit((unsigned char)kb_start[1])) {
                    char hex[3] = {kb_start[0], kb_start[1], 0};
                    key_bytes[0] = (uint8_t)strtol(hex, NULL, 16);
                    kb_start += 2;
                    while (*kb_start == ' ') kb_start++;
                    if (isxdigit((unsigned char)kb_start[0]) && isxdigit((unsigned char)kb_start[1])) {
                        hex[0] = kb_start[0];
                        hex[1] = kb_start[1];
                        key_bytes[1] = (uint8_t)strtol(hex, NULL, 16);
                    }
                }
            }
        }
        return STN_J2534_STATUS_OK;
    }
    
    return STN_J2534_STATUS_PROTOCOL_ERROR;
}

stn_j2534_status_t stn_j2534_fast_init(void)
{
    char response[256];
    
    // ISO 14230-4 fast init
    return stn_j2534_send_cmd("ATFI\r", response, sizeof(response), 5000);
}

stn_j2534_status_t stn_j2534_set_timing(uint32_t p1_max, uint32_t p2_max, 
                                         uint32_t p3_max, uint32_t p4_min)
{
    char cmd[32];
    char response[64];
    stn_j2534_status_t status;
    
    // ATTP (Protocol Timing) - sets P2 max timeout
    // Value in hex, units of 4ms for ELM, or use STPTO for STN (ms precision)
    if (p2_max > 0) {
        // Try STN STPTO first (millisecond precision)
        snprintf(cmd, sizeof(cmd), "STPTO%lu\r", p2_max);
        status = stn_j2534_send_cmd(cmd, response, sizeof(response), 500);
        if (status != STN_J2534_STATUS_OK) {
            // Fall back to ATST (4ms units, max 0xFF = 1020ms)
            uint32_t st_value = p2_max / 4;
            if (st_value > 0xFF) st_value = 0xFF;
            snprintf(cmd, sizeof(cmd), "ATST%02lX\r", st_value);
            stn_j2534_send_cmd(cmd, response, sizeof(response), 500);
        }
    }
    
    // P4 min - inter-byte time for tester (ATAT controls adaptive timing)
    // P4_MIN < 5ms suggests tight timing, disable adaptive
    if (p4_min > 0 && p4_min < 5) {
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), 500);
    } else {
        stn_j2534_send_cmd("ATAT1\r", response, sizeof(response), 500);
    }
    
    // P3 affects inter-message timing - use ATIB for interbyte timing
    // Note: Most timing is auto-handled by OBD chip, this is best-effort
    
    ESP_LOGI(TAG, "Timing set: P1=%lu P2=%lu P3=%lu P4=%lu", 
             p1_max, p2_max, p3_max, p4_min);
    
    return STN_J2534_STATUS_OK;
}

stn_j2534_status_t stn_j2534_tester_present(void)
{
    char response[128];
    
    // Send TesterPresent (0x3E) with suppressPositiveResponse subfunction (0x80)
    // This keeps the ECU diagnostic session alive without cluttering responses
    return stn_j2534_send_cmd("3E80\r", response, sizeof(response), 500);
}

// Keep-alive state
static bool s_keep_alive_active = false;
static TimerHandle_t s_keep_alive_timer = NULL;

static void keep_alive_timer_callback(TimerHandle_t xTimer)
{
    if (s_keep_alive_active && stn_j2534_is_legacy_active()) {
        stn_j2534_tester_present();
    }
}

stn_j2534_status_t stn_j2534_start_keep_alive(uint32_t interval_ms)
{
    if (s_keep_alive_timer == NULL) {
        s_keep_alive_timer = xTimerCreate("kwp_keepalive", 
                                           pdMS_TO_TICKS(interval_ms),
                                           pdTRUE,  // Auto-reload
                                           NULL,
                                           keep_alive_timer_callback);
    } else {
        xTimerChangePeriod(s_keep_alive_timer, pdMS_TO_TICKS(interval_ms), 100);
    }
    
    if (s_keep_alive_timer) {
        s_keep_alive_active = true;
        xTimerStart(s_keep_alive_timer, 100);
        ESP_LOGI(TAG, "Keep-alive started: %lu ms interval", interval_ms);
        return STN_J2534_STATUS_OK;
    }
    
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_stop_keep_alive(void)
{
    s_keep_alive_active = false;
    if (s_keep_alive_timer) {
        xTimerStop(s_keep_alive_timer, 100);
        ESP_LOGI(TAG, "Keep-alive stopped");
    }
    return STN_J2534_STATUS_OK;
}

stn_j2534_status_t stn_j2534_send_message(
    const uint8_t *data, 
    uint32_t data_len,
    stn_j2534_msg_t *responses,
    uint32_t max_responses,
    uint32_t *num_responses,
    uint32_t timeout_ms)
{
    char cmd[STN_J2534_BUFFER_SIZE];
    char response[STN_J2534_BUFFER_SIZE];
    stn_j2534_status_t status;
    
    *num_responses = 0;
    
    if (data_len > 7) {
        // For messages > 7 bytes, would need multi-frame handling
        // Not implemented yet - legacy protocols typically use shorter messages
        ESP_LOGW(TAG, "Message too long for legacy protocol: %lu bytes", data_len);
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Set timeout using STPTO if available (millisecond precision)
    // Fall back to ATST if STPTO fails
    char timeout_cmd[32];
    snprintf(timeout_cmd, sizeof(timeout_cmd), "STPTO%lu\r", timeout_ms);
    status = stn_j2534_send_cmd(timeout_cmd, response, sizeof(response), 500);
    if (status != STN_J2534_STATUS_OK) {
        // Fall back to ATST (4ms units)
        uint32_t timeout_hex = (timeout_ms / 4);
        if (timeout_hex > 0xFF) timeout_hex = 0xFF;
        snprintf(timeout_cmd, sizeof(timeout_cmd), "ATST%02lX\r", timeout_hex);
        stn_j2534_send_cmd(timeout_cmd, response, sizeof(response), 500);
    }
    
    // Build hex data command
    stn_j2534_format_hex(data, data_len, cmd);
    strcat(cmd, "\r");
    
    // Send and wait for responses
    status = stn_j2534_send_cmd(cmd, response, sizeof(response), timeout_ms + 500);
    
    if (status == STN_J2534_STATUS_OK || status == STN_J2534_STATUS_NO_DATA) {
        // Parse responses (multiple lines possible)
        char* line_start = response;
        char* line_end;
        uint32_t resp_count = 0;
        
        while ((line_end = strchr(line_start, '\r')) != NULL && resp_count < max_responses) {
            *line_end = '\0';
            
            // Skip empty lines and prompt
            if (strlen(line_start) >= 2 && line_start[0] != '>') {
                // Parse this response line
                int parsed_len = stn_j2534_parse_hex_response(
                    line_start, 
                    responses[resp_count].data, 
                    sizeof(responses[resp_count].data));
                
                if (parsed_len > 0) {
                    responses[resp_count].data_len = parsed_len;
                    responses[resp_count].timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
                    responses[resp_count].rx_status = 0;
                    resp_count++;
                }
            }
            
            line_start = line_end + 1;
        }
        
        *num_responses = resp_count;
        
        if (resp_count == 0 && status == STN_J2534_STATUS_OK) {
            return STN_J2534_STATUS_NO_DATA;
        }
    }
    
    return status;
}

stn_j2534_status_t stn_j2534_read_voltage(uint32_t *voltage_mv)
{
    char response[64];
    stn_j2534_status_t status;
    
    status = stn_j2534_send_cmd("ATRV\r", response, sizeof(response), 
                                 STN_J2534_CMD_TIMEOUT_MS);
    
    if (status == STN_J2534_STATUS_OK) {
        // Parse voltage (format: "12.6V" or "12.6")
        float voltage = 0;
        if (sscanf(response, "%f", &voltage) == 1) {
            *voltage_mv = (uint32_t)(voltage * 1000);
            return STN_J2534_STATUS_OK;
        }
    }
    
    *voltage_mv = 0;
    return status;
}

bool stn_j2534_is_legacy_active(void)
{
    return (s_config.protocol >= STN_PROTO_J1850PWM && 
            s_config.protocol <= STN_PROTO_ISO14230_FAST);
}

stn_protocol_t stn_j2534_get_protocol(void)
{
    return s_config.protocol;
}

#else  // HARDWARE_VER != WICAN_PRO

/* Stub implementations for non-Pro hardware */

esp_err_t stn_j2534_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

stn_j2534_status_t stn_j2534_reset(void)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_select_protocol(stn_protocol_t protocol)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_set_header(const uint8_t *header, uint8_t header_len)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_set_filter(const uint8_t *filter, const uint8_t *mask, uint8_t len)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_five_baud_init(uint8_t target_address, uint8_t *key_bytes)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_fast_init(void)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_set_timing(uint32_t p1_max, uint32_t p2_max,
                                         uint32_t p3_max, uint32_t p4_min)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_tester_present(void)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_start_keep_alive(uint32_t interval_ms)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_stop_keep_alive(void)
{
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_send_message(
    const uint8_t *data, 
    uint32_t data_len,
    stn_j2534_msg_t *responses,
    uint32_t max_responses,
    uint32_t *num_responses,
    uint32_t timeout_ms)
{
    *num_responses = 0;
    return STN_J2534_STATUS_CHIP_ERROR;
}

stn_j2534_status_t stn_j2534_read_voltage(uint32_t *voltage_mv)
{
    *voltage_mv = 0;
    return STN_J2534_STATUS_CHIP_ERROR;
}

bool stn_j2534_is_legacy_active(void)
{
    return false;
}

stn_protocol_t stn_j2534_get_protocol(void)
{
    return STN_PROTO_AUTO;
}

#endif  // HARDWARE_VER == WICAN_PRO
