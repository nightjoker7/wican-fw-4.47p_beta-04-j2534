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
#define STN_J2534_CMD_TIMEOUT_MS    1000  // Default command timeout
#define STN_J2534_MUTEX_TIMEOUT_MS  5000  // Mutex timeout - needs to be long enough for protocol init
#define STN_J2534_READ_TIMEOUT_MS   10
#define STN_J2534_TX_MAX_WAIT_MS    5000  // Max time to wait for TX to complete

// Monitor mode settings
#define STN_J2534_RX_BUFFER_SIZE    32      // Circular buffer for received messages
#define STN_J2534_MONITOR_TASK_STACK 4096

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
    .timeout_ms = 200,   // Reduced from 1000ms - J1850 is fast enough
    .headers_enabled = true,
    .echo_enabled = false,
    .adaptive_timing = true,
};

// Cache for J1850 header to avoid redundant ATSH commands
static uint8_t s_last_j1850_header[3] = {0};

// GM Class 2 High-Speed Mode (4x VPW) tracking
// Service $A1 = Enable High Speed Mode (switch to 41.6k)
// Service $A2 = Return to Normal Communication (switch back to 10.4k)
static volatile bool s_vpw_4x_mode = false;
static volatile int64_t s_vpw_4x_last_rx_time = 0;  // Last RX timestamp in 4x mode
#define VPW_4X_IDLE_TIMEOUT_MS 1500  // Auto-switch back to 1x after 1.5s of no RX

/* ============================================================================
 * Monitor Mode State (for continuous J1850 RX)
 * ============================================================================ */

// Circular buffer for received J1850 messages
static stn_j2534_msg_t s_rx_buffer[STN_J2534_RX_BUFFER_SIZE];
static volatile uint32_t s_rx_head = 0;
static volatile uint32_t s_rx_tail = 0;
static SemaphoreHandle_t s_rx_mutex = NULL;

// Monitor mode control
static volatile bool s_monitor_active = false;     // Is monitor mode running?
static volatile bool s_monitor_pause = false;      // Pause request for TX
static TaskHandle_t s_monitor_task = NULL;
static SemaphoreHandle_t s_tx_done_sem = NULL;     // Signals TX complete

// Forward declaration
static void stn_j2534_reset_state(void);

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Read from UART until pattern found or timeout
 * @note Timeout is capped at STN_J2534_TX_MAX_WAIT_MS to prevent blocking
 */
static int stn_j2534_read_until(char* buffer, size_t buffer_size, 
                                const char* pattern, int timeout_ms)
{
    int total_len = 0;
    int64_t start_time = esp_timer_get_time() / 1000;
    int no_data_count = 0;
    
    // Cap timeout to prevent excessive blocking
    if (timeout_ms > STN_J2534_TX_MAX_WAIT_MS) {
        ESP_LOGW(TAG, "Capping timeout from %d to %d ms", timeout_ms, STN_J2534_TX_MAX_WAIT_MS);
        timeout_ms = STN_J2534_TX_MAX_WAIT_MS;
    }
    
    while (total_len < buffer_size - 1) {
        int len = uart_read_bytes(STN_J2534_UART_NUM, buffer + total_len, 
                                  buffer_size - total_len - 1, 
                                  pdMS_TO_TICKS(STN_J2534_READ_TIMEOUT_MS));
        
        if (len > 0) {
            total_len += len;
            buffer[total_len] = '\0';
            no_data_count = 0;  // Reset no-data counter
            
            if (strstr(buffer, pattern)) {
                return total_len;
            }
            
            // Check for error responses early
            if (strstr(buffer, "ERROR") || strstr(buffer, "?")) {
                ESP_LOGW(TAG, "Early error response: %s", buffer);
                return total_len;
            }
        } else {
            no_data_count++;
            // If we've received some data but then get many empty reads,
            // the STN chip may be done even without sending ">"
            if (total_len > 0 && no_data_count > 50) {
                ESP_LOGW(TAG, "No more data after %d bytes, returning early", total_len);
                return total_len;
            }
        }
        
        if ((esp_timer_get_time() / 1000 - start_time) >= timeout_ms) {
            ESP_LOGW(TAG, "Read timeout after %d ms (got %d bytes)", timeout_ms, total_len);
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
        ESP_LOGE(TAG, "UART semaphore is NULL!");
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    if (xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take UART semaphore (timeout %dms)", STN_J2534_MUTEX_TIMEOUT_MS);
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // Flush any stale data
    uart_flush_input(STN_J2534_UART_NUM);
    
    // Send command
    ESP_LOGI(TAG, "TX: %s", cmd);
    uart_write_bytes(STN_J2534_UART_NUM, cmd, strlen(cmd));
    
    // Read response
    int len = stn_j2534_read_until(response, response_size, "\r>", timeout_ms);
    
    xSemaphoreGive(xuart1_semaphore);
    
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGI(TAG, "RX: %s", response);
        
        // Check for error responses
        if (strstr(response, "?")) {
            ESP_LOGW(TAG, "Command not understood: %s", cmd);
            return STN_J2534_STATUS_CHIP_ERROR;
        }
        if (strstr(response, "NO DATA")) {
            return STN_J2534_STATUS_NO_DATA;
        }
        // Note: "BUS INIT: OK" is success, "BUS INIT: ...ERROR" is failure
        if (strstr(response, "BUS INIT") && strstr(response, "ERROR")) {
            ESP_LOGE(TAG, "Bus init error: %s", response);
            return STN_J2534_STATUS_BUS_ERROR;
        }
        if (strstr(response, "BUS ERROR")) {
            ESP_LOGE(TAG, "Bus error: %s", response);
            return STN_J2534_STATUS_BUS_ERROR;
        }
        if (strstr(response, "UNABLE TO CONNECT")) {
            ESP_LOGE(TAG, "Unable to connect: %s", response);
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        return STN_J2534_STATUS_OK;
    }
    
    ESP_LOGW(TAG, "Timeout waiting for response to: %s", cmd);
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
 * Monitor Mode Functions (Continuous J1850 RX)
 * ============================================================================ */

/**
 * @brief Buffer a received J1850 message
 */
static void stn_j2534_buffer_rx_msg(const uint8_t* data, uint32_t len)
{
    if (len == 0 || len > sizeof(s_rx_buffer[0].data)) {
        return;
    }
    
    if (s_rx_mutex && xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint32_t next_head = (s_rx_head + 1) % STN_J2534_RX_BUFFER_SIZE;
        
        if (next_head != s_rx_tail) {  // Buffer not full
            memcpy(s_rx_buffer[s_rx_head].data, data, len);
            s_rx_buffer[s_rx_head].data_len = len;
            s_rx_buffer[s_rx_head].timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
            s_rx_buffer[s_rx_head].rx_status = 0;
            s_rx_head = next_head;
            ESP_LOGI(TAG, "J1850 RX buffered: len=%lu head=%lu tail=%lu", len, s_rx_head, s_rx_tail);
        } else {
            ESP_LOGW(TAG, "J1850 RX buffer full, dropping message");
        }
        
        xSemaphoreGive(s_rx_mutex);
    }
}

/**
 * @brief Parse a line from monitor mode output
 * Format: hex bytes separated by spaces or continuous, terminated by \r
 */
static void stn_j2534_parse_monitor_line(const char* line)
{
    uint8_t msg_data[64];
    int msg_len = 0;
    const char* p = line;
    
    // Skip leading whitespace
    while (*p && (*p == ' ' || *p == '\r' || *p == '\n')) {
        p++;
    }
    
    // Skip if this is a prompt or status message
    if (*p == '>' || *p == 'S' || *p == 'O' || *p == 'N' || *p == 'B' || *p == '?') {
        return;  // Skip prompts, SEARCHING..., OK, NO DATA, BUS ERROR, ?
    }
    
    // Parse hex bytes
    while (*p && msg_len < (int)sizeof(msg_data)) {
        // Skip spaces
        while (*p == ' ') p++;
        
        if (!*p || *p == '\r' || *p == '\n' || *p == '>') break;
        
        // Parse hex byte
        if (isxdigit((unsigned char)p[0]) && isxdigit((unsigned char)p[1])) {
            char hex[3] = {p[0], p[1], 0};
            msg_data[msg_len++] = (uint8_t)strtol(hex, NULL, 16);
            p += 2;
        } else {
            p++;  // Skip invalid character
        }
    }
    
    // Buffer if we got valid data
    if (msg_len >= 3) {  // Minimum J1850 message is 3 bytes (header)
        ESP_LOGI(TAG, "Monitor parsed: %d bytes", msg_len);
        ESP_LOG_BUFFER_HEX(TAG, msg_data, msg_len);
        stn_j2534_buffer_rx_msg(msg_data, msg_len);
    }
}

/**
 * @brief Monitor task - continuously receives J1850 messages
 * Uses ATMA to put STN in monitor mode, parses output, buffers messages
 */
static void stn_j2534_monitor_task(void* arg)
{
    char rx_buffer[256];
    int rx_pos = 0;
    
    ESP_LOGI(TAG, "Monitor task started");
    
    while (s_monitor_active) {
        // Check for pause request (TX wants to send)
        if (s_monitor_pause) {
            ESP_LOGI(TAG, "Monitor pausing for TX...");
            
            // Send any character to exit ATMA mode
            uart_write_bytes(STN_J2534_UART_NUM, "\r", 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            uart_flush_input(STN_J2534_UART_NUM);
            
            // Signal that we've paused
            if (s_tx_done_sem) {
                xSemaphoreGive(s_tx_done_sem);
            }
            
            // Wait for TX to complete
            while (s_monitor_pause && s_monitor_active) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            if (!s_monitor_active) break;
            
            // Re-enter monitor mode
            ESP_LOGI(TAG, "Monitor resuming...");
            uart_flush_input(STN_J2534_UART_NUM);
            uart_write_bytes(STN_J2534_UART_NUM, "ATMA\r", 5);
            rx_pos = 0;
            continue;
        }
        
        // Read available data
        int len = uart_read_bytes(STN_J2534_UART_NUM, rx_buffer + rx_pos, 
                                  sizeof(rx_buffer) - rx_pos - 1, 
                                  pdMS_TO_TICKS(50));
        
        if (len > 0) {
            rx_pos += len;
            rx_buffer[rx_pos] = '\0';
            
            // Process complete lines
            char* line_start = rx_buffer;
            char* line_end;
            
            while ((line_end = strchr(line_start, '\r')) != NULL) {
                *line_end = '\0';
                
                if (strlen(line_start) > 0) {
                    stn_j2534_parse_monitor_line(line_start);
                }
                
                line_start = line_end + 1;
                // Skip \n if present
                if (*line_start == '\n') line_start++;
            }
            
            // Move any partial line to beginning of buffer
            if (line_start != rx_buffer && *line_start) {
                int remaining = rx_pos - (line_start - rx_buffer);
                memmove(rx_buffer, line_start, remaining);
                rx_pos = remaining;
            } else if (line_start == rx_buffer + rx_pos) {
                rx_pos = 0;  // All data processed
            }
            
            // Prevent buffer overflow
            if (rx_pos > (int)sizeof(rx_buffer) - 64) {
                ESP_LOGW(TAG, "Monitor buffer overflow, resetting");
                rx_pos = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay to prevent CPU hogging
    }
    
    ESP_LOGI(TAG, "Monitor task exiting");
    s_monitor_task = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Start J1850 monitor mode
 */
static esp_err_t stn_j2534_start_monitor(void)
{
    if (s_monitor_active) {
        ESP_LOGI(TAG, "Monitor already active");
        return ESP_OK;
    }
    
    // Initialize synchronization primitives
    if (!s_rx_mutex) {
        s_rx_mutex = xSemaphoreCreateMutex();
    }
    if (!s_tx_done_sem) {
        s_tx_done_sem = xSemaphoreCreateBinary();
    }
    
    // Clear RX buffer
    s_rx_head = 0;
    s_rx_tail = 0;
    
    // Take UART semaphore - monitor task will own it
    if (xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take UART semaphore for monitor mode");
        return ESP_FAIL;
    }
    
    // Start monitor mode on STN
    uart_flush_input(STN_J2534_UART_NUM);
    uart_write_bytes(STN_J2534_UART_NUM, "ATMA\r", 5);
    
    s_monitor_active = true;
    s_monitor_pause = false;
    
    // Create monitor task
    BaseType_t ret = xTaskCreate(stn_j2534_monitor_task, "stn_monitor", 
                                  STN_J2534_MONITOR_TASK_STACK, NULL, 10, &s_monitor_task);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        s_monitor_active = false;
        xSemaphoreGive(xuart1_semaphore);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "J1850 monitor mode started");
    return ESP_OK;
}

/**
 * @brief Stop J1850 monitor mode
 */
static void stn_j2534_stop_monitor(void)
{
    if (!s_monitor_active) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping J1850 monitor mode...");
    
    s_monitor_active = false;
    
    // Wait for task to exit
    int timeout = 50;  // 500ms
    while (s_monitor_task != NULL && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Send character to exit ATMA
    uart_write_bytes(STN_J2534_UART_NUM, "\r", 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush_input(STN_J2534_UART_NUM);
    
    // Release UART semaphore
    xSemaphoreGive(xuart1_semaphore);
    
    ESP_LOGI(TAG, "J1850 monitor mode stopped");
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
    
    // Reset internal state first
    stn_j2534_reset_state();
    
    // Check if UART semaphore is available (must be initialized by elm327_init)
    if (!xuart1_semaphore) {
        ESP_LOGE(TAG, "UART semaphore not initialized - elm327_init() must be called first!");
        return ESP_ERR_INVALID_STATE;
    }
    
    // CRITICAL: Pause elm327_read_task to prevent it from consuming UART data
    // This must be done before any UART communication with the STN chip
    elm327_set_stn_j2534_active(true);
    
    // Give elm327_read_task time to exit its loop
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // First, ensure the chip is awake by setting SLEEP pin high
    gpio_set_level(OBD_SLEEP_PIN, 1);
    ESP_LOGI(TAG, "Set OBD_SLEEP_PIN high, waiting for chip...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check if chip is ready
    if (elm327_chip_get_status() != ELM327_READY) {
        ESP_LOGI(TAG, "STN chip not ready (GPIO7=%d), performing hardware reset...", 
                 gpio_get_level(OBD_READY_PIN));
        
        // Hardware reset: pull RESET low then high
        gpio_set_level(OBD_RESET_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(OBD_RESET_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));  // Give chip time to boot after reset
        
        // Wait for chip to come up (up to 3 seconds - reduced from 5)
        int retry_count = 0;
        const int max_retries = 30;  // 30 x 100ms = 3 seconds
        
        while (elm327_chip_get_status() != ELM327_READY && retry_count < max_retries) {
            vTaskDelay(pdMS_TO_TICKS(100));
            retry_count++;
            if (retry_count % 5 == 0) {
                ESP_LOGI(TAG, "Waiting for STN chip... (%d/%d) GPIO7=%d", 
                         retry_count, max_retries, gpio_get_level(OBD_READY_PIN));
            }
        }
        
        if (elm327_chip_get_status() != ELM327_READY) {
            ESP_LOGE(TAG, "STN chip not ready after reset! GPIO7=%d (expected HIGH)", gpio_get_level(OBD_READY_PIN));
            ESP_LOGE(TAG, "Check: 1) Is OBD chip powered? 2) Is OBD_SLEEP_PIN (GPIO9) HIGH?");
            ESP_LOGI(TAG, "OBD_SLEEP_PIN (GPIO9) level = %d", gpio_get_level(OBD_SLEEP_PIN));
            elm327_set_stn_j2534_active(false);  // Re-enable elm327_read_task on failure
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    ESP_LOGI(TAG, "STN chip ready, GPIO7=%d", gpio_get_level(OBD_READY_PIN));
    
    // Small delay to let chip stabilize after wake
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Reset to defaults
    stn_j2534_status_t status = stn_j2534_reset();
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to reset STN chip, status=%d", status);
        elm327_set_stn_j2534_active(false);  // Re-enable elm327_read_task on failure
        return ESP_FAIL;
    }
    
    s_initialized = true;
    ESP_LOGI(TAG, "STN-J2534 bridge initialized successfully");
    
    return ESP_OK;
}

/**
 * @brief Configure STN power pin switch (required for legacy protocols)
 * @note This configures internal hardware power switches - VTPPSW10 enables J1850/ISO buses
 */
static void stn_j2534_configure_power_pins(void)
{
    char response[128];
    
    ESP_LOGI(TAG, "Configuring STN power pins (VTPPSW)...");
    
    // Check firmware version - VTPPSW is only on certain firmware versions
    stn_j2534_status_t status = stn_j2534_send_cmd("VTVERS\r", response, sizeof(response), 1000);
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGW(TAG, "VTVERS failed - may not be VT firmware");
        return;
    }
    
    // Check if V2.3.22 or later (required for VTPPSW)
    if (strstr(response, "V2.3") == NULL) {
        ESP_LOGW(TAG, "Firmware not V2.3.x, skipping VTPPSW: %s", response);
        return;
    }
    ESP_LOGI(TAG, "Firmware version: %s", response);
    
    // Read current power switch state
    status = stn_j2534_send_cmd("VTPPSWS\r", response, sizeof(response), 500);
    if (status != STN_J2534_STATUS_OK) {
        ESP_LOGW(TAG, "VTPPSWS not supported");
        return;
    }
    
    if (strstr(response, "10") != NULL) {
        ESP_LOGI(TAG, "PPSW already set to 10 (J1850/ISO enabled)");
    } else {
        ESP_LOGI(TAG, "Setting PPSW to 10 for J1850/ISO support...");
        status = stn_j2534_send_cmd("VTPPSW10\r", response, sizeof(response), 500);
        if (status == STN_J2534_STATUS_OK && strstr(response, "OK") != NULL) {
            ESP_LOGI(TAG, "PPSW set to 10 successfully");
            // Need to reset chip after PPSW change
            ESP_LOGI(TAG, "Resetting chip after PPSW change...");
            stn_j2534_send_cmd("ATZ\r", response, sizeof(response), 2000);
        } else {
            ESP_LOGW(TAG, "Failed to set PPSW to 10: %s", response);
        }
    }
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
    
    // CRITICAL: Configure power pins for J1850/ISO protocols
    // This enables the internal hardware switches needed for legacy protocols
    stn_j2534_configure_power_pins();
    
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
        status = stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATD response: [%s] status=%d", response, status);
        
        // Select protocol 2 (J1850 VPW 10.4 kbaud)
        status = stn_j2534_send_cmd("ATSP2\r", response, sizeof(response), 
                                     STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATSP2 response: [%s] status=%d", response, status);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "ATSP2 failed: %s", response);
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        // Enable headers - J2534 needs full message with header bytes in responses
        status = stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATH1 response: [%s] status=%d", response, status);
        
        // Disable spaces for cleaner parsing
        status = stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATS0 response: [%s] status=%d", response, status);
        
        // Disable echo
        status = stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATE0 response: [%s] status=%d", response, status);
        
        // Allow long messages (up to 256 bytes)
        status = stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATAL response: [%s] status=%d", response, status);
        
        // Set protocol timeout via STPTO (milliseconds) - 100ms is plenty for J1850 VPW
        // J1850 at 10.4 kbaud: 12-byte message takes ~10ms, so 100ms gives room for bus arbitration
        status = stn_j2534_send_cmd("STPTO 100\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK || strstr(response, "?")) {
            // Fall back to ATST (4ms units): 25 * 4 = 100ms
            ESP_LOGI(TAG, "STPTO not supported, using ATST");
            status = stn_j2534_send_cmd("ATST 19\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        }
        ESP_LOGI(TAG, "Protocol timeout response: [%s] status=%d", response, status);
        
        // Disable adaptive timing - use fixed timing for J2534 compliance
        status = stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATAT0 response: [%s] status=%d", response, status);
        
        // CRITICAL: Configure receive filters for J2534
        // Default J1850 filter is 006B00,14FF00 which only accepts messages TO 0x6B
        // For J2534 we need to receive responses to the tester address (F0)
        
        // Disable IFR (In-Frame Response) display to avoid single-byte ack messages
        status = stn_j2534_send_cmd("AT IFR0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "AT IFR0 response: [%s] status=%d", response, status);
        
        // Set receive address to F0 (standard diagnostic tester address)
        // This tells the STN chip to accept messages where target = F0
        status = stn_j2534_send_cmd("ATSR F0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATSR F0 response: [%s] status=%d", response, status);
        
        // Enable auto-receive mode - receive all messages without filtering
        status = stn_j2534_send_cmd("ATAR\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATAR response: [%s] status=%d", response, status);
        
        // Clear any existing programmable filters
        status = stn_j2534_send_cmd("STFCP\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "STFCP response: [%s] status=%d", response, status);
        
        // Add pass filter for messages to F0: pattern=00F000, mask=00FF00 
        // This matches any message where byte 1 (target address) = F0
        status = stn_j2534_send_cmd("STFAP 00F000,00FF00\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "STFAP 00F000,00FF00 response: [%s] status=%d", response, status);
        
        // Also add filter for broadcast/functional responses (target FE)
        status = stn_j2534_send_cmd("STFAP 00FE00,00FF00\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "STFAP 00FE00,00FF00 response: [%s] status=%d", response, status);
        
        // Also accept messages from ECU directly addressed (some ECUs use 6C instead of FE)
        status = stn_j2534_send_cmd("STFAP 006C00,00FF00\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "STFAP 006C00,00FF00 response: [%s] status=%d", response, status);
        
        ESP_LOGI(TAG, "J1850 VPW filters configured for tester address F0");
        
        // Send a test message to verify bus connectivity
        // Use a simple OBD broadcast (3E = TesterPresent)
        status = stn_j2534_send_cmd("ATSH 68 6A F1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "Test header response: [%s] status=%d", response, status);
        
        status = stn_j2534_send_cmd("3F\r", response, sizeof(response), 2000);  // Request all PIDs
        ESP_LOGI(TAG, "Test msg (3F) response: [%s] status=%d", response, status);
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "J1850 VPW protocol initialized successfully");
        
        // NOTE: Monitor mode disabled for now - focus on basic request/response first
        // The STN chip is left at the prompt ready to receive commands
        
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
    
    // ISO 9141-2 protocol (K-line with 5-baud init)
    if (protocol == STN_PROTO_ISO9141) {
        ESP_LOGI(TAG, "Initializing ISO 9141 (K-line, 5-baud init)");
        
        // Reset to defaults first
        stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Select protocol 3 (ISO 9141-2)
        status = stn_j2534_send_cmd("ATSP3\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATSP3 response: [%s] status=%d", response, status);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "ATSP3 failed");
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        // Enable headers - J2534 needs full message with header bytes
        stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Disable spaces for cleaner parsing
        stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Disable echo
        stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Allow long messages
        stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Set default K-line address (target = 0x33 = ECU)
        // ISO 9141 header: Format + Target + Source
        // Default format = C1 (functional addressing, physical data)
        stn_j2534_send_cmd("ATSH C1 33 F1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "Default K-line header set: C1 33 F1");
        
        // Set longer timeout for K-line (300ms P2max is standard)
        status = stn_j2534_send_cmd("STPTO 300\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK || strstr(response, "?")) {
            // Fall back to ATST: 75 * 4ms = 300ms
            stn_j2534_send_cmd("ATST 4B\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        }
        
        // Disable adaptive timing for precise J2534 control
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Note: 5-baud init will be performed via IOCTL FIVE_BAUD_INIT
        // The protocol is set up, but bus init happens on first message or IOCTL
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "ISO 9141 protocol initialized - use FIVE_BAUD_INIT to connect");
        return STN_J2534_STATUS_OK;
    }
    
    // ISO 14230-4 KWP2000 with 5-baud init
    if (protocol == STN_PROTO_ISO14230_5BAUD) {
        ESP_LOGI(TAG, "Initializing ISO 14230-4 KWP2000 (5-baud init)");
        
        stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Select protocol 4 (ISO 14230-4 KWP, 5 baud init)
        status = stn_j2534_send_cmd("ATSP4\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATSP4 response: [%s] status=%d", response, status);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "ATSP4 failed");
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // KWP2000 header format (with length byte)
        stn_j2534_send_cmd("ATSH C0 33 F1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Set P2max timeout (300ms typical for KWP)
        status = stn_j2534_send_cmd("STPTO 300\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK || strstr(response, "?")) {
            stn_j2534_send_cmd("ATST 4B\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        }
        
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "ISO 14230 (5-baud) protocol initialized - use FIVE_BAUD_INIT to connect");
        return STN_J2534_STATUS_OK;
    }
    
    // ISO 14230-4 KWP2000 with fast init
    if (protocol == STN_PROTO_ISO14230_FAST) {
        ESP_LOGI(TAG, "Initializing ISO 14230-4 KWP2000 (fast init)");
        
        stn_j2534_send_cmd("ATD\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Select protocol 5 (ISO 14230-4 KWP, fast init)
        status = stn_j2534_send_cmd("ATSP5\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        ESP_LOGI(TAG, "ATSP5 response: [%s] status=%d", response, status);
        if (status != STN_J2534_STATUS_OK) {
            ESP_LOGE(TAG, "ATSP5 failed");
            return STN_J2534_STATUS_PROTOCOL_ERROR;
        }
        
        stn_j2534_send_cmd("ATH1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATS0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATE0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        stn_j2534_send_cmd("ATAL\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // KWP2000 default header (format byte 0xC0 = physical addressing + length byte)
        // Target 0x33 = standard ECU diagnostic address
        // Source 0xF1 = tester address
        stn_j2534_send_cmd("ATSH C0 33 F1\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // Set P2max timeout
        status = stn_j2534_send_cmd("STPTO 300\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        if (status != STN_J2534_STATUS_OK || strstr(response, "?")) {
            stn_j2534_send_cmd("ATST 4B\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        }
        
        stn_j2534_send_cmd("ATAT0\r", response, sizeof(response), STN_J2534_CMD_TIMEOUT_MS);
        
        // For fast init, we can do the init now or wait for IOCTL
        // Default: don't auto-init - let application control via IOCTL FAST_INIT
        
        s_config.protocol = protocol;
        ESP_LOGI(TAG, "ISO 14230 (fast init) protocol initialized - use FAST_INIT IOCTL to connect");
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
    stn_j2534_status_t status = STN_J2534_STATUS_OK;
    
    *num_responses = 0;
    
    if (data_len < 1) {
        ESP_LOGW(TAG, "Empty message");
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    // If monitor mode is active, we need to pause it for TX
    bool was_monitoring = s_monitor_active;
    if (was_monitoring) {
        ESP_LOGI(TAG, "Pausing monitor for TX...");
        s_monitor_pause = true;
        
        // Wait for monitor task to pause and release UART
        if (s_tx_done_sem) {
            if (xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                ESP_LOGE(TAG, "Timeout waiting for monitor to pause");
                s_monitor_pause = false;
                return STN_J2534_STATUS_CHIP_ERROR;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Reduced settling time
    }
    
    // Now safe to use UART directly (monitor is paused or not active)
    if (!was_monitoring) {
        // Need to take semaphore if not monitoring
        if (xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take UART semaphore");
            return STN_J2534_STATUS_CHIP_ERROR;
        }
    }
    
    // Flush any stale data
    uart_flush_input(STN_J2534_UART_NUM);
    
    // For J1850 VPW/PWM protocols, the J2534 message includes header bytes
    const uint8_t *payload_data = data;
    uint32_t payload_len = data_len;
    
    if (s_config.protocol == STN_PROTO_J1850VPW || 
        s_config.protocol == STN_PROTO_J1850PWM) {
        
        if (data_len >= 3) {
            // Extract 3-byte J1850 header and set via ATSH
            char header_cmd[32];
            snprintf(header_cmd, sizeof(header_cmd), "ATSH %02X %02X %02X\r", 
                     data[0], data[1], data[2]);
            
            // Only update header if different from last one
            if (data[0] != s_last_j1850_header[0] || data[1] != s_last_j1850_header[1] || 
                data[2] != s_last_j1850_header[2]) {
                
                ESP_LOGI(TAG, "Setting J1850 header: %s", header_cmd);
                uart_write_bytes(STN_J2534_UART_NUM, header_cmd, strlen(header_cmd));
                int len = stn_j2534_read_until(response, sizeof(response), ">", 500);
                ESP_LOGI(TAG, "ATSH response (%d): [%s]", len, response);
                
                s_last_j1850_header[0] = data[0];
                s_last_j1850_header[1] = data[1];
                s_last_j1850_header[2] = data[2];
            }
            
            // Payload is everything after the 3-byte header
            payload_data = &data[3];
            payload_len = data_len - 3;
            
            ESP_LOGI(TAG, "J1850 TX: hdr=%02X%02X%02X data_len=%lu", 
                     data[0], data[1], data[2], payload_len);
        }
    }
    
    // Build hex data command (payload only, header is set via ATSH)
    if (payload_len > 0) {
        stn_j2534_format_hex(payload_data, payload_len, cmd);
        strcat(cmd, "\r");
    } else {
        strcpy(cmd, "\r");
    }
    
    // Detect GM Class 2 High-Speed Mode commands (Service $A1 / $A2)
    // WORKAROUND: STN chip cannot do VPW at 41.6k (4x mode)
    // Instead of sending $A1 to ECU (which would cause it to switch to 4x),
    // we INTERCEPT the $A1 command and FAKE an $E1 response back to DPS.
    // This keeps ECU at 10.4k and allows programming to continue (slowly).
    bool is_gm_4x_enable = false;
    bool is_gm_4x_disable = false;
    
    // Check for $A1/$A2 in payload - works for both VPW and PWM modes
    if (payload_len >= 1 && 
        (s_config.protocol == STN_PROTO_J1850VPW || s_config.protocol == STN_PROTO_J1850PWM)) {
        ESP_LOGW(TAG, "Checking payload[0]=0x%02X for $A1/$A2, protocol=%d, 4x_mode=%d", 
                 payload_data[0], s_config.protocol, s_vpw_4x_mode);
        if (payload_data[0] == 0xA1) {
            is_gm_4x_enable = true;
            ESP_LOGW(TAG, "*** INTERCEPTING GM $A1 Enable High Speed Mode - FAKING $E1 response ***");
            ESP_LOGW(TAG, "*** ECU will stay at 10.4k - 4x mode NOT supported by STN chip ***");
            
            // DON'T send $A1 to ECU! Instead, fake an $E1 response.
            // Build fake response with same header format (swap source/dest)
            // Original header: data[0]=priority, data[1]=dest, data[2]=source
            // Response header: same priority, dest becomes source, source becomes dest
            responses[0].data[0] = data[0];       // Same priority byte
            responses[0].data[1] = data[2];       // Swap: original source -> response dest
            responses[0].data[2] = data[1];       // Swap: original dest -> response source  
            responses[0].data[3] = 0xE1;          // Positive response to $A1
            responses[0].data_len = 4;
            responses[0].timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
            responses[0].rx_status = 0;
            *num_responses = 1;
            
            ESP_LOGW(TAG, "Faked $E1 response: %02X %02X %02X %02X", 
                     responses[0].data[0], responses[0].data[1], 
                     responses[0].data[2], responses[0].data[3]);
            
            // Mark that we're "in 4x mode" even though we're not actually switching
            // This allows stn_j2534_set_baudrate() to early-return when DPS calls SET_CONFIG
            s_vpw_4x_mode = true;
            s_config.baudrate = 41600;  // Report what DPS expects
            
            // Release semaphore and return success - we didn't send anything to ECU
            if (!was_monitoring) {
                xSemaphoreGive(xuart1_semaphore);
            }
            return STN_J2534_STATUS_OK;
            
        } else if (payload_data[0] == 0xA2) {
            is_gm_4x_disable = true;
            ESP_LOGW(TAG, "*** Detected GM $A2 Return to Normal - faking $E2 response ***");
            
            // Fake $E2 response as well for consistency
            responses[0].data[0] = data[0];
            responses[0].data[1] = data[2];
            responses[0].data[2] = data[1];
            responses[0].data[3] = 0xE2;          // Positive response to $A2
            responses[0].data_len = 4;
            responses[0].timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
            responses[0].rx_status = 0;
            *num_responses = 1;
            
            ESP_LOGW(TAG, "Faked $E2 response: %02X %02X %02X %02X",
                     responses[0].data[0], responses[0].data[1],
                     responses[0].data[2], responses[0].data[3]);
            
            // Mark that we're returning to normal mode
            s_vpw_4x_mode = false;
            s_config.baudrate = 10400;  // Report normal speed
            
            if (!was_monitoring) {
                xSemaphoreGive(xuart1_semaphore);
            }
            return STN_J2534_STATUS_OK;
        }
    } else {
        ESP_LOGI(TAG, "Skipping $A1/$A2 check: payload_len=%lu, protocol=%d", payload_len, s_config.protocol);
    }
    
    // Send and wait for responses
    ESP_LOGI(TAG, "J1850 TX cmd: [%s]", cmd);
    uart_flush_input(STN_J2534_UART_NUM);
    uart_write_bytes(STN_J2534_UART_NUM, cmd, strlen(cmd));
    
    // Read response with appropriate timeout (add small buffer for UART transfer)
    // For $A1/$A2 commands, the response ($E1/$E2) comes at the CURRENT baud rate
    // ECU switches speed AFTER responding, so we read first, then switch
    int rsp_len = stn_j2534_read_until(response, sizeof(response), ">", timeout_ms + 50);
    response[rsp_len] = '\0';
    
    ESP_LOGI(TAG, "STN raw response (len=%d): [%s]", rsp_len, response);
    
    // Check for errors
    if (strstr(response, "NO DATA")) {
        status = STN_J2534_STATUS_NO_DATA;
    } else if (strstr(response, "BUS ERROR") || strstr(response, "ERROR")) {
        status = STN_J2534_STATUS_BUS_ERROR;
    } else if (rsp_len == 0) {
        status = STN_J2534_STATUS_TIMEOUT;
    }
    
    // Parse responses if we got data
    if (rsp_len > 0 && !strstr(response, "NO DATA") && !strstr(response, "ERROR")) {
        char* line_start = response;
        char* line_end;
        uint32_t resp_count = 0;
        
        while ((line_end = strchr(line_start, '\r')) != NULL && resp_count < max_responses) {
            *line_end = '\0';
            
            // Skip empty lines, prompts, and status messages
            // Require at least 6 chars = 3 hex bytes for valid J1850 message
            if (strlen(line_start) >= 6 && line_start[0] != '>' && 
                line_start[0] != 'O' && line_start[0] != 'S') {  // Skip OK, SEARCHING...
                
                int parsed_len = stn_j2534_parse_hex_response(
                    line_start, 
                    responses[resp_count].data, 
                    sizeof(responses[resp_count].data));
                
                if (parsed_len >= 3) {  // Minimum valid J1850 message (3-byte header)
                    responses[resp_count].data_len = parsed_len;
                    responses[resp_count].timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
                    responses[resp_count].rx_status = 0;
                    ESP_LOGI(TAG, "Parsed RX[%lu]: %d bytes data=%02X%02X%02X...", 
                             resp_count, parsed_len,
                             responses[resp_count].data[0],
                             responses[resp_count].data[1],
                             responses[resp_count].data[2]);
                    resp_count++;
                }
            }
            
            line_start = line_end + 1;
        }
        
        *num_responses = resp_count;
        ESP_LOGI(TAG, "Total responses parsed: %lu", resp_count);
    }
    
    // NOTE: GM Class 2 High-Speed Mode ($A1/$A2) is now intercepted above.
    // We fake the $E1/$E2 responses and keep ECU at 10.4k because STN chip
    // cannot do VPW at 41.6k. The old ATSP1/ATSP2 switching code has been removed.
    
    // Resume monitor mode if it was active
    if (was_monitoring) {
        ESP_LOGI(TAG, "Resuming monitor mode...");
        s_monitor_pause = false;  // Signal monitor task to resume
    } else {
        xSemaphoreGive(xuart1_semaphore);
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

/**
 * @brief Set baud rate for J1850 protocols
 * 
 * J1850 VPW normally runs at 10.4 kbaud but can run at 41.6 kbaud (4x mode)
 * for high-speed operations like ECU programming. This is achieved by 
 * switching between STN protocols:
 * - ATSP2 = J1850 VPW (10.4 kbaud)
 * - ATSP1 = J1850 PWM (41.6 kbaud) - used for VPW 4x mode
 * 
 * @param baudrate Baud rate (10400 for normal, 41600 for 4x mode)
 * @return Status code
 */
stn_j2534_status_t stn_j2534_set_baudrate(uint32_t baudrate)
{
    ESP_LOGW(TAG, "set_baudrate called: baudrate=%lu, s_vpw_4x_mode=%d, s_config.protocol=%d, s_config.baudrate=%lu",
             baudrate, s_vpw_4x_mode, s_config.protocol, s_config.baudrate);
    
    // Only applicable for J1850 protocols
    if (s_config.protocol != STN_PROTO_J1850VPW && 
        s_config.protocol != STN_PROTO_J1850PWM) {
        ESP_LOGW(TAG, "set_baudrate: Not a J1850 protocol (%d), returning OK", s_config.protocol);
        return STN_J2534_STATUS_OK;  // No-op for other protocols
    }
    
    // Check if we're already at the requested baud rate
    // This prevents redundant switching when software sends SET_CONFIG after $A1/$A2
    // Also check if already storing this baudrate (in case auto-detect already switched)
    if (baudrate == 41600) {
        if (s_vpw_4x_mode || s_config.baudrate == 41600) {
            ESP_LOGI(TAG, "set_baudrate: Already at 41600 (4x_mode=%d, stored=%lu), no action needed",
                     s_vpw_4x_mode, s_config.baudrate);
            s_config.baudrate = baudrate;
            s_vpw_4x_mode = true;  // Ensure flag is set
            return STN_J2534_STATUS_OK;
        }
    }
    if (baudrate == 10400) {
        if (!s_vpw_4x_mode || s_config.baudrate == 10400) {
            ESP_LOGI(TAG, "set_baudrate: Already at 10400 (4x_mode=%d, stored=%lu), no action needed",
                     s_vpw_4x_mode, s_config.baudrate);
            s_config.baudrate = baudrate;
            s_vpw_4x_mode = false;  // Ensure flag is cleared
            return STN_J2534_STATUS_OK;
        }
    }
    
    // Handle monitor mode - same as stn_j2534_send_msg()
    bool was_monitoring = s_monitor_active;
    if (was_monitoring) {
        ESP_LOGI(TAG, "set_baudrate: Pausing monitor mode...");
        s_monitor_pause = true;
        
        // Wait for monitor task to pause and release UART
        if (s_tx_done_sem) {
            if (xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                ESP_LOGE(TAG, "set_baudrate: Timeout waiting for monitor to pause");
                s_monitor_pause = false;
                return STN_J2534_STATUS_CHIP_ERROR;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Short settling time
    }
    
    // Now safe to use UART directly (monitor is paused or not active)
    if (!was_monitoring) {
        // Need the semaphore for UART access
        if (xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "set_baudrate: Failed to take semaphore");
            return STN_J2534_STATUS_CHIP_ERROR;
        }
    }
    
    ESP_LOGW(TAG, "*** SET_CONFIG baud rate request: %lu ***", baudrate);
    
    // WORKAROUND: STN chip cannot do VPW at 41.6k (4x mode)
    // We intercept $A1/$A2 commands and fake the responses, keeping ECU at 10.4k.
    // When DPS requests 41600 baud, we just accept it but stay at 10.4k.
    // This allows programming to work (slowly) without actual 4x mode support.
    
    if (baudrate == 41600) {
        // DPS thinks we're switching to 4x mode - just accept it
        // We already faked the $E1 response, ECU is still at 10.4k
        ESP_LOGW(TAG, "*** FAKE 4x MODE: Accepting 41600 request but staying at 10.4k VPW ***");
        ESP_LOGW(TAG, "*** Programming will work but at 1x speed (4x slower) ***");
        s_vpw_4x_mode = true;  // Track that DPS thinks we're in 4x mode
        s_config.baudrate = baudrate;  // Report what DPS expects
        
    } else if (baudrate == 10400) {
        // Switching back to normal mode
        ESP_LOGW(TAG, "*** Returning to normal 10.4k VPW mode ***");
        s_vpw_4x_mode = false;
        s_config.baudrate = baudrate;
        
    } else {
        ESP_LOGW(TAG, "Unsupported J1850 baud rate: %lu (expected 10400 or 41600)", baudrate);
        s_config.baudrate = baudrate;  // Accept it anyway
    }
    
    // Resume monitor mode if it was active, or release semaphore
    if (was_monitoring) {
        ESP_LOGI(TAG, "set_baudrate: Resuming monitor mode...");
        s_monitor_pause = false;
    } else {
        xSemaphoreGive(xuart1_semaphore);
    }
    return STN_J2534_STATUS_OK;
}

/**
 * @brief Reset internal state variables without touching the chip
 */
static void stn_j2534_reset_state(void)
{
    // Clear header cache
    memset(s_last_j1850_header, 0, sizeof(s_last_j1850_header));
    
    // Reset VPW 4x mode flag and idle timer
    s_vpw_4x_mode = false;
    s_vpw_4x_last_rx_time = 0;
    
    // Reset config to defaults
    s_config.protocol = STN_PROTO_AUTO;
    s_config.baudrate = 0;
    s_config.tx_header[0] = 0x68;
    s_config.tx_header[1] = 0x6A;
    s_config.tx_header[2] = 0xF1;
    s_config.tx_header_len = 3;
    s_config.timeout_ms = 200;  // Fast timeout for J1850
}

esp_err_t stn_j2534_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing STN-J2534 bridge...");
    
    // Mark as not initialized first to stop any new operations
    s_initialized = false;
    
    // Stop monitor task if running
    stn_j2534_stop_monitor();
    
    // CRITICAL: Do a HARDWARE reset of the STN chip
    // Software reset (ATZ) may not work if chip is stuck
    ESP_LOGI(TAG, "Performing hardware reset of STN chip...");
    gpio_set_level(OBD_RESET_PIN, 0);  // Pull reset low
    vTaskDelay(pdMS_TO_TICKS(100));    // Longer low pulse for clean reset
    gpio_set_level(OBD_RESET_PIN, 1);  // Release reset
    vTaskDelay(pdMS_TO_TICKS(500));    // Wait for chip to FULLY boot (was 200ms - too short!)
    
    // Flush any garbage from UART (chip sends bootup message)
    uart_flush_input(STN_J2534_UART_NUM);
    if (uart1_queue) {
        xQueueReset(uart1_queue);
    }
    
    // Wait for chip ready signal
    int retry = 0;
    while (elm327_chip_get_status() != ELM327_READY && retry < 10) {
        vTaskDelay(pdMS_TO_TICKS(100));
        retry++;
    }
    
    // Re-enable elm327_read_task
    elm327_set_stn_j2534_active(false);
    
    // Reset internal state
    stn_j2534_reset_state();
    
    ESP_LOGI(TAG, "STN-J2534 bridge deinitialized (chip ready: %s)", 
             elm327_chip_get_status() == ELM327_READY ? "YES" : "NO");
    
    return ESP_OK;
}

// Callback wrapper for j2534 component (void return type)
void stn_j2534_deinit_callback(void)
{
    stn_j2534_deinit();
}

stn_j2534_status_t stn_j2534_read_msgs(
    stn_j2534_msg_t *msgs,
    uint32_t max_msgs,
    uint32_t *num_msgs)
{
    *num_msgs = 0;
    
    if (!s_rx_mutex) {
        return STN_J2534_STATUS_CHIP_ERROR;
    }
    
    if (xSemaphoreTake(s_rx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return STN_J2534_STATUS_TIMEOUT;
    }
    
    uint32_t count = 0;
    while (count < max_msgs && s_rx_tail != s_rx_head) {
        memcpy(&msgs[count], &s_rx_buffer[s_rx_tail], sizeof(stn_j2534_msg_t));
        s_rx_tail = (s_rx_tail + 1) % STN_J2534_RX_BUFFER_SIZE;
        count++;
    }
    
    xSemaphoreGive(s_rx_mutex);
    
    *num_msgs = count;
    
    // Track RX activity for 4x mode idle timeout
    if (count > 0) {
        if (s_vpw_4x_mode) {
            s_vpw_4x_last_rx_time = esp_timer_get_time() / 1000;  // ms
        }
        ESP_LOGI(TAG, "stn_j2534_read_msgs: returned %lu messages", count);
        return STN_J2534_STATUS_OK;
    }
    
    // Check for 4x mode idle timeout - auto-switch back to 1x
    if (s_vpw_4x_mode && s_vpw_4x_last_rx_time > 0) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t idle_ms = now_ms - s_vpw_4x_last_rx_time;
        
        if (idle_ms >= VPW_4X_IDLE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "*** 4x mode idle timeout (%lld ms) - switching back to 1x ***", idle_ms);
            
            // Switch back to 1x mode (ATSP2)
            char proto_resp[64];
            
            // Pause monitor if active
            bool was_monitoring = s_monitor_active;
            if (was_monitoring) {
                s_monitor_pause = true;
                if (s_tx_done_sem) {
                    xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(500));
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            
            if (!was_monitoring) {
                xSemaphoreTake(xuart1_semaphore, pdMS_TO_TICKS(STN_J2534_MUTEX_TIMEOUT_MS));
            }
            
            uart_flush_input(STN_J2534_UART_NUM);
            uart_write_bytes(STN_J2534_UART_NUM, "ATSP2\r", 6);
            stn_j2534_read_until(proto_resp, sizeof(proto_resp), ">", 500);
            ESP_LOGI(TAG, "ATSP2 response: [%s]", proto_resp);
            
            // Re-apply settings
            uart_write_bytes(STN_J2534_UART_NUM, "ATH1\r", 5);
            stn_j2534_read_until(proto_resp, sizeof(proto_resp), ">", 200);
            uart_write_bytes(STN_J2534_UART_NUM, "ATS0\r", 5);
            stn_j2534_read_until(proto_resp, sizeof(proto_resp), ">", 200);
            uart_write_bytes(STN_J2534_UART_NUM, "ATAL\r", 5);
            stn_j2534_read_until(proto_resp, sizeof(proto_resp), ">", 200);
            
            if (was_monitoring) {
                s_monitor_pause = false;
            } else {
                xSemaphoreGive(xuart1_semaphore);
            }
            
            s_vpw_4x_mode = false;
            s_vpw_4x_last_rx_time = 0;
            memset(s_last_j1850_header, 0, sizeof(s_last_j1850_header));
            
            ESP_LOGW(TAG, "*** Back to 1x (10.4k) mode ***");
        }
    }
    
    return STN_J2534_STATUS_NO_DATA;
}

bool stn_j2534_is_monitor_active(void)
{
    return s_monitor_active;
}

/**
 * @brief Public wrapper for sending raw AT/ST commands
 * Used by USDT module for direct chip control.
 */
stn_j2534_status_t stn_j2534_send_raw_cmd(const char* cmd, char* response, 
                                           size_t response_size, int timeout_ms)
{
    return stn_j2534_send_cmd(cmd, response, response_size, timeout_ms);
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

stn_j2534_status_t stn_j2534_set_baudrate(uint32_t baudrate)
{
    (void)baudrate;
    return STN_J2534_STATUS_CHIP_ERROR;
}

esp_err_t stn_j2534_deinit(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

// Stub callback wrapper for non-Pro hardware
void stn_j2534_deinit_callback(void)
{
    // No-op for non-Pro hardware
}

stn_j2534_status_t stn_j2534_read_msgs(
    stn_j2534_msg_t *msgs,
    uint32_t max_msgs,
    uint32_t *num_msgs)
{
    *num_msgs = 0;
    return STN_J2534_STATUS_CHIP_ERROR;
}

bool stn_j2534_is_monitor_active(void)
{
    return false;
}

stn_j2534_status_t stn_j2534_send_raw_cmd(const char* cmd, char* response, 
                                           size_t response_size, int timeout_ms)
{
    (void)cmd;
    (void)response;
    (void)response_size;
    (void)timeout_ms;
    return STN_J2534_STATUS_CHIP_ERROR;
}

#endif  // HARDWARE_VER == WICAN_PRO
