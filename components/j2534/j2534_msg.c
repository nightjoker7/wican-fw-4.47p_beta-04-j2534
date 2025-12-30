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
 * @file j2534_msg.c
 * @brief J2534 Message Handling - Read and write message operations
 * 
 * This file contains:
 * - j2534_read_msgs() - Read messages from RX buffer with extended wait
 * - j2534_write_msgs() - Write messages with ISO-TP multi-frame support
 * - j2534_buffer_tx_indication() - Buffer TX echo messages
 */

#include "j2534_internal.h"
#include "can.h"
#include "stn_j2534.h"
#include "stn_j2534_usdt.h"
#include "hw_config.h"

#define TAG J2534_TAG

/* ============================================================================
 * TX Indication (Echo) Buffering
 * ============================================================================ */

/**
 * @brief Buffer a TX indication (echo) message
 *
 * J2534 requires that transmitted messages be echoed back to the application
 * with RxStatus = 0x01 (TX_MSG_TYPE) to confirm successful transmission.
 * This is critical for subnet discovery and other diagnostic operations.
 *
 * CRITICAL FIX: For large messages using ext_data, we only echo the first
 * 12 bytes (CAN ID + first frame data). Setting data_size to the original
 * message size would cause ReadMsgs to copy garbage memory past the buffer.
 */
void j2534_buffer_tx_indication(j2534_msg_t *orig_msg, uint32_t protocol_id)
{
    uint32_t next_head = (j2534_rx_msg_head + 1) % j2534_rx_msg_buffer_actual_size;
    if (next_head == j2534_rx_msg_tail) {
        ESP_LOGW(TAG, "TX indication: RX buffer full, dropping echo");
        return;
    }

    j2534_msg_t *msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
    memset(msg, 0, sizeof(j2534_msg_t));

    // For large messages using ext_data, only echo the CAN ID (first 4 bytes)
    uint8_t *src_data = (orig_msg->ext_data != NULL) ? orig_msg->ext_data : orig_msg->data;
    uint32_t echo_size = orig_msg->data_size;

    // Limit echo size to buffer capacity
    if (echo_size > J2534_MAX_MSG_DATA_SIZE) {
        // For TX echo, only the first frame (CAN ID + up to 8 bytes) matters
        echo_size = 12;  // 4-byte CAN ID + 8 bytes data max
    }

    msg->protocol_id = protocol_id;
    msg->tx_flags = orig_msg->tx_flags;
    // CRITICAL FIX: data_size must reflect actual stored data, not original message size
    // Otherwise ReadMsgs will try to copy garbage memory past end of data[] buffer
    msg->data_size = echo_size;
    msg->extra_data_index = echo_size;
    memcpy(msg->data, src_data, echo_size);

    // Set TX_MSG_TYPE flag to indicate this is a TX echo/confirmation
    msg->rx_status = J2534_TX_MSG_TYPE_MASK;  // 0x01 = TX_MSG_TYPE
    msg->timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    msg->ext_data = NULL;  // Never use ext_data for buffered messages

    j2534_rx_msg_head = next_head;
    ESP_LOGI(TAG, "TX indication buffered: ID=0x%02X%02X%02X%02X, size=%lu (orig=%lu)",
             msg->data[0], msg->data[1], msg->data[2], msg->data[3], 
             msg->data_size, orig_msg->data_size);
}

/* ============================================================================
 * Read Messages
 * ============================================================================ */

j2534_error_t j2534_read_msgs(uint32_t channel_id, j2534_msg_t *msgs,
                              uint32_t *num_msgs, uint32_t timeout)
{
    ESP_LOGI(TAG, "j2534_read_msgs: ch=%lu num=%lu timeout=%lu", 
             channel_id, *num_msgs, timeout);

    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    if (!msgs || !num_msgs) {
        j2534_set_error(J2534_ERR_NULL_PARAMETER, "NULL parameter");
        return J2534_ERR_NULL_PARAMETER;
    }

#if HARDWARE_VER == WICAN_PRO
    // Handle legacy protocol reads via STN monitor buffer
    if (j2534_is_legacy_protocol(ch->protocol_id)) {
        uint32_t requested = *num_msgs;
        uint32_t read_count = 0;
        TickType_t start = xTaskGetTickCount();
        TickType_t timeout_ticks = pdMS_TO_TICKS(timeout);
        
        bool is_j1850 = j2534_is_j1850_protocol(ch->protocol_id);
        bool is_kline = j2534_is_kline_protocol(ch->protocol_id);
        
        ESP_LOGI(TAG, "Legacy ReadMsgs: proto=0x%lX j1850=%d kline=%d requested=%lu timeout=%lu", 
                 ch->protocol_id, is_j1850, is_kline, requested, timeout);
        
        // First check our internal buffer
        while (read_count < requested && j2534_rx_msg_head != j2534_rx_msg_tail) {
            memcpy(&msgs[read_count], &j2534_rx_msg_buffer[j2534_rx_msg_tail],
                   sizeof(j2534_msg_t));
            j2534_rx_msg_tail = (j2534_rx_msg_tail + 1) % j2534_rx_msg_buffer_actual_size;
            read_count++;
        }
        
        // Static buffer for USDT reassembly (J1850 only)
        static uint8_t usdt_complete_msg[4096 + 3];  // Max USDT buffer + header
        
        while (read_count < requested) {
            stn_j2534_msg_t stn_msgs[8];
            uint32_t num_stn_msgs = 0;
            
            stn_j2534_status_t status = stn_j2534_read_msgs(stn_msgs, 8, &num_stn_msgs);
            
            if (num_stn_msgs > 0) {
                ESP_LOGI(TAG, "Legacy ReadMsgs: got %lu from STN monitor", num_stn_msgs);
                
                for (uint32_t i = 0; i < num_stn_msgs && read_count < requested; i++) {
                    
                    // J1850 protocols: Check for USDT multi-frame that needs reassembly
                    if (is_j1850) {
                        #define J1850_HEADER_SIZE_RX 3
                        
                        if (stn_msgs[i].data_len > J1850_HEADER_SIZE_RX) {
                            uint8_t pci = stn_msgs[i].data[J1850_HEADER_SIZE_RX];
                            usdt_frame_type_t frame_type = usdt_get_frame_type(pci);
                            
                            // Check if multi-frame (FF or CF)
                            if (frame_type == USDT_FRAME_FF || frame_type == USDT_FRAME_CF) {
                                uint32_t complete_len = 0;
                                
                                bool complete = usdt_process_rx_frame(
                                    stn_msgs[i].data,
                                    stn_msgs[i].data_len,
                                    usdt_complete_msg,
                                    &complete_len
                                );
                                
                                if (complete && complete_len > 0) {
                                    // Reassembled message ready
                                    msgs[read_count].protocol_id = ch->protocol_id;
                                    msgs[read_count].rx_status = 0;
                                    msgs[read_count].timestamp = stn_msgs[i].timestamp;
                                    msgs[read_count].data_size = complete_len;
                                    
                                    // Cap at buffer size
                                    if (complete_len > sizeof(msgs[read_count].data)) {
                                        complete_len = sizeof(msgs[read_count].data);
                                    }
                                    memcpy(msgs[read_count].data, usdt_complete_msg, complete_len);
                                    msgs[read_count].extra_data_index = complete_len;
                                    
                                    ESP_LOGI(TAG, "J1850 USDT RX complete: len=%lu", complete_len);
                                    read_count++;
                                } else {
                                    // Still accumulating CF frames, or FC sent
                                    ESP_LOGI(TAG, "J1850 USDT RX: frame processed, waiting for more");
                                }
                                continue;  // Don't buffer raw FF/CF frames
                            }
                            
                            // Check for FC frame - just log and skip
                            if (frame_type == USDT_FRAME_FC) {
                                // Process FC to update TX state
                                usdt_process_rx_frame(
                                    stn_msgs[i].data,
                                    stn_msgs[i].data_len,
                                    usdt_complete_msg,
                                    &(uint32_t){0}
                                );
                                ESP_LOGI(TAG, "J1850 FC received and processed");
                                continue;  // Don't buffer FC frames
                            }
                        }
                    }
                    
                    // K-line protocols (ISO 9141/14230): Direct pass-through
                    // The STN chip handles K-line framing natively - no USDT segmentation
                    // Message format: [Format][Target][Source][Length?][Data...][Checksum]
                    // STN chip strips checksum, so we get raw message without CS
                    if (is_kline) {
                        // Log K-line specific message info
                        if (stn_msgs[i].data_len >= 3) {
                            uint8_t fmt = stn_msgs[i].data[0];
                            uint8_t tgt = stn_msgs[i].data[1];
                            uint8_t src = stn_msgs[i].data[2];
                            ESP_LOGI(TAG, "K-line RX: Fmt=%02X Tgt=%02X Src=%02X len=%lu",
                                     fmt, tgt, src, stn_msgs[i].data_len);
                        }
                    }
                    
                    // Buffer the message (single frame for J1850, or all K-line messages)
                    msgs[read_count].protocol_id = ch->protocol_id;
                    msgs[read_count].rx_status = 0;
                    msgs[read_count].timestamp = stn_msgs[i].timestamp;
                    msgs[read_count].data_size = stn_msgs[i].data_len;
                    memcpy(msgs[read_count].data, stn_msgs[i].data, stn_msgs[i].data_len);
                    msgs[read_count].extra_data_index = stn_msgs[i].data_len;
                    
                    ESP_LOGI(TAG, "Legacy RX[%lu]: len=%lu data=%02X%02X%02X%02X...",
                             read_count, msgs[read_count].data_size,
                             msgs[read_count].data[0], msgs[read_count].data[1],
                             msgs[read_count].data[2], 
                             stn_msgs[i].data_len > 3 ? msgs[read_count].data[3] : 0);
                    
                    read_count++;
                }
            }
            
            // For J1850: Check if USDT RX is in progress - extend timeout if needed
            if (is_j1850 && usdt_rx_in_progress()) {
                // Keep polling while multi-frame reception is active
                TickType_t elapsed = xTaskGetTickCount() - start;
                if (elapsed >= timeout_ticks + pdMS_TO_TICKS(2000)) {
                    ESP_LOGW(TAG, "J1850 USDT RX timeout");
                    usdt_reset();
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(5));  // Faster polling during multi-frame
                continue;
            }
            
            // Check timeout
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout_ticks) {
                break;
            }
            
            if (num_stn_msgs == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay before retry
            }
        }
        
        *num_msgs = read_count;
        ESP_LOGI(TAG, "Legacy ReadMsgs: returning %lu messages", read_count);
        
        if (read_count == 0) {
            return J2534_ERR_BUFFER_EMPTY;
        }
        return J2534_STATUS_NOERROR;
    }
#endif

    // CRITICAL FIX: Also switch active channel on READ, not just write!
    if (j2534_active_channel != channel_id) {
        ESP_LOGI(TAG, "j2534_read_msgs: Switching active channel %lu -> %lu (proto=0x%lX)",
                 j2534_active_channel, channel_id, ch->protocol_id);

        // Flush CAN hardware receive queue
        twai_clear_receive_queue();
        vTaskDelay(pdMS_TO_TICKS(5));

        // Switch active channel
        j2534_active_channel = channel_id;

        // Clear stale messages
        j2534_rx_msg_head = 0;
        j2534_rx_msg_tail = 0;

        // Reset ISO-TP state machines (using safe reset to preserve PSRAM buffers)
        memset((void*)&isotp_fc_state, 0, sizeof(isotp_fc_state));
        j2534_reset_isotp_tx_state();
        j2534_reset_isotp_rx_state();
    }

    uint32_t requested = *num_msgs;
    uint32_t read_count = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout);

    // Maximum time to wait for multi-frame message (5 seconds)
    const TickType_t max_isotp_wait_ticks = pdMS_TO_TICKS(5000);
    uint32_t last_log_time = 0;

    while (read_count < requested) {
        // Check for messages in buffer
        if (j2534_rx_msg_head != j2534_rx_msg_tail) {
            memcpy(&msgs[read_count], &j2534_rx_msg_buffer[j2534_rx_msg_tail],
                   sizeof(j2534_msg_t));
            j2534_rx_msg_tail = (j2534_rx_msg_tail + 1) % j2534_rx_msg_buffer_actual_size;

            // Clear awaiting state if we received actual response (not TX echo)
            if ((msgs[read_count].rx_status & 0x01) == 0) {
                if (isotp_tx_state.awaiting_response) {
                    ESP_LOGI(TAG, "j2534_read_msgs: ECU response received");
                    isotp_tx_state.awaiting_response = false;
                }
            }

            read_count++;
        } else {
            // Check timeout
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout_ticks) {
                // Extended wait for ISO-TP multi-frame operations
                
                // Case 1: Multi-frame ISO-TP RX in progress
                if (isotp_rx_state.active && elapsed < max_isotp_wait_ticks) {
                    if ((elapsed - last_log_time) >= pdMS_TO_TICKS(200)) {
                        ESP_LOGI(TAG, "j2534_read_msgs: ISO-TP RX in progress, rx=%lu/%lu",
                                 isotp_rx_state.received_length, 
                                 isotp_rx_state.expected_length);
                        last_log_time = elapsed;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                    continue;
                } else if (isotp_rx_state.active) {
                    ESP_LOGW(TAG, "j2534_read_msgs: ISO-TP RX max wait exceeded");
                    isotp_rx_state.active = false;
                }

                // Case 2: Awaiting ECU response after large TX
                if (isotp_tx_state.awaiting_response) {
                    TickType_t tx_elapsed = xTaskGetTickCount() - isotp_tx_state.tx_complete_time;
                    if (tx_elapsed < max_isotp_wait_ticks) {
                        if ((elapsed - last_log_time) >= pdMS_TO_TICKS(200)) {
                            ESP_LOGI(TAG, "j2534_read_msgs: Awaiting ECU response on 0x%lX",
                                     isotp_tx_state.response_can_id);
                            last_log_time = elapsed;
                        }
                        vTaskDelay(pdMS_TO_TICKS(1));
                        continue;
                    } else {
                        ESP_LOGW(TAG, "j2534_read_msgs: ECU response wait exceeded");
                        isotp_tx_state.awaiting_response = false;
                    }
                }

                break;  // Timeout
            }

            // Log periodically while waiting
            if ((elapsed - last_log_time) >= pdMS_TO_TICKS(100)) {
                last_log_time = elapsed;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    *num_msgs = read_count;

    if (read_count == 0) {
        return J2534_ERR_BUFFER_EMPTY;
    }

    return J2534_STATUS_NOERROR;
}

/* ============================================================================
 * Write Messages
 * ============================================================================ */

j2534_error_t j2534_write_msgs(uint32_t channel_id, j2534_msg_t *msgs,
                               uint32_t *num_msgs, uint32_t timeout)
{
    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    // Switch active channel if needed
    if (j2534_active_channel != channel_id) {
        ESP_LOGI(TAG, "j2534_write_msgs: Switching active channel %lu -> %lu",
                 j2534_active_channel, channel_id);

        twai_clear_receive_queue();
        vTaskDelay(pdMS_TO_TICKS(5));
        j2534_active_channel = channel_id;
        j2534_rx_msg_head = 0;
        j2534_rx_msg_tail = 0;

        // Reset ISO-TP state (using safe reset to preserve PSRAM buffers)
        memset((void*)&isotp_fc_state, 0, sizeof(isotp_fc_state));
        j2534_reset_isotp_tx_state();
        j2534_reset_isotp_rx_state();

        ch->iso15765_ext_addr = false;
        ch->iso15765_ext_addr_byte = 0;
    }

    if (!msgs || !num_msgs) {
        j2534_set_error(J2534_ERR_NULL_PARAMETER, "NULL parameter");
        return J2534_ERR_NULL_PARAMETER;
    }

    uint32_t requested = *num_msgs;
    uint32_t sent_count = 0;

#if HARDWARE_VER == WICAN_PRO
    // Handle legacy protocols through OBD chip
    if (j2534_is_legacy_protocol(ch->protocol_id)) {
        bool is_j1850 = j2534_is_j1850_protocol(ch->protocol_id);
        bool is_kline = j2534_is_kline_protocol(ch->protocol_id);
        
        ESP_LOGI(TAG, "Legacy WriteMsgs: proto=0x%lX j1850=%d kline=%d msgs=%lu timeout=%lu", 
                 ch->protocol_id, is_j1850, is_kline, requested, timeout);
        
        // Initialize USDT if J1850 protocol (not needed for K-line)
        if (is_j1850) {
            usdt_init();
        }
        
        for (uint32_t i = 0; i < requested; i++) {
            stn_j2534_msg_t responses[4];
            uint32_t num_responses = 0;
            
            // Log the TX message
            ESP_LOGI(TAG, "Legacy TX[%lu]: size=%lu data=%02X%02X%02X%02X%02X%02X%02X%02X",
                     i, msgs[i].data_size,
                     msgs[i].data[0], msgs[i].data[1], msgs[i].data[2], msgs[i].data[3],
                     msgs[i].data[4], msgs[i].data[5], msgs[i].data[6], msgs[i].data[7]);

            stn_j2534_status_t status;
            
            // K-line protocols (ISO 9141/14230): Direct pass-through
            // The STN chip handles K-line framing natively - no USDT segmentation needed
            // Message format: [Format][Target][Source][Length?][Data...] - checksum added by STN
            if (is_kline) {
                // K-line message format depends on format byte
                // Bit 7 of format byte: 1 = length in format byte, 0 = separate length byte
                // The STN chip handles this automatically
                
                ESP_LOGI(TAG, "K-line TX: Fmt=%02X Tgt=%02X Src=%02X size=%lu",
                         msgs[i].data[0], msgs[i].data[1], msgs[i].data[2], msgs[i].data_size);
                
                // Set header from message
                if (msgs[i].data_size >= 3) {
                    stn_j2534_set_header(msgs[i].data, 3);
                }
                
                // Send message - STN chip adds checksum
                status = stn_j2534_send_message(
                    msgs[i].data,
                    msgs[i].data_size,
                    responses,
                    4,
                    &num_responses,
                    timeout > 0 ? timeout : 2000  // K-line is slower, longer timeout
                );
                
                ESP_LOGI(TAG, "K-line stn_send_message: status=%d num_responses=%lu", status, num_responses);
                
                if (status == STN_J2534_STATUS_OK || num_responses > 0) {
                    sent_count++;
                    
                    // Buffer responses
                    for (uint32_t r = 0; r < num_responses; r++) {
                        ESP_LOGI(TAG, "K-line RX[%lu]: len=%lu Fmt=%02X Tgt=%02X Src=%02X",
                                 r, responses[r].data_len,
                                 responses[r].data[0], responses[r].data[1], responses[r].data[2]);
                        
                        // K-line: buffer directly (no USDT reassembly needed)
                        uint32_t next_head = (j2534_rx_msg_head + 1) % j2534_rx_msg_buffer_actual_size;
                        if (next_head != j2534_rx_msg_tail) {
                            j2534_msg_t *rx_msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
                            memset(rx_msg, 0, sizeof(j2534_msg_t));
                            rx_msg->protocol_id = ch->protocol_id;
                            rx_msg->rx_status = 0;
                            rx_msg->timestamp = responses[r].timestamp;
                            rx_msg->data_size = responses[r].data_len;
                            memcpy(rx_msg->data, responses[r].data, responses[r].data_len);
                            j2534_rx_msg_head = next_head;
                            ESP_LOGI(TAG, "K-line RX buffered");
                        } else {
                            ESP_LOGW(TAG, "K-line RX buffer full!");
                        }
                    }
                    
                    j2534_buffer_tx_indication(&msgs[i], ch->protocol_id);
                } else {
                    ESP_LOGW(TAG, "K-line TX failed: status=%d", status);
                }
                continue;  // Next message
            }
            
            // J1850 message format: [Header 3 bytes] [Data N bytes]
            // Header: Priority, Target, Source
            // If data portion > 6 bytes, need USDT multi-frame
            #define J1850_HEADER_SIZE 3
            #define J1850_SF_DATA_MAX 6
            
            uint32_t data_len = msgs[i].data_size - J1850_HEADER_SIZE;
            
            if (msgs[i].data_size > J1850_HEADER_SIZE && data_len > J1850_SF_DATA_MAX) {
                // Multi-frame needed - use USDT
                ESP_LOGI(TAG, "J1850 USDT TX: header=%02X%02X%02X data_len=%lu",
                         msgs[i].data[0], msgs[i].data[1], msgs[i].data[2], data_len);
                
                status = usdt_send_message(
                    msgs[i].data,                    // First 3 bytes are header
                    msgs[i].data + J1850_HEADER_SIZE, // Data starts after header
                    data_len,
                    timeout > 0 ? timeout : 2000     // Longer timeout for multi-frame
                );
                
                if (status == STN_J2534_STATUS_OK) {
                    sent_count++;
                    j2534_buffer_tx_indication(&msgs[i], ch->protocol_id);
                    ESP_LOGI(TAG, "J1850 USDT TX complete");
                } else {
                    ESP_LOGE(TAG, "J1850 USDT TX failed: status=%d", status);
                }
                
                // Note: Multi-frame responses will come via separate ReadMsgs calls
                // The USDT RX state machine handles reassembly
                
            } else {
                // Single frame - use existing path
                status = stn_j2534_send_message(
                    msgs[i].data,
                    msgs[i].data_size,
                    responses,
                    4,
                    &num_responses,
                    timeout > 0 ? timeout : 1000
                );
                
                ESP_LOGI(TAG, "J1850 stn_send_message: status=%d num_responses=%lu", status, num_responses);

                if (status == STN_J2534_STATUS_OK || num_responses > 0) {
                    sent_count++;

                    // Buffer responses as RX messages (may include USDT frames)
                    for (uint32_t r = 0; r < num_responses; r++) {
                        ESP_LOGI(TAG, "J1850 RX[%lu]: len=%lu data=%02X%02X%02X%02X%02X%02X%02X%02X",
                                 r, responses[r].data_len,
                                 responses[r].data[0], responses[r].data[1], 
                                 responses[r].data[2], responses[r].data[3],
                                 responses[r].data[4], responses[r].data[5],
                                 responses[r].data[6], responses[r].data[7]);
                        
                        // Check if this is a USDT frame that needs reassembly
                        if (responses[r].data_len > J1850_HEADER_SIZE) {
                            uint8_t pci = responses[r].data[J1850_HEADER_SIZE];
                            usdt_frame_type_t frame_type = usdt_get_frame_type(pci);
                            
                            if (frame_type == USDT_FRAME_FF || frame_type == USDT_FRAME_CF) {
                                // Multi-frame response - process via USDT
                                static uint8_t complete_msg[4096];
                                uint32_t complete_len = 0;
                                
                                bool complete = usdt_process_rx_frame(
                                    responses[r].data,
                                    responses[r].data_len,
                                    complete_msg,
                                    &complete_len
                                );
                                
                                if (complete && complete_len > 0) {
                                    // Reassembled message ready - buffer it
                                    uint32_t next_head = (j2534_rx_msg_head + 1) % j2534_rx_msg_buffer_actual_size;
                                    if (next_head != j2534_rx_msg_tail) {
                                        j2534_msg_t *rx_msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
                                        memset(rx_msg, 0, sizeof(j2534_msg_t));
                                        rx_msg->protocol_id = ch->protocol_id;
                                        rx_msg->rx_status = 0;
                                        rx_msg->timestamp = responses[r].timestamp;
                                        rx_msg->data_size = complete_len;
                                        memcpy(rx_msg->data, complete_msg, complete_len);
                                        j2534_rx_msg_head = next_head;
                                        ESP_LOGI(TAG, "J1850 USDT RX complete: len=%lu", complete_len);
                                    }
                                }
                                continue;  // Don't buffer raw CF/FF frames
                            }
                        }
                        
                        // Single frame or other - buffer directly
                        uint32_t next_head = (j2534_rx_msg_head + 1) % j2534_rx_msg_buffer_actual_size;
                        if (next_head != j2534_rx_msg_tail) {
                            j2534_msg_t *rx_msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
                            memset(rx_msg, 0, sizeof(j2534_msg_t));
                            rx_msg->protocol_id = ch->protocol_id;
                            rx_msg->rx_status = 0;
                            rx_msg->timestamp = responses[r].timestamp;
                            rx_msg->data_size = responses[r].data_len;
                            memcpy(rx_msg->data, responses[r].data, responses[r].data_len);
                            j2534_rx_msg_head = next_head;
                            ESP_LOGI(TAG, "J1850 RX buffered: head=%lu tail=%lu", j2534_rx_msg_head, j2534_rx_msg_tail);
                        } else {
                            ESP_LOGW(TAG, "J1850 RX buffer full!");
                        }
                    }

                    j2534_buffer_tx_indication(&msgs[i], ch->protocol_id);
                } else {
                    ESP_LOGW(TAG, "J1850 TX failed: status=%d", status);
                }
            }
        }

        ESP_LOGI(TAG, "Legacy WriteMsgs done: sent=%lu", sent_count);
        *num_msgs = sent_count;
        return J2534_STATUS_NOERROR;
    }
#endif

    // Clear stale ISO-TP state at start of write
    if (isotp_tx_state.awaiting_response) {
        isotp_tx_state.awaiting_response = false;
    }
    if (isotp_rx_state.active) {
        isotp_rx_state.active = false;
    }
    if (isotp_fc_state.waiting_for_fc) {
        isotp_fc_state.waiting_for_fc = false;
        isotp_fc_state.fc_received = false;
    }

    // Process each message
    for (uint32_t i = 0; i < requested; i++) {
        twai_message_t can_frame;
        memset(&can_frame, 0, sizeof(can_frame));

        if (msgs[i].data_size < 4) {
            continue;
        }

        // Use ext_data pointer if available (for large transfers)
        uint8_t *msg_data_ptr = (msgs[i].ext_data != NULL) ? msgs[i].ext_data : msgs[i].data;

        can_frame.identifier = (msg_data_ptr[0] << 24) |
                               (msg_data_ptr[1] << 16) |
                               (msg_data_ptr[2] << 8) |
                               msg_data_ptr[3];

        // Track last TX ID for functional addressing
        j2534_last_tx_id = can_frame.identifier;
        
        // Auto-populate functional lookup table for OBD-II functional addressing
        // When sending to 0x7DF (11-bit) or 0x18DB33F1 (29-bit), auto-add expected response IDs
        if (ch->funct_msg_count == 0) {
            bool is_functional_request = false;
            
            // Check for standard OBD-II functional addresses
            if (can_frame.identifier == 0x7DF && !can_frame.extd) {
                is_functional_request = true;
            } else if (can_frame.identifier == 0x18DB33F1 && can_frame.extd) {
                is_functional_request = true;
            }
            
            if (is_functional_request) {
                ESP_LOGI(TAG, "Functional request detected (0x%lX), auto-populating response IDs",
                         can_frame.identifier);
                
                // Add standard OBD-II response IDs (0x7E8-0x7EF for ECU #1-8)
                for (uint32_t ecuid = 0x7E8; ecuid <= 0x7EF; ecuid++) {
                    bool found = false;
                    for (uint32_t j = 0; j < J2534_MAX_FUNCT_MSG_IDS; j++) {
                        if (ch->funct_msg_table[j].active && ch->funct_msg_table[j].can_id == ecuid) {
                            found = true;
                            break;
                        }
                    }
                    if (!found && ch->funct_msg_count < J2534_MAX_FUNCT_MSG_IDS) {
                        for (uint32_t j = 0; j < J2534_MAX_FUNCT_MSG_IDS; j++) {
                            if (!ch->funct_msg_table[j].active) {
                                ch->funct_msg_table[j].can_id = ecuid;
                                ch->funct_msg_table[j].active = true;
                                ch->funct_msg_count++;
                                break;
                            }
                        }
                    }
                }
                ESP_LOGI(TAG, "Auto-added %lu functional response IDs", ch->funct_msg_count);
            }
        }

        // Handle extended ID
        if (msgs[i].tx_flags & J2534_CAN_29BIT_ID) {
            can_frame.extd = 1;
        } else {
            can_frame.extd = 0;
            can_frame.identifier &= 0x7FF;
        }

        // Get payload (skip CAN ID bytes)
        uint32_t payload_len = msgs[i].data_size - 4;
        uint8_t *payload = &msg_data_ptr[4];

        // Check if ISO15765 protocol
        bool is_iso15765 = (ch->protocol_id == J2534_PROTOCOL_ISO15765 ||
                           ch->protocol_id == J2534_PROTOCOL_ISO15765_PS ||
                           ch->protocol_id == J2534_PROTOCOL_SW_ISO15765_PS);

        if (is_iso15765 && payload_len > 0) {
            bool use_padding = (msgs[i].tx_flags & J2534_ISO15765_PADDING) != 0;

            // Check for pre-formatted ISO-TP frames
            // IMPORTANT: Don't persist extended addressing detection from pre-formatted
            // frames as this can cause false positives on subsequent normal frames
            bool is_preformatted = false;
            if (payload_len == 8) {
                uint8_t first_pci = (payload[0] >> 4) & 0x0F;
                if (first_pci <= 0x3) {
                    // Standard ISO-TP pre-formatted frame
                    is_preformatted = true;
                } else if (first_pci >= 0x4 && ((payload[1] >> 4) & 0x0F) <= 0x3) {
                    // Looks like extended addressing pre-formatted frame
                    // Don't set ch->iso15765_ext_addr to avoid sticky false positives
                    is_preformatted = true;
                    ESP_LOGI(TAG, "TX: Pre-formatted frame with extended addr 0x%02X", payload[0]);
                }
            }

            if (is_preformatted) {
                // Send pre-formatted frame as-is
                can_frame.data_length_code = 8;
                memcpy(can_frame.data, payload, 8);
            } else if (payload_len <= 7) {
                // Single Frame
                can_frame.data[0] = (uint8_t)payload_len;
                memcpy(&can_frame.data[1], payload, payload_len);
                can_frame.data_length_code = 1 + payload_len;

                if (use_padding && can_frame.data_length_code < 8) {
                    memset(&can_frame.data[can_frame.data_length_code], 0xAA,
                           8 - can_frame.data_length_code);
                    can_frame.data_length_code = 8;
                }
            } else {
                // Multi-frame: Use j2534_send_isotp_message helper
                j2534_error_t isotp_result = j2534_send_isotp_message(
                    ch, can_frame.identifier, payload, payload_len,
                    can_frame.extd, use_padding);

                if (isotp_result == J2534_STATUS_NOERROR) {
                    j2534_buffer_tx_indication(&msgs[i], ch->protocol_id);

                    // Mark awaiting response
                    isotp_tx_state.awaiting_response = true;
                    isotp_tx_state.tx_complete_time = xTaskGetTickCount();
                    isotp_tx_state.response_can_id = can_frame.identifier + 8;

                    sent_count++;
                }
                continue;
            }
        } else {
            // Raw CAN
            if (payload_len > 8) payload_len = 8;
            can_frame.data_length_code = payload_len;
            memcpy(can_frame.data, payload, payload_len);
        }

        // Send frame
        can_frame.self = 0;
        uint32_t send_timeout = (timeout > 0) ? timeout : 100;
        esp_err_t send_result = can_send(&can_frame, pdMS_TO_TICKS(send_timeout));

        if (send_result == ESP_OK) {
            sent_count++;
            j2534_buffer_tx_indication(&msgs[i], ch->protocol_id);
        } else {
            ESP_LOGE(TAG, "CAN send FAILED: err=%d", send_result);
        }
    }

    *num_msgs = sent_count;
    return J2534_STATUS_NOERROR;
}
