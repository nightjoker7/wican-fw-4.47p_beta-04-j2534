/**
 * @file j2534_isotp.c
 * @brief J2534 ISO-TP (ISO 15765-2) frame handling
 *
 * This file contains ISO-TP transport layer functions for segmented
 * message handling, including:
 * - Single Frame (SF) processing
 * - First Frame (FF) and Flow Control (FC) handling
 * - Consecutive Frame (CF) reassembly
 * - Extended addressing detection
 */

#include "j2534.h"
#include "j2534_internal.h"
#include "can.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "j2534_isotp";

/* ===========================================================================
 * ISO-TP Helper Functions
 * =========================================================================== */

uint32_t isotp_get_flow_control_id(j2534_channel_t *ch, uint32_t rx_id)
{
    // Look for a flow control filter that matches this RX ID
    for (uint32_t i = 0; i < J2534_MAX_FILTERS; i++) {
        if (!ch->filters[i].active) continue;
        if (ch->filters[i].filter_type != J2534_FILTER_FLOW_CONTROL) continue;

        // Check if pattern matches this RX ID
        uint32_t pattern = (ch->filters[i].pattern[0] << 24) |
                          (ch->filters[i].pattern[1] << 16) |
                          (ch->filters[i].pattern[2] << 8) |
                          ch->filters[i].pattern[3];

        uint32_t mask = (ch->filters[i].mask[0] << 24) |
                       (ch->filters[i].mask[1] << 16) |
                       (ch->filters[i].mask[2] << 8) |
                       ch->filters[i].mask[3];

        if ((rx_id & mask) == (pattern & mask)) {
            // Return the flow control ID
            uint32_t fc_id = (ch->filters[i].flow_control[0] << 24) |
                            (ch->filters[i].flow_control[1] << 16) |
                            (ch->filters[i].flow_control[2] << 8) |
                            ch->filters[i].flow_control[3];
            return fc_id;
        }
    }

    // Default: standard OBD-II response ID mapping
    // RX 0x7E8-0x7EF -> TX 0x7E0-0x7E7
    if (rx_id >= 0x7E8 && rx_id <= 0x7EF) {
        return rx_id - 8;
    }

    return rx_id - 8;  // Default assumption
}

void isotp_send_flow_control(j2534_channel_t *ch, uint32_t tx_id, bool is_extended)
{
    twai_message_t fc_frame;
    memset(&fc_frame, 0, sizeof(fc_frame));

    fc_frame.identifier = tx_id;
    fc_frame.extd = is_extended ? 1 : 0;
    fc_frame.data_length_code = 8;

    // Flow Control: PCI=0x30, FS=0 (CTS), BS=0 (no limit), STmin
    fc_frame.data[0] = 0x30;  // FC frame with CTS (Continue To Send)
    fc_frame.data[1] = ch->iso15765_bs;  // Block Size (0 = no limit)
    fc_frame.data[2] = ch->iso15765_stmin;  // STmin (default 10ms)
    // Padding
    fc_frame.data[3] = 0xAA;
    fc_frame.data[4] = 0xAA;
    fc_frame.data[5] = 0xAA;
    fc_frame.data[6] = 0xAA;
    fc_frame.data[7] = 0xAA;

    esp_err_t result = can_send(&fc_frame, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "ISO-TP: Sent FC to 0x%lX, result=%d", tx_id, result);
}

void isotp_buffer_complete_message(j2534_channel_t *ch, uint32_t can_id,
                                          uint8_t *data, uint32_t data_len, bool is_extended)
{
    uint32_t next_head = (j2534_rx_msg_head + 1) % J2534_RX_MSG_BUFFER_SIZE;
    if (next_head == j2534_rx_msg_tail) {
        ESP_LOGW(TAG, "ISO-TP: RX buffer full!");
        return;
    }

    j2534_msg_t *msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
    memset(msg, 0, sizeof(j2534_msg_t));

    msg->protocol_id = ch->protocol_id;
    msg->timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    msg->rx_status = is_extended ? J2534_CAN_29BIT_ID : 0;

    // Put CAN ID in first 4 bytes
    msg->data[0] = (can_id >> 24) & 0xFF;
    msg->data[1] = (can_id >> 16) & 0xFF;
    msg->data[2] = (can_id >> 8) & 0xFF;
    msg->data[3] = can_id & 0xFF;

    // Copy reassembled data (without ISO-TP headers)
    uint32_t copy_len = data_len;
    if (copy_len > J2534_MAX_MSG_DATA_SIZE - 4) {
        ESP_LOGW(TAG, "ISO-TP: Message truncated from %lu to %d bytes",
                 data_len, J2534_MAX_MSG_DATA_SIZE - 4);
        copy_len = J2534_MAX_MSG_DATA_SIZE - 4;
    }
    memcpy(&msg->data[4], data, copy_len);
    msg->data_size = 4 + copy_len;

    j2534_rx_msg_head = next_head;
    ESP_LOGI(TAG, "ISO-TP: Complete message buffered, ID=0x%lX, len=%lu", can_id, copy_len);
}

/* ===========================================================================
 * CAN Frame to J2534 Message Conversion
 * =========================================================================== */

int32_t j2534_can_to_msg(twai_message_t *frame, uint8_t *output_buf)
{
    if (!frame || !output_buf || !j2534_is_active()) {
        return 0;
    }

    j2534_channel_t *ch = j2534_get_channel(j2534_active_channel);
    if (!ch) {
        return 0;
    }

    // Check if ISO15765 protocol - need ISO-TP handling
    bool is_iso15765 = (ch->protocol_id == J2534_PROTOCOL_ISO15765 ||
                       ch->protocol_id == J2534_PROTOCOL_ISO15765_PS ||
                       ch->protocol_id == J2534_PROTOCOL_SW_ISO15765_PS);

    // Check if raw CAN protocol
    bool is_raw_can = (ch->protocol_id == J2534_PROTOCOL_CAN ||
                       ch->protocol_id == J2534_PROTOCOL_CAN_PS ||
                       ch->protocol_id == J2534_PROTOCOL_SW_CAN_PS);

    // Check filters - start with checking if frame passes any filter
    bool has_pass_filter = false;
    for (uint32_t i = 0; i < J2534_MAX_FILTERS; i++) {
        if (ch->filters[i].active && ch->filters[i].filter_type != J2534_FILTER_BLOCK) {
            has_pass_filter = true;
            break;
        }
    }

    // Default: pass if no filters at all, or for ISO15765 if no PASS/FLOW_CONTROL filters
    bool pass = (ch->filter_count == 0 && ch->funct_msg_count == 0);

    // Special handling for functional addressing
    bool functional_mode = (j2534_last_tx_id == 0x101 || j2534_last_tx_id == 0x10B ||
                           j2534_last_tx_id == 0x7DF ||
                           (j2534_last_tx_id >= 0x100 && j2534_last_tx_id <= 0x10F));

    bool permissive_mode = false;

    ESP_LOGI(TAG, "j2534_can_to_msg: RX frame ID=0x%lX, DLC=%d, filter_count=%lu, funct_count=%lu, iso15765=%d, raw_can=%d, has_pass=%d, last_tx=0x%lX, func_mode=%d",
             frame->identifier, frame->data_length_code, ch->filter_count, ch->funct_msg_count, is_iso15765, is_raw_can, has_pass_filter, j2534_last_tx_id, functional_mode);

    // Check PASS/FLOW_CONTROL filters
    for (uint32_t i = 0; i < J2534_MAX_FILTERS && !pass; i++) {
        if (!ch->filters[i].active) continue;
        if (ch->filters[i].filter_type == J2534_FILTER_BLOCK) continue;

        uint32_t masked_id = frame->identifier;
        uint32_t mask = (ch->filters[i].mask[0] << 24) |
                        (ch->filters[i].mask[1] << 16) |
                        (ch->filters[i].mask[2] << 8) |
                        ch->filters[i].mask[3];
        uint32_t pattern = (ch->filters[i].pattern[0] << 24) |
                           (ch->filters[i].pattern[1] << 16) |
                           (ch->filters[i].pattern[2] << 8) |
                           ch->filters[i].pattern[3];

        ESP_LOGI(TAG, "  filter[%lu] type=%lu: ID=0x%lX & mask=0x%lX = 0x%lX, pattern=0x%lX, match=%d",
                 i, ch->filters[i].filter_type, masked_id, mask, (masked_id & mask), (pattern & mask),
                 ((masked_id & mask) == (pattern & mask)));

        if ((masked_id & mask) == (pattern & mask)) {
            pass = true;
        }

        // For FLOW_CONTROL filters, also check if this could be a response from functional addressing
        if (!pass && ch->filters[i].filter_type == J2534_FILTER_FLOW_CONTROL && is_iso15765) {
            uint32_t fc_id = (ch->filters[i].flow_control[0] << 24) |
                            (ch->filters[i].flow_control[1] << 16) |
                            (ch->filters[i].flow_control[2] << 8) |
                            ch->filters[i].flow_control[3];

            bool is_functional_request = (fc_id == 0x101 || fc_id == 0x10B || fc_id == 0x7DF ||
                                          (fc_id >= 0x100 && fc_id <= 0x1FF));
            bool is_likely_response = (frame->identifier >= 0x100 && frame->identifier <= 0x7FF);

            if (is_functional_request && is_likely_response) {
                ESP_LOGI(TAG, "  FLOW_CONTROL functional response: fc=0x%lX rx=0x%lX - PASS", fc_id, frame->identifier);
                pass = true;
                permissive_mode = true;
            }
        }
    }

    // For ISO15765 with functional addressing:
    // CRITICAL: Only use permissive mode if NO explicit PASS filters are configured!
    if (!pass && is_iso15765 && functional_mode && !has_pass_filter) {
        bool is_likely_ecu_response = (frame->identifier >= 0x100 && frame->identifier <= 0x7FF);
        if (is_likely_ecu_response) {
            ESP_LOGI(TAG, "  ISO15765 functional mode: accepting ID=0x%lX (last_tx=0x%lX)",
                     frame->identifier, j2534_last_tx_id);
            pass = true;
            permissive_mode = true;
        }
    }

    // For ISO15765 with PHYSICAL addressing:
    // CRITICAL: Only use this fallback if NO explicit PASS filters are configured!
    if (!pass && is_iso15765 && j2534_last_tx_id != 0 && !has_pass_filter) {
        uint32_t expected_rx_id = j2534_last_tx_id + 8;

        if (frame->identifier == expected_rx_id) {
            ESP_LOGI(TAG, "  ISO15765 physical addressing: accepting ID=0x%lX (last_tx=0x%lX + 8)",
                     frame->identifier, j2534_last_tx_id);
            pass = true;
        }
        else if (j2534_last_tx_id >= 8 && frame->identifier == j2534_last_tx_id - 8) {
            ESP_LOGI(TAG, "  ISO15765 physical addressing: accepting ID=0x%lX (last_tx=0x%lX - 8)",
                     frame->identifier, j2534_last_tx_id);
            pass = true;
        }
    }

    // Check functional message lookup table
    for (uint32_t i = 0; i < J2534_MAX_FUNCT_MSG_IDS && !pass; i++) {
        if (!ch->funct_msg_table[i].active) continue;

        if (frame->identifier == ch->funct_msg_table[i].can_id) {
            ESP_LOGI(TAG, "  FUNCT match: ID=0x%lX matches functional entry", frame->identifier);
            pass = true;
        }
    }

    // Check block filters
    for (uint32_t i = 0; i < J2534_MAX_FILTERS && pass; i++) {
        if (!ch->filters[i].active) continue;
        if (ch->filters[i].filter_type != J2534_FILTER_BLOCK) continue;

        uint32_t masked_id = frame->identifier;
        uint32_t mask = (ch->filters[i].mask[0] << 24) |
                        (ch->filters[i].mask[1] << 16) |
                        (ch->filters[i].mask[2] << 8) |
                        ch->filters[i].mask[3];
        uint32_t pattern = (ch->filters[i].pattern[0] << 24) |
                           (ch->filters[i].pattern[1] << 16) |
                           (ch->filters[i].pattern[2] << 8) |
                           ch->filters[i].pattern[3];

        if ((masked_id & mask) == (pattern & mask)) {
            pass = false;
        }
    }

    if (!pass) {
        ESP_LOGI(TAG, "j2534_can_to_msg: FILTERED OUT frame ID=0x%lX (permissive=%d)",
                 frame->identifier, permissive_mode);
        return 0;
    }

    // For raw CAN: skip ISO-TP processing and go directly to buffering
    if (is_raw_can) {
        ESP_LOGI(TAG, "j2534_can_to_msg: RAW CAN passed filters, buffering ID=0x%lX", frame->identifier);
        goto buffer_message;
    }

    // Handle ISO-TP framing if ISO15765 protocol
    if (is_iso15765 && frame->data_length_code > 0) {
        // Check for extended addressing mode
        uint8_t pci_offset = 0;
        uint8_t ext_addr_byte = 0;

        if (ch->iso15765_ext_addr) {
            pci_offset = 1;
            ext_addr_byte = frame->data[0];
            ESP_LOGI(TAG, "ISO-TP: Using extended addressing (configured), ext_addr=0x%02X", ext_addr_byte);
        } else if (frame->data_length_code >= 2) {
            // Auto-detect extended addressing
            uint8_t first_byte = frame->data[0];
            uint8_t second_byte = frame->data[1];
            uint8_t first_pci_type = (first_byte >> 4) & 0x0F;
            uint8_t second_pci_type = (second_byte >> 4) & 0x0F;

            if (first_pci_type >= 0x4 && second_pci_type <= 0x3) {
                pci_offset = 1;
                ext_addr_byte = first_byte;
                ch->iso15765_ext_addr = true;
                ch->iso15765_ext_addr_byte = ext_addr_byte;
                ESP_LOGI(TAG, "ISO-TP: Detected extended addressing, ext_addr=0x%02X", ext_addr_byte);
            }
        }

        uint8_t pci_type = (frame->data[pci_offset] >> 4) & 0x0F;

        switch (pci_type) {
            case 0x0:  // Single Frame (SF)
            {
                uint8_t sf_dl = frame->data[pci_offset] & 0x0F;
                if (sf_dl > 0 && sf_dl <= (7 - pci_offset)) {
                    ESP_LOGI(TAG, "ISO-TP: SF received, len=%d, pci_offset=%d", sf_dl, pci_offset);

                    if (pci_offset > 0) {
                        uint8_t temp_data[8];
                        temp_data[0] = ext_addr_byte;
                        memcpy(&temp_data[1], &frame->data[pci_offset + 1], sf_dl);
                        isotp_buffer_complete_message(ch, frame->identifier,
                                                      temp_data, sf_dl + 1, frame->extd);
                    } else {
                        isotp_buffer_complete_message(ch, frame->identifier,
                                                      &frame->data[1], sf_dl, frame->extd);
                    }
                }
                return 0;
            }

            case 0x1:  // First Frame (FF)
            {
                uint16_t ff_dl = ((frame->data[pci_offset] & 0x0F) << 8) | frame->data[pci_offset + 1];
                ESP_LOGI(TAG, "ISO-TP: FF received, total_len=%d, pci_offset=%d", ff_dl, pci_offset);

                if (isotp_rx_state.active) {
                    ESP_LOGW(TAG, "ISO-TP: New FF received while multi-frame in progress, aborting previous");
                }

                uint32_t fc_tx_id = isotp_get_flow_control_id(ch, frame->identifier);

                isotp_rx_state.active = true;
                isotp_rx_state.can_id = frame->identifier;
                isotp_rx_state.expected_length = ff_dl + pci_offset;
                isotp_rx_state.received_length = 0;
                isotp_rx_state.next_seq_num = 1;
                isotp_rx_state.is_extended = frame->extd;
                isotp_rx_state.last_frame_time = xTaskGetTickCount();
                isotp_rx_state.flow_control_id = fc_tx_id;
                isotp_rx_state.block_size = ch->iso15765_bs;
                isotp_rx_state.block_count = 0;

                uint32_t copy_start = 0;
                if (pci_offset > 0) {
                    isotp_rx_state.data[0] = ext_addr_byte;
                    copy_start = 1;
                    isotp_rx_state.received_length = 1;
                }
                uint32_t ff_data_bytes = 6 - pci_offset;
                memcpy((uint8_t*)&isotp_rx_state.data[copy_start], &frame->data[pci_offset + 2], ff_data_bytes);
                isotp_rx_state.received_length += ff_data_bytes;

                isotp_send_flow_control(ch, fc_tx_id, frame->extd);

                return 0;
            }

            case 0x2:  // Consecutive Frame (CF)
            {
                if (!isotp_rx_state.active) {
                    ESP_LOGW(TAG, "ISO-TP: CF without active FF!");
                    return 0;
                }

                TickType_t elapsed = xTaskGetTickCount() - isotp_rx_state.last_frame_time;
                if (elapsed > pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "ISO-TP: CF timeout (N_Cr), elapsed=%lu ms, aborting multi-frame RX",
                             elapsed * portTICK_PERIOD_MS);
                    isotp_rx_state.active = false;
                    return 0;
                }

                uint8_t seq_num = frame->data[pci_offset] & 0x0F;
                if (seq_num != isotp_rx_state.next_seq_num) {
                    ESP_LOGW(TAG, "ISO-TP: Seq mismatch! Expected %d, got %d",
                             isotp_rx_state.next_seq_num, seq_num);
                    isotp_rx_state.active = false;
                    return 0;
                }

                uint32_t remaining = isotp_rx_state.expected_length - isotp_rx_state.received_length;
                uint32_t cf_data_len = (remaining > (7 - pci_offset)) ? (7 - pci_offset) : remaining;

                if (isotp_rx_state.received_length + cf_data_len <= sizeof(isotp_rx_state.data)) {
                    memcpy((uint8_t*)&isotp_rx_state.data[isotp_rx_state.received_length],
                           &frame->data[pci_offset + 1], cf_data_len);
                    isotp_rx_state.received_length += cf_data_len;
                }

                isotp_rx_state.next_seq_num = (isotp_rx_state.next_seq_num + 1) & 0x0F;
                isotp_rx_state.last_frame_time = xTaskGetTickCount();
                isotp_rx_state.block_count++;

                ESP_LOGI(TAG, "ISO-TP: CF[%d], %lu/%lu bytes, block=%d/%d",
                         seq_num, isotp_rx_state.received_length, isotp_rx_state.expected_length,
                         isotp_rx_state.block_count, isotp_rx_state.block_size);

                if (isotp_rx_state.received_length >= isotp_rx_state.expected_length) {
                    isotp_buffer_complete_message(ch, isotp_rx_state.can_id,
                                                  (uint8_t*)isotp_rx_state.data,
                                                  isotp_rx_state.expected_length,
                                                  isotp_rx_state.is_extended);
                    isotp_rx_state.active = false;
                }
                else if (isotp_rx_state.block_size > 0 &&
                         isotp_rx_state.block_count >= isotp_rx_state.block_size) {
                    ESP_LOGI(TAG, "ISO-TP: Block size %d reached, sending FC", isotp_rx_state.block_size);
                    isotp_send_flow_control(ch, isotp_rx_state.flow_control_id, isotp_rx_state.is_extended);
                    isotp_rx_state.block_count = 0;
                }

                return 0;
            }

            case 0x3:  // Flow Control (FC)
            {
                uint8_t fc_fs = frame->data[pci_offset] & 0x0F;
                uint8_t fc_bs = frame->data[pci_offset + 1];
                uint8_t fc_stmin = frame->data[pci_offset + 2];

                ESP_LOGI(TAG, "ISO-TP: FC received, FS=%d, BS=%d, STmin=%d", fc_fs, fc_bs, fc_stmin);

                if (isotp_fc_state.waiting_for_fc) {
                    isotp_fc_state.fc_flow_status = fc_fs;
                    isotp_fc_state.fc_block_size = fc_bs;
                    isotp_fc_state.fc_stmin = fc_stmin;
                    isotp_fc_state.fc_received = true;
                    ESP_LOGI(TAG, "ISO-TP: FC signaled to TX task");
                }

                return 0;
            }

            default:
                ESP_LOGW(TAG, "ISO-TP: Unknown PCI type 0x%X, buffering as raw", pci_type);
                break;
        }
    }

buffer_message:
    // Non-ISO-TP or raw CAN - buffer as-is
    ESP_LOGI(TAG, "j2534_can_to_msg: BUFFERED frame ID=0x%lX", frame->identifier);

    uint32_t next_head = (j2534_rx_msg_head + 1) % J2534_RX_MSG_BUFFER_SIZE;
    if (next_head == j2534_rx_msg_tail) {
        ESP_LOGW(TAG, "j2534_can_to_msg: RX buffer full!");
        return 0;
    }

    j2534_msg_t *msg = &j2534_rx_msg_buffer[j2534_rx_msg_head];
    memset(msg, 0, sizeof(j2534_msg_t));

    msg->protocol_id = ch->protocol_id;
    msg->timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    msg->rx_status = 0;

    if (frame->extd) {
        msg->rx_status |= J2534_CAN_29BIT_ID;
    }

    msg->data[0] = (frame->identifier >> 24) & 0xFF;
    msg->data[1] = (frame->identifier >> 16) & 0xFF;
    msg->data[2] = (frame->identifier >> 8) & 0xFF;
    msg->data[3] = frame->identifier & 0xFF;

    memcpy(&msg->data[4], frame->data, frame->data_length_code);
    msg->data_size = 4 + frame->data_length_code;

    j2534_rx_msg_head = next_head;

    return 0;
}

/**
 * @brief Send ISO-TP multi-frame message
 * 
 * Handles First Frame (FF) transmission, Flow Control (FC) reception,
 * and Consecutive Frame (CF) transmission with proper timing.
 * 
 * @param ch Channel handle
 * @param can_id CAN identifier
 * @param data Pointer to data buffer
 * @param data_len Length of data
 * @param is_extended True for 29-bit CAN ID
 * @param padding True to pad frames to 8 bytes
 * @return J2534 error code
 */
j2534_error_t j2534_send_isotp_message(j2534_channel_t *ch, uint32_t can_id,
                                        uint8_t *data, uint32_t data_len,
                                        bool is_extended, bool padding)
{
    twai_message_t can_frame = {0};
    can_frame.identifier = can_id;
    can_frame.extd = is_extended ? 1 : 0;
    can_frame.self = 0;
    
    uint32_t timeout = 1000;  // Default timeout 1 second
    
    // First Frame: PCI 0x1X XX where XXX = total length
    can_frame.data[0] = 0x10 | ((data_len >> 8) & 0x0F);
    can_frame.data[1] = data_len & 0xFF;
    memcpy(&can_frame.data[2], data, 6);  // First 6 bytes
    can_frame.data_length_code = 8;
    
    ESP_LOGI(TAG, "ISO15765 FF: ID=0x%lX, total_len=%lu", can_id, data_len);
    
    // Setup FC wait state BEFORE sending FF
    isotp_fc_state.waiting_for_fc = true;
    isotp_fc_state.fc_received = false;
    isotp_fc_state.fc_rx_id = can_id;
    
    // Send First Frame
    uint32_t ff_timeout = (timeout > 0) ? timeout : 100;
    esp_err_t send_result = can_send(&can_frame, pdMS_TO_TICKS(ff_timeout));
    if (send_result != ESP_OK) {
        ESP_LOGE(TAG, "ISO15765 FF send FAILED: err=%d", send_result);
        isotp_fc_state.waiting_for_fc = false;
        return J2534_ERR_FAILED;
    }
    
    // Wait for Flow Control from ECU
    TickType_t fc_start = xTaskGetTickCount();
    TickType_t fc_timeout = pdMS_TO_TICKS(timeout > 0 ? timeout : 1000);
    uint8_t fc_stmin = ch->iso15765_stmin;
    uint8_t fc_bs = 0;
    
    ESP_LOGI(TAG, "ISO15765: Waiting for FC from ECU...");
    
    while (!isotp_fc_state.fc_received) {
        TickType_t elapsed = xTaskGetTickCount() - fc_start;
        if (elapsed >= fc_timeout) {
            ESP_LOGW(TAG, "ISO15765: FC timeout! Sending CFs with default timing");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (isotp_fc_state.fc_received) {
        // Handle FC Wait status (FS=1)
        while (isotp_fc_state.fc_flow_status == 1) {
            ESP_LOGI(TAG, "ISO15765: FC Wait received, waiting for CTS...");
            isotp_fc_state.fc_received = false;
            
            TickType_t wait_start = xTaskGetTickCount();
            while (!isotp_fc_state.fc_received) {
                if ((xTaskGetTickCount() - wait_start) >= fc_timeout) {
                    ESP_LOGW(TAG, "ISO15765: FC CTS timeout after Wait");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            if (!isotp_fc_state.fc_received) break;
        }
        
        if (isotp_fc_state.fc_flow_status == 2) {
            ESP_LOGE(TAG, "ISO15765: FC Overflow received, aborting");
            isotp_fc_state.waiting_for_fc = false;
            return J2534_ERR_BUFFER_OVERFLOW;
        }
        
        fc_stmin = isotp_fc_state.fc_stmin;
        fc_bs = isotp_fc_state.fc_block_size;
        ESP_LOGI(TAG, "ISO15765: FC received, BS=%d STmin=%d", fc_bs, fc_stmin);
    }
    
    isotp_fc_state.waiting_for_fc = false;
    
    // Send Consecutive Frames
    uint32_t remaining = data_len - 6;
    uint32_t offset = 6;
    uint8_t seq_num = 1;
    uint8_t block_count = 0;
    
    while (remaining > 0) {
        // Check if we need to wait for another FC (block size reached)
        if (fc_bs > 0 && block_count >= fc_bs) {
            ESP_LOGI(TAG, "ISO15765: Block size %d reached, waiting for FC", fc_bs);
            isotp_fc_state.waiting_for_fc = true;
            isotp_fc_state.fc_received = false;
            
            TickType_t bs_start = xTaskGetTickCount();
            while (!isotp_fc_state.fc_received) {
                if ((xTaskGetTickCount() - bs_start) >= fc_timeout) {
                    ESP_LOGW(TAG, "ISO15765: FC timeout after block");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            
            if (isotp_fc_state.fc_received) {
                while (isotp_fc_state.fc_flow_status == 1) {
                    ESP_LOGI(TAG, "ISO15765: FC Wait in block, waiting for CTS...");
                    isotp_fc_state.fc_received = false;
                    
                    TickType_t wait_start = xTaskGetTickCount();
                    while (!isotp_fc_state.fc_received) {
                        if ((xTaskGetTickCount() - wait_start) >= fc_timeout) {
                            ESP_LOGW(TAG, "ISO15765: FC CTS timeout in block");
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    if (!isotp_fc_state.fc_received) break;
                }
                
                if (isotp_fc_state.fc_flow_status == 2) {
                    ESP_LOGE(TAG, "ISO15765: FC Overflow in block, aborting");
                    isotp_fc_state.waiting_for_fc = false;
                    return J2534_ERR_BUFFER_OVERFLOW;
                }
                
                fc_stmin = isotp_fc_state.fc_stmin;
                ESP_LOGI(TAG, "ISO15765: Block FC received, new STmin=%d", fc_stmin);
            }
            
            isotp_fc_state.waiting_for_fc = false;
            block_count = 0;
        }
        
        // Build Consecutive Frame
        memset(&can_frame, 0, sizeof(can_frame));
        can_frame.identifier = can_id;
        can_frame.extd = is_extended ? 1 : 0;
        can_frame.self = 0;
        
        // CF PCI: 0x2N where N is sequence number (0-F, wraps)
        can_frame.data[0] = 0x20 | (seq_num & 0x0F);
        uint32_t cf_len = (remaining > 7) ? 7 : remaining;
        memcpy(&can_frame.data[1], &data[offset], cf_len);
        can_frame.data_length_code = 1 + cf_len;
        
        if (padding && can_frame.data_length_code < 8) {
            memset(&can_frame.data[can_frame.data_length_code], 0xAA,
                   8 - can_frame.data_length_code);
            can_frame.data_length_code = 8;
        }
        
        uint32_t cf_timeout = (timeout > 0) ? timeout : 100;
        send_result = can_send(&can_frame, pdMS_TO_TICKS(cf_timeout));
        if (send_result != ESP_OK) {
            ESP_LOGE(TAG, "ISO15765 CF[%d] send FAILED: err=%d", seq_num, send_result);
            return J2534_ERR_FAILED;
        }
        
        ESP_LOGI(TAG, "ISO15765 CF[%d]: len=%lu", seq_num, cf_len);
        
        offset += cf_len;
        remaining -= cf_len;
        seq_num = (seq_num + 1) & 0x0F;
        block_count++;
        
        // Inter-frame delay (STmin from FC)
        if (remaining > 0 && fc_stmin > 0) {
            if (fc_stmin <= 0x7F) {
                if (fc_stmin == 0) {
                    taskYIELD();
                } else {
                    vTaskDelay(pdMS_TO_TICKS(fc_stmin));
                }
            } else if (fc_stmin >= 0xF1 && fc_stmin <= 0xF9) {
                uint32_t us_delay = (fc_stmin - 0xF0) * 100;
                esp_rom_delay_us(us_delay);
            } else if (fc_stmin >= 0x80 && fc_stmin <= 0xF0) {
                vTaskDelay(pdMS_TO_TICKS(127));
            }
        } else if (remaining > 0) {
            taskYIELD();
        }
    }
    
    // Wait for CAN TX queue to drain
    {
        twai_status_info_t status_info;
        TickType_t drain_start = xTaskGetTickCount();
        TickType_t drain_timeout = pdMS_TO_TICKS(5000);
        
        ESP_LOGI(TAG, "ISO15765: Waiting for TX queue to drain...");
        
        while (1) {
            twai_get_status_info(&status_info);
            if (status_info.msgs_to_tx == 0) {
                ESP_LOGI(TAG, "ISO15765: TX queue drained, all frames sent");
                break;
            }
            if ((xTaskGetTickCount() - drain_start) >= drain_timeout) {
                ESP_LOGW(TAG, "ISO15765: TX drain timeout, %lu msgs still pending",
                         status_info.msgs_to_tx);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    ESP_LOGI(TAG, "ISO15765: Multi-frame TX complete");
    return J2534_STATUS_NOERROR;
}