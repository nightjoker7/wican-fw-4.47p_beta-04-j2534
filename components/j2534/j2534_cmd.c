/**
 * @file j2534_cmd.c
 * @brief J2534 command processing and protocol parser
 *
 * This file contains:
 * - Binary protocol parsing (SYNC, command, length, data, checksum)
 * - Command dispatch and response framing
 */

#include "j2534.h"
#include "j2534_internal.h"
#include "can.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "j2534_cmd";

/* ===========================================================================
 * Protocol Command Processing
 * =========================================================================== */

void j2534_handle_command(uint8_t cmd, uint8_t *data, uint16_t len, QueueHandle_t *q)
{
    static uint8_t resp_data[512];
    uint16_t resp_len = 0;
    j2534_error_t status = J2534_STATUS_NOERROR;

    switch (cmd) {
        case J2534_CMD_OPEN:
        {
            uint32_t dev_id;
            status = j2534_open(&dev_id);
            resp_data[0] = (dev_id >> 24) & 0xFF;
            resp_data[1] = (dev_id >> 16) & 0xFF;
            resp_data[2] = (dev_id >> 8) & 0xFF;
            resp_data[3] = dev_id & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_CLOSE:
        {
            uint32_t dev_id = (data[0] << 24) | (data[1] << 16) |
                              (data[2] << 8) | data[3];
            status = j2534_close(dev_id);
            break;
        }

        case J2534_CMD_CONNECT:
        {
            if (len < 16) {
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t dev_id = (data[0] << 24) | (data[1] << 16) |
                              (data[2] << 8) | data[3];
            uint32_t protocol = (data[4] << 24) | (data[5] << 16) |
                                (data[6] << 8) | data[7];
            uint32_t flags = (data[8] << 24) | (data[9] << 16) |
                             (data[10] << 8) | data[11];
            uint32_t baud = (data[12] << 24) | (data[13] << 16) |
                            (data[14] << 8) | data[15];
            uint32_t ch_id;
            status = j2534_connect(dev_id, protocol, flags, baud, &ch_id);
            resp_data[0] = (ch_id >> 24) & 0xFF;
            resp_data[1] = (ch_id >> 16) & 0xFF;
            resp_data[2] = (ch_id >> 8) & 0xFF;
            resp_data[3] = ch_id & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_DISCONNECT:
        {
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            status = j2534_disconnect(ch_id);
            break;
        }

        case J2534_CMD_START_PERIODIC_MSG:
        {
            ESP_LOGI(TAG, "CMD_START_PERIODIC_MSG: len=%u", len);
            if (len < 20) {
                ESP_LOGE(TAG, "START_PERIODIC_MSG: len %u < 20", len);
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t interval = (data[4] << 24) | (data[5] << 16) |
                                (data[6] << 8) | data[7];

            j2534_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.protocol_id = (data[8] << 24) | (data[9] << 16) |
                              (data[10] << 8) | data[11];
            msg.tx_flags = (data[12] << 24) | (data[13] << 16) |
                           (data[14] << 8) | data[15];
            msg.data_size = (data[16] << 24) | (data[17] << 16) |
                            (data[18] << 8) | data[19];

            if (msg.data_size > 0 && len >= 20 + msg.data_size) {
                memcpy(msg.data, &data[20], msg.data_size);
            }

            uint32_t msg_id;
            status = j2534_start_periodic_msg(ch_id, &msg, interval, &msg_id);
            resp_data[0] = (msg_id >> 24) & 0xFF;
            resp_data[1] = (msg_id >> 16) & 0xFF;
            resp_data[2] = (msg_id >> 8) & 0xFF;
            resp_data[3] = msg_id & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_STOP_PERIODIC_MSG:
        {
            if (len < 8) {
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t msg_id = (data[4] << 24) | (data[5] << 16) |
                              (data[6] << 8) | data[7];
            status = j2534_stop_periodic_msg(ch_id, msg_id);
            break;
        }

        case J2534_CMD_READ_MSGS:
        {
            if (len < 12) {
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t max_msgs = (data[4] << 24) | (data[5] << 16) |
                                (data[6] << 8) | data[7];
            uint32_t timeout = (data[8] << 24) | (data[9] << 16) |
                               (data[10] << 8) | data[11];

            j2534_msg_t msgs[2];
            uint32_t num_msgs = (max_msgs > 2) ? 2 : max_msgs;

            status = j2534_read_msgs(ch_id, msgs, &num_msgs, timeout);

            uint16_t max_resp = 500;

            resp_data[0] = (num_msgs >> 24) & 0xFF;
            resp_data[1] = (num_msgs >> 16) & 0xFF;
            resp_data[2] = (num_msgs >> 8) & 0xFF;
            resp_data[3] = num_msgs & 0xFF;
            resp_len = 4;

            for (uint32_t i = 0; i < num_msgs; i++) {
                uint32_t msg_overhead = 20;
                if (resp_len + msg_overhead + msgs[i].data_size > max_resp) {
                    ESP_LOGW(TAG, "READ_MSGS: Response too large, truncating at msg %lu", i);
                    resp_data[0] = (i >> 24) & 0xFF;
                    resp_data[1] = (i >> 16) & 0xFF;
                    resp_data[2] = (i >> 8) & 0xFF;
                    resp_data[3] = i & 0xFF;
                    break;
                }

                // Protocol ID
                resp_data[resp_len++] = (msgs[i].protocol_id >> 24) & 0xFF;
                resp_data[resp_len++] = (msgs[i].protocol_id >> 16) & 0xFF;
                resp_data[resp_len++] = (msgs[i].protocol_id >> 8) & 0xFF;
                resp_data[resp_len++] = msgs[i].protocol_id & 0xFF;
                // RX status
                resp_data[resp_len++] = (msgs[i].rx_status >> 24) & 0xFF;
                resp_data[resp_len++] = (msgs[i].rx_status >> 16) & 0xFF;
                resp_data[resp_len++] = (msgs[i].rx_status >> 8) & 0xFF;
                resp_data[resp_len++] = msgs[i].rx_status & 0xFF;
                // Timestamp
                resp_data[resp_len++] = (msgs[i].timestamp >> 24) & 0xFF;
                resp_data[resp_len++] = (msgs[i].timestamp >> 16) & 0xFF;
                resp_data[resp_len++] = (msgs[i].timestamp >> 8) & 0xFF;
                resp_data[resp_len++] = msgs[i].timestamp & 0xFF;
                // Data size
                resp_data[resp_len++] = (msgs[i].data_size >> 24) & 0xFF;
                resp_data[resp_len++] = (msgs[i].data_size >> 16) & 0xFF;
                resp_data[resp_len++] = (msgs[i].data_size >> 8) & 0xFF;
                resp_data[resp_len++] = msgs[i].data_size & 0xFF;
                // Extra data index (= DataSize per J2534 spec)
                resp_data[resp_len++] = (msgs[i].data_size >> 24) & 0xFF;
                resp_data[resp_len++] = (msgs[i].data_size >> 16) & 0xFF;
                resp_data[resp_len++] = (msgs[i].data_size >> 8) & 0xFF;
                resp_data[resp_len++] = msgs[i].data_size & 0xFF;
                // Data
                memcpy(&resp_data[resp_len], msgs[i].data, msgs[i].data_size);
                resp_len += msgs[i].data_size;
            }
            break;
        }

        case J2534_CMD_WRITE_MSGS:
        {
            ESP_LOGI(TAG, "CMD_WRITE_MSGS: len=%u", len);
            if (len < 24) {
                ESP_LOGE(TAG, "CMD_WRITE_MSGS: len < 24");
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t timeout = (data[4] << 24) | (data[5] << 16) |
                               (data[6] << 8) | data[7];

            ESP_LOGI(TAG, "CMD_WRITE_MSGS: ch_id=%lu timeout=%lu", ch_id, timeout);

            j2534_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.protocol_id = (data[8] << 24) | (data[9] << 16) |
                              (data[10] << 8) | data[11];
            msg.tx_flags = (data[12] << 24) | (data[13] << 16) |
                           (data[14] << 8) | data[15];
            msg.data_size = (data[16] << 24) | (data[17] << 16) |
                            (data[18] << 8) | data[19];

            ESP_LOGI(TAG, "CMD_WRITE_MSGS: proto=%lu flags=%lu data_size=%lu",
                     msg.protocol_id, msg.tx_flags, msg.data_size);

            if (msg.data_size > 0 && len >= 20 + msg.data_size) {
                if (msg.data_size <= J2534_MAX_MSG_DATA_SIZE) {
                    memcpy(msg.data, &data[20], msg.data_size);
                    msg.ext_data = NULL;
                } else {
                    msg.ext_data = &data[20];
                    ESP_LOGI(TAG, "CMD_WRITE_MSGS: Using ext_data for large msg (%lu bytes)", msg.data_size);
                }
                ESP_LOGI(TAG, "CMD_WRITE_MSGS: CAN_ID=0x%02X%02X%02X%02X",
                         data[20], data[21], data[22], data[23]);
            }

            uint32_t num = 1;
            status = j2534_write_msgs(ch_id, &msg, &num, timeout);
            ESP_LOGI(TAG, "CMD_WRITE_MSGS: status=%d num=%lu", status, num);
            resp_data[0] = (num >> 24) & 0xFF;
            resp_data[1] = (num >> 16) & 0xFF;
            resp_data[2] = (num >> 8) & 0xFF;
            resp_data[3] = num & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_START_MSG_FILTER:
        {
            ESP_LOGI(TAG, "CMD_START_MSG_FILTER: len=%u", len);
            if (len < 8) {
                ESP_LOGE(TAG, "CMD_START_MSG_FILTER: len < 8");
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t filter_type = (data[4] << 24) | (data[5] << 16) |
                                   (data[6] << 8) | data[7];

            ESP_LOGI(TAG, "CMD_START_MSG_FILTER: ch_id=%lu filter_type=%lu len=%u", ch_id, filter_type, len);

            j2534_msg_t mask_msg, pattern_msg, flow_ctrl_msg;
            j2534_msg_t *flow_ctrl_ptr = NULL;
            memset(&mask_msg, 0, sizeof(mask_msg));
            memset(&pattern_msg, 0, sizeof(pattern_msg));
            memset(&flow_ctrl_msg, 0, sizeof(flow_ctrl_msg));

            if (len >= 16) {
                memcpy(mask_msg.data, &data[8], 4);
                mask_msg.data_size = 4;
                memcpy(pattern_msg.data, &data[12], 4);
                pattern_msg.data_size = 4;
                ESP_LOGI(TAG, "CMD_START_MSG_FILTER: mask=%02X%02X%02X%02X pattern=%02X%02X%02X%02X",
                         data[8], data[9], data[10], data[11],
                         data[12], data[13], data[14], data[15]);
            }

            if (filter_type == 3 && len >= 20) {
                memcpy(flow_ctrl_msg.data, &data[16], 4);
                flow_ctrl_msg.data_size = 4;
                flow_ctrl_ptr = &flow_ctrl_msg;
                ESP_LOGI(TAG, "CMD_START_MSG_FILTER: flow_ctrl=%02X%02X%02X%02X",
                         data[16], data[17], data[18], data[19]);
            }

            uint32_t filter_id;
            status = j2534_start_msg_filter(ch_id, filter_type,
                                            &mask_msg, &pattern_msg, flow_ctrl_ptr, &filter_id);
            ESP_LOGI(TAG, "CMD_START_MSG_FILTER: status=%d filter_id=%lu", status, filter_id);
            resp_data[0] = (filter_id >> 24) & 0xFF;
            resp_data[1] = (filter_id >> 16) & 0xFF;
            resp_data[2] = (filter_id >> 8) & 0xFF;
            resp_data[3] = filter_id & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_STOP_MSG_FILTER:
        {
            if (len < 8) {
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t filter_id = (data[4] << 24) | (data[5] << 16) |
                                 (data[6] << 8) | data[7];
            status = j2534_stop_msg_filter(ch_id, filter_id);
            break;
        }

        case J2534_CMD_READ_VERSION:
        {
            char fw[32], dll[32], api[32];
            status = j2534_read_version(j2534_device_id, fw, dll, api);
            memcpy(&resp_data[0], fw, 32);
            memcpy(&resp_data[32], dll, 32);
            memcpy(&resp_data[64], api, 32);
            resp_len = 96;
            break;
        }

        case J2534_CMD_IOCTL:
        {
            if (len < 8) {
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t ioctl_id = (data[4] << 24) | (data[5] << 16) |
                                (data[6] << 8) | data[7];

            uint32_t output_val = 0;

            if (ioctl_id == J2534_IOCTL_SET_CONFIG) {
                if (len < 12) {
                    status = J2534_ERR_INVALID_MSG;
                    break;
                }
                uint32_t num_params = (data[8] << 24) | (data[9] << 16) |
                                      (data[10] << 8) | data[11];

                if (len < 12 + (num_params * 8)) {
                    status = J2534_ERR_INVALID_MSG;
                    break;
                }

                j2534_sconfig_t *pairs = malloc(sizeof(j2534_sconfig_t) * num_params);
                if (!pairs) {
                    status = J2534_ERR_FAILED;
                    break;
                }

                for (uint32_t i = 0; i < num_params; i++) {
                    uint32_t offset = 12 + (i * 8);
                    pairs[i].parameter = (data[offset] << 24) | (data[offset+1] << 16) |
                                         (data[offset+2] << 8) | data[offset+3];
                    pairs[i].value = (data[offset+4] << 24) | (data[offset+5] << 16) |
                                     (data[offset+6] << 8) | data[offset+7];
                }

                j2534_sconfig_list_t list;
                list.num_of_params = num_params;
                list.config_ptr = pairs;

                status = j2534_ioctl(ch_id, ioctl_id, &list, &output_val);
                free(pairs);
            } else {
                status = j2534_ioctl(ch_id, ioctl_id, &data[8], &output_val);
            }

            resp_data[0] = (output_val >> 24) & 0xFF;
            resp_data[1] = (output_val >> 16) & 0xFF;
            resp_data[2] = (output_val >> 8) & 0xFF;
            resp_data[3] = output_val & 0xFF;
            resp_len = 4;
            break;
        }

        case J2534_CMD_GET_LAST_ERROR:
        {
            char err_desc[80];
            j2534_get_last_error(err_desc);
            memcpy(resp_data, err_desc, 80);
            resp_len = 80;
            break;
        }

        case J2534_CMD_WRITE_MSGS_BATCH:
        {
            /**
             * Batch write for raw CAN frames - used for ECU reprogramming
             * 
             * Packet format:
             *   channel_id(4) + timeout(4) + num_msgs(4) + [msg_len(2) + msg_data(msg_len)]...
             * 
             * Each message is: CAN_ID(4) + data(8) = 12 bytes for raw CAN
             * 
             * This command queues ALL frames to the CAN TX buffer before responding,
             * enabling high-speed consecutive frame transmission for ISO-TP.
             */
            ESP_LOGI(TAG, "CMD_WRITE_MSGS_BATCH: len=%u", len);
            if (len < 12) {
                ESP_LOGE(TAG, "CMD_WRITE_MSGS_BATCH: len < 12");
                status = J2534_ERR_INVALID_MSG;
                break;
            }
            
            uint32_t ch_id = (data[0] << 24) | (data[1] << 16) |
                             (data[2] << 8) | data[3];
            uint32_t timeout = (data[4] << 24) | (data[5] << 16) |
                               (data[6] << 8) | data[7];
            uint32_t num_msgs_to_send = (data[8] << 24) | (data[9] << 16) |
                                        (data[10] << 8) | data[11];
            
            ESP_LOGI(TAG, "CMD_WRITE_MSGS_BATCH: ch=%lu timeout=%lu num=%lu", 
                     ch_id, timeout, num_msgs_to_send);
            
            j2534_channel_t *ch = j2534_get_channel(ch_id);
            if (!ch) {
                status = J2534_ERR_INVALID_CHANNEL_ID;
                break;
            }
            
            // Switch active channel if needed
            if (j2534_active_channel != ch_id) {
                twai_clear_receive_queue();
                j2534_active_channel = ch_id;
                j2534_rx_msg_head = 0;
                j2534_rx_msg_tail = 0;
            }
            
            uint32_t sent_count = 0;
            uint32_t offset = 12;  // Start after header
            uint32_t send_timeout = (timeout > 0) ? timeout : 100;
            
            for (uint32_t i = 0; i < num_msgs_to_send && offset + 2 <= len; i++) {
                // Read message length (2 bytes)
                uint16_t msg_len = (data[offset] << 8) | data[offset + 1];
                offset += 2;
                
                if (msg_len < 5 || offset + msg_len > len) {  // min: CAN_ID(4) + flags(1)
                    ESP_LOGW(TAG, "BATCH: Invalid msg len %u at offset %lu", msg_len, offset);
                    break;
                }
                
                // Parse CAN frame
                twai_message_t can_frame;
                memset(&can_frame, 0, sizeof(can_frame));
                
                can_frame.identifier = (data[offset] << 24) |
                                       (data[offset + 1] << 16) |
                                       (data[offset + 2] << 8) |
                                       data[offset + 3];
                
                // Parse flags byte (bit 0 = extended ID)
                uint8_t flags = data[offset + 4];
                if (flags & 0x01) {
                    // 29-bit extended ID
                    can_frame.identifier &= 0x1FFFFFFF;
                    can_frame.extd = 1;
                } else {
                    // 11-bit standard ID
                    can_frame.identifier &= 0x7FF;
                    can_frame.extd = 0;
                }
                can_frame.self = 0;
                
                uint8_t payload_len = msg_len - 5;  // subtract CAN_ID(4) + flags(1)
                if (payload_len > 8) payload_len = 8;
                can_frame.data_length_code = payload_len;
                memcpy(can_frame.data, &data[offset + 5], payload_len);
                
                offset += msg_len;
                
                // Queue the frame
                esp_err_t send_result = can_send(&can_frame, pdMS_TO_TICKS(send_timeout));
                if (send_result == ESP_OK) {
                    sent_count++;
                } else {
                    ESP_LOGW(TAG, "BATCH: Frame %lu send failed: %d", i, send_result);
                    break;
                }
            }
            
            // Wait for TX queue to drain (important for timing-critical operations)
            twai_status_info_t status_info;
            TickType_t drain_start = xTaskGetTickCount();
            TickType_t drain_timeout = pdMS_TO_TICKS(5000);
            
            while (1) {
                twai_get_status_info(&status_info);
                if (status_info.msgs_to_tx == 0) {
                    break;
                }
                if ((xTaskGetTickCount() - drain_start) >= drain_timeout) {
                    ESP_LOGW(TAG, "BATCH: TX drain timeout, %lu msgs pending",
                             status_info.msgs_to_tx);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            
            ESP_LOGI(TAG, "CMD_WRITE_MSGS_BATCH: sent %lu/%lu msgs", sent_count, num_msgs_to_send);
            
            resp_data[0] = (sent_count >> 24) & 0xFF;
            resp_data[1] = (sent_count >> 16) & 0xFF;
            resp_data[2] = (sent_count >> 8) & 0xFF;
            resp_data[3] = sent_count & 0xFF;
            resp_len = 4;
            break;
        }

        default:
            status = J2534_ERR_NOT_SUPPORTED;
            break;
    }

    j2534_send_response(cmd, status, resp_data, resp_len, q);
}

/* ===========================================================================
 * Protocol Parser
 * =========================================================================== */

int8_t j2534_process(uint8_t *buf, uint32_t len, twai_message_t *frame, QueueHandle_t *q)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];

        switch (j2534_parse_state) {
            case 0:  // Wait for SYNC1
                if (byte == J2534_SYNC1) {
                    j2534_rx_buffer[0] = byte;
                    j2534_rx_index = 1;
                    j2534_parse_state = 1;
                }
                break;

            case 1:  // Wait for SYNC2
                if (byte == J2534_SYNC2) {
                    j2534_rx_buffer[1] = byte;
                    j2534_rx_index = 2;
                    j2534_parse_state = 2;
                } else {
                    j2534_parse_state = 0;
                }
                break;

            case 2:  // Command byte
                j2534_rx_buffer[j2534_rx_index++] = byte;
                j2534_parse_state = 3;
                break;

            case 3:  // Length high byte
                j2534_rx_buffer[j2534_rx_index++] = byte;
                j2534_expected_len = byte << 8;
                j2534_parse_state = 4;
                break;

            case 4:  // Length low byte
                j2534_rx_buffer[j2534_rx_index++] = byte;
                j2534_expected_len |= byte;
                if (j2534_expected_len > J2534_MAX_PACKET_SIZE - 10) {
                    j2534_parse_state = 0;
                } else if (j2534_expected_len == 0) {
                    j2534_parse_state = 6;  // Skip to checksum
                } else {
                    j2534_parse_state = 5;
                }
                break;

            case 5:  // Data bytes
                j2534_rx_buffer[j2534_rx_index++] = byte;
                if (j2534_rx_index >= 5 + j2534_expected_len) {
                    j2534_parse_state = 6;
                }
                break;

            case 6:  // Checksum
            {
                uint8_t calc_cs = j2534_calc_checksum(j2534_rx_buffer, j2534_rx_index);
                if (calc_cs == byte) {
                    uint8_t cmd = j2534_rx_buffer[2];
                    j2534_handle_command(cmd, &j2534_rx_buffer[5], j2534_expected_len, q);
                } else {
                    ESP_LOGW(TAG, "Checksum mismatch: expected %02X, got %02X",
                             calc_cs, byte);
                }
                j2534_parse_state = 0;
                break;
            }

            default:
                j2534_parse_state = 0;
                break;
        }
    }

    return 0;
}
