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
 * @file j2534_channel.c
 * @brief J2534 Channel Management - Connect, disconnect, and filter management
 * 
 * This file contains:
 * - Protocol channel connect/disconnect
 * - Message filter setup and removal
 * - Filter matching logic for incoming frames
 */

#include "j2534_internal.h"
#include "can.h"
#include "stn_j2534.h"
#include "hw_config.h"

#define TAG J2534_TAG

/* ============================================================================
 * Channel Connect/Disconnect
 * ============================================================================ */

j2534_error_t j2534_connect(uint32_t device_id, uint32_t protocol_id,
                            uint32_t flags, uint32_t baudrate,
                            uint32_t *channel_id)
{
    if (!j2534_device_open || device_id != j2534_device_id) {
        j2534_set_error(J2534_ERR_INVALID_DEVICE_ID, "Invalid device ID");
        return J2534_ERR_INVALID_DEVICE_ID;
    }

    // Validate protocol and determine if CAN or legacy (via OBD chip)
    uint32_t base_protocol = protocol_id;
    bool is_legacy_protocol = false;
    bool is_swcan = false;  // Single-Wire CAN (GMLAN)
    stn_protocol_t stn_proto = STN_PROTO_AUTO;

    switch (protocol_id) {
        // CAN-based protocols - use ESP32 TWAI directly
        case J2534_PROTOCOL_CAN:
        case J2534_PROTOCOL_ISO15765:
            break;

        // J2534-2 Extended CAN protocols - map to CAN
        case J2534_PROTOCOL_CAN_PS:
        case J2534_PROTOCOL_CAN_CH1:
        case J2534_PROTOCOL_CAN_CH2:
            base_protocol = J2534_PROTOCOL_CAN;
            ESP_LOGI(TAG, "Mapping extended protocol 0x%04lX to CAN", protocol_id);
            break;

        // Single-Wire CAN (GMLAN) - default 33.3 kbps
        case J2534_PROTOCOL_SW_CAN_PS:
            base_protocol = J2534_PROTOCOL_CAN;
            is_swcan = true;
            if (baudrate == 0) baudrate = 33333;  // Default SWCAN baud rate
            ESP_LOGI(TAG, "SWCAN (GMLAN) protocol, baud=%lu", baudrate);
            break;

        // J2534-2 Extended ISO15765 protocols - map to ISO15765
        case J2534_PROTOCOL_ISO15765_PS:
            base_protocol = J2534_PROTOCOL_ISO15765;
            ESP_LOGI(TAG, "Mapping extended protocol 0x%04lX to ISO15765", protocol_id);
            break;

        // Single-Wire CAN ISO15765 (GMLAN with ISO-TP)
        case J2534_PROTOCOL_SW_ISO15765_PS:
            base_protocol = J2534_PROTOCOL_ISO15765;
            is_swcan = true;
            if (baudrate == 0) baudrate = 33333;  // Default SWCAN baud rate
            ESP_LOGI(TAG, "SWCAN ISO15765 (GMLAN ISO-TP), baud=%lu", baudrate);
            break;

#if HARDWARE_VER == WICAN_PRO
        // Legacy protocols - route through OBD interpreter chip
        case J2534_PROTOCOL_J1850VPW:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_J1850VPW;
            ESP_LOGI(TAG, "Using OBD chip for J1850 VPW");
            break;

        case J2534_PROTOCOL_J1850PWM:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_J1850PWM;
            ESP_LOGI(TAG, "Using OBD chip for J1850 PWM");
            break;

        case J2534_PROTOCOL_ISO9141:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_ISO9141;
            ESP_LOGI(TAG, "Using OBD chip for ISO 9141");
            break;

        case J2534_PROTOCOL_ISO14230:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_ISO14230_FAST;
            ESP_LOGI(TAG, "Using OBD chip for ISO 14230 (KWP2000)");
            break;

        // Extended legacy protocols
        case J2534_PROTOCOL_J1850VPW_PS:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_J1850VPW;
            base_protocol = J2534_PROTOCOL_J1850VPW;
            break;

        case J2534_PROTOCOL_J1850PWM_PS:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_J1850PWM;
            base_protocol = J2534_PROTOCOL_J1850PWM;
            break;

        case J2534_PROTOCOL_ISO9141_PS:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_ISO9141;
            base_protocol = J2534_PROTOCOL_ISO9141;
            break;

        case J2534_PROTOCOL_ISO14230_PS:
            is_legacy_protocol = true;
            stn_proto = STN_PROTO_ISO14230_FAST;
            base_protocol = J2534_PROTOCOL_ISO14230;
            break;
#endif

        default:
            j2534_set_error(J2534_ERR_INVALID_PROTOCOL_ID, "Unsupported protocol");
            ESP_LOGE(TAG, "Unsupported protocol ID: 0x%04lX", protocol_id);
            return J2534_ERR_INVALID_PROTOCOL_ID;
    }

    // Allocate channel
    j2534_channel_t *ch = j2534_alloc_channel();
    if (!ch) {
        j2534_set_error(J2534_ERR_EXCEEDED_LIMIT, "No free channels");
        return J2534_ERR_EXCEEDED_LIMIT;
    }

#if HARDWARE_VER == WICAN_PRO
    if (is_legacy_protocol) {
        // Initialize OBD chip bridge if needed
        if (stn_j2534_init() != ESP_OK) {
            j2534_set_error(J2534_ERR_FAILED, "OBD chip not ready");
            return J2534_ERR_FAILED;
        }

        // Select protocol on OBD chip
        stn_j2534_status_t status = stn_j2534_select_protocol(stn_proto);
        if (status != STN_J2534_STATUS_OK) {
            j2534_set_error(J2534_ERR_FAILED, "Failed to select protocol on OBD chip");
            return J2534_ERR_FAILED;
        }

        // Disable CAN since we're using legacy protocol
        can_disable();
    } else
#endif
    {
        // Configure CAN for CAN-based protocols
        can_disable();
        uint32_t can_rate = j2534_baudrate_to_can(baudrate);
        can_set_bitrate(can_rate);
        can_set_silent(0);
        can_set_loopback((flags & 0x01) ? 1 : 0);
        can_enable();

        // Wait for CAN to be ready
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Suppress unused variable warning when not on PRO
    (void)is_legacy_protocol;
    (void)is_swcan;
    (void)stn_proto;
    (void)base_protocol;

    // Setup channel
    ch->active = true;
    ch->protocol_id = protocol_id;
    ch->flags = flags;
    ch->baudrate = baudrate;
    ch->loopback = (flags & 0x01) ? true : false;
    ch->filter_count = 0;
    ch->funct_msg_count = 0;
    
    // ISO15765 flow control parameters - RX side (what we send in FC frames)
    ch->iso15765_bs = 0;       // Block size: 0 = no limit (receive all CFs without FC)
    ch->iso15765_stmin = 0;    // STmin: 0 = no delay required
    
    // ISO15765 flow control parameters - TX side (override ECU's FC values)
    ch->iso15765_bs_tx = 0;    // 0 = use ECU's block size from FC
    ch->iso15765_stmin_tx = 0; // 0 = use ECU's STmin from FC
    ch->iso15765_wft_max = 0;  // 0 = unlimited wait frames
    
    ch->iso15765_ext_addr = false;
    ch->iso15765_ext_addr_byte = 0;
    
    // Set default timing parameters for ISO9141/ISO14230 (KWP2000)
    // These are J2534 standard defaults in milliseconds
    ch->p1_min = 0;
    ch->p1_max = 20;      // Inter-byte time ECU response
    ch->p2_min = 25;
    ch->p2_max = 50;      // Request to response time (P2 timeout)
    ch->p3_min = 55;
    ch->p3_max = 5000;    // Response to next request
    ch->p4_min = 5;
    ch->p4_max = 20;      // Inter-byte time tester request

    // Clear RX buffer
    j2534_rx_msg_head = 0;
    j2534_rx_msg_tail = 0;

    *channel_id = ch->channel_id;
    j2534_active_channel = ch->channel_id;

    ESP_LOGI(TAG, "Connected channel %lu, protocol 0x%lX, baud %lu",
             ch->channel_id, protocol_id, baudrate);

    return J2534_STATUS_NOERROR;
}

j2534_error_t j2534_disconnect(uint32_t channel_id)
{
    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

#if HARDWARE_VER == WICAN_PRO
    // Stop KWP keep-alive if active
    if (j2534_is_legacy_protocol(ch->protocol_id)) {
        stn_j2534_stop_keep_alive();
    }
#endif

    // Stop all periodic messages for this channel
    for (int i = 0; i < J2534_MAX_PERIODIC_MSGS_ACTIVE; i++) {
        if (j2534_periodic_msgs[i].active && j2534_periodic_msgs[i].channel_id == channel_id) {
            j2534_periodic_msgs[i].active = false;
            ESP_LOGI(TAG, "Stopped periodic msg %lu during disconnect", j2534_periodic_msgs[i].msg_id);
        }
    }

    // Reset ISO-TP state
    memset((void*)&isotp_fc_state, 0, sizeof(isotp_fc_state));
    memset((void*)&isotp_tx_state, 0, sizeof(isotp_tx_state));
    memset((void*)&isotp_rx_state, 0, sizeof(isotp_rx_state));

    // Clear filters
    memset(ch->filters, 0, sizeof(ch->filters));
    ch->filter_count = 0;

    // Deactivate channel
    ch->active = false;

    if (j2534_active_channel == channel_id) {
        j2534_active_channel = 0;
        can_disable();
    }

    ESP_LOGI(TAG, "Disconnected channel %lu", channel_id);
    return J2534_STATUS_NOERROR;
}

/* ============================================================================
 * Message Filters
 * ============================================================================ */

j2534_error_t j2534_start_msg_filter(uint32_t channel_id,
                                     uint32_t filter_type,
                                     j2534_msg_t *mask_msg,
                                     j2534_msg_t *pattern_msg,
                                     j2534_msg_t *flow_control_msg,
                                     uint32_t *filter_id)
{
    ESP_LOGI(TAG, "j2534_start_msg_filter: ch_id=%lu type=%lu", channel_id, filter_type);

    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        ESP_LOGE(TAG, "j2534_start_msg_filter: Invalid channel ID %lu", channel_id);
        j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    // Log filter details
    if (mask_msg) {
        ESP_LOGI(TAG, "  MASK: size=%lu data=%02X %02X %02X %02X...",
                 mask_msg->data_size,
                 mask_msg->data_size > 0 ? mask_msg->data[0] : 0,
                 mask_msg->data_size > 1 ? mask_msg->data[1] : 0,
                 mask_msg->data_size > 2 ? mask_msg->data[2] : 0,
                 mask_msg->data_size > 3 ? mask_msg->data[3] : 0);
    }
    if (pattern_msg) {
        ESP_LOGI(TAG, "  PATTERN: size=%lu data=%02X %02X %02X %02X...",
                 pattern_msg->data_size,
                 pattern_msg->data_size > 0 ? pattern_msg->data[0] : 0,
                 pattern_msg->data_size > 1 ? pattern_msg->data[1] : 0,
                 pattern_msg->data_size > 2 ? pattern_msg->data[2] : 0,
                 pattern_msg->data_size > 3 ? pattern_msg->data[3] : 0);
    }
    if (flow_control_msg) {
        ESP_LOGI(TAG, "  FLOW_CTRL: size=%lu data=%02X %02X %02X %02X...",
                 flow_control_msg->data_size,
                 flow_control_msg->data_size > 0 ? flow_control_msg->data[0] : 0,
                 flow_control_msg->data_size > 1 ? flow_control_msg->data[1] : 0,
                 flow_control_msg->data_size > 2 ? flow_control_msg->data[2] : 0,
                 flow_control_msg->data_size > 3 ? flow_control_msg->data[3] : 0);
    }

    if (ch->filter_count >= J2534_MAX_FILTERS) {
        j2534_set_error(J2534_ERR_EXCEEDED_LIMIT, "Max filters reached");
        return J2534_ERR_EXCEEDED_LIMIT;
    }

    // Find free filter slot
    j2534_filter_t *filter = NULL;
    uint32_t fid = 0;
    for (int i = 0; i < J2534_MAX_FILTERS; i++) {
        if (!ch->filters[i].active) {
            filter = &ch->filters[i];
            fid = i + 1;
            break;
        }
    }

    if (!filter) {
        j2534_set_error(J2534_ERR_EXCEEDED_LIMIT, "No free filter slots");
        return J2534_ERR_EXCEEDED_LIMIT;
    }

    // Setup filter
    filter->filter_id = fid;
    filter->filter_type = filter_type;
    filter->protocol_id = ch->protocol_id;
    filter->active = true;

    // Copy mask (first 4 bytes of data are CAN ID)
    if (mask_msg && mask_msg->data_size >= 4) {
        memcpy(filter->mask, mask_msg->data, 4);
    }

    // Copy pattern
    if (pattern_msg && pattern_msg->data_size >= 4) {
        memcpy(filter->pattern, pattern_msg->data, 4);
    }

    // Copy flow control (for ISO15765)
    if (flow_control_msg && flow_control_msg->data_size >= 4) {
        memcpy(filter->flow_control, flow_control_msg->data, 4);
    }

    ch->filter_count++;
    *filter_id = fid;

    // Clear RX buffer when filter is set
    j2534_rx_msg_head = 0;
    j2534_rx_msg_tail = 0;

    ESP_LOGI(TAG, "Started filter %lu on channel %lu", fid, channel_id);
    return J2534_STATUS_NOERROR;
}

j2534_error_t j2534_stop_msg_filter(uint32_t channel_id, uint32_t filter_id)
{
    j2534_channel_t *ch = j2534_get_channel(channel_id);
    if (!ch) {
        j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    if (filter_id == 0 || filter_id > J2534_MAX_FILTERS) {
        j2534_set_error(J2534_ERR_INVALID_FILTER_ID, "Invalid filter ID");
        return J2534_ERR_INVALID_FILTER_ID;
    }

    j2534_filter_t *filter = &ch->filters[filter_id - 1];
    if (!filter->active) {
        j2534_set_error(J2534_ERR_INVALID_FILTER_ID, "Filter not active");
        return J2534_ERR_INVALID_FILTER_ID;
    }

    filter->active = false;
    ch->filter_count--;

    ESP_LOGI(TAG, "Stopped filter %lu on channel %lu", filter_id, channel_id);
    return J2534_STATUS_NOERROR;
}

void j2534_clear_all_filters(j2534_channel_t *ch)
{
    if (!ch) return;
    memset(ch->filters, 0, sizeof(ch->filters));
    ch->filter_count = 0;
    ESP_LOGI(TAG, "Cleared all filters on channel %lu", ch->channel_id);
}

/* ============================================================================
 * Filter Matching Logic
 * ============================================================================ */

/**
 * @brief Check if a CAN frame matches the channel's filters
 * @param ch Channel to check filters on
 * @param frame Received CAN frame
 * @param flow_control_id Output: flow control CAN ID if ISO15765 filter matched
 * @return true if frame should be passed through, false if blocked
 */
bool j2534_filter_message(j2534_channel_t *ch, twai_message_t *frame, uint32_t *flow_control_id)
{
    if (!ch || !frame) return false;

    // Convert frame CAN ID to 4-byte format for filter comparison
    uint8_t frame_id[4] = {
        (frame->identifier >> 24) & 0xFF,
        (frame->identifier >> 16) & 0xFF,
        (frame->identifier >> 8) & 0xFF,
        frame->identifier & 0xFF
    };

    // For 11-bit CAN IDs, they're in the lower bits
    if (!frame->extd) {
        frame_id[0] = 0;
        frame_id[1] = 0;
        frame_id[2] = (frame->identifier >> 8) & 0x07;
        frame_id[3] = frame->identifier & 0xFF;
    }

    bool has_pass_filter = false;
    bool passed = false;

    // First pass: check if any PASS filters exist
    for (int i = 0; i < J2534_MAX_FILTERS; i++) {
        if (ch->filters[i].active && ch->filters[i].filter_type == J2534_FILTER_PASS) {
            has_pass_filter = true;
            break;
        }
    }

    // Second pass: apply filters
    for (int i = 0; i < J2534_MAX_FILTERS; i++) {
        if (!ch->filters[i].active) continue;

        j2534_filter_t *f = &ch->filters[i];

        // Apply mask to both frame ID and pattern
        bool match = true;
        for (int j = 0; j < 4; j++) {
            if ((frame_id[j] & f->mask[j]) != (f->pattern[j] & f->mask[j])) {
                match = false;
                break;
            }
        }

        if (match) {
            if (f->filter_type == J2534_FILTER_PASS || f->filter_type == J2534_FILTER_FLOW_CONTROL) {
                passed = true;
                if (flow_control_id && f->filter_type == J2534_FILTER_FLOW_CONTROL) {
                    *flow_control_id = (f->flow_control[0] << 24) |
                                       (f->flow_control[1] << 16) |
                                       (f->flow_control[2] << 8) |
                                       f->flow_control[3];
                }
            } else if (f->filter_type == J2534_FILTER_BLOCK) {
                return false;  // Blocked
            }
        }
    }

    // For ISO15765 with functional addressing, allow responses in certain ranges
    // BUT only when NO explicit PASS filters are configured
    if (ch->protocol_id == J2534_PROTOCOL_ISO15765 && !has_pass_filter) {
        uint32_t can_id = frame->identifier;

        // Standard OBD-II response range (0x7E8-0x7EF)
        if (can_id >= 0x7E8 && can_id <= 0x7EF) {
            return true;
        }

        // ECU response ranges for physical addressing
        if ((can_id >= 0x600 && can_id <= 0x6FF) ||
            (can_id >= 0x700 && can_id <= 0x7FF)) {
            return true;
        }
    }

    // For raw CAN with no filters, pass everything
    if (ch->protocol_id == J2534_PROTOCOL_CAN && ch->filter_count == 0) {
        return true;
    }

    // If we have PASS filters but none matched, block
    if (has_pass_filter && !passed) {
        return false;
    }

    // No PASS filters configured and not blocked = pass through
    return !has_pass_filter || passed;
}
