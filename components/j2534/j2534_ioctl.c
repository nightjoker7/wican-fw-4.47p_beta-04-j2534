/**
 * @file j2534_ioctl.c
 * @brief J2534 IOCTL operations
 *
 * This file contains IOCTL handling functions:
 * - GET_CONFIG / SET_CONFIG
 * - Buffer clearing operations
 * - Filter management
 * - Legacy protocol initialization (5-baud, fast init)
 */

#include "j2534.h"
#include "j2534_internal.h"
#include "can.h"
#include "esp_log.h"
#include <string.h>

#if HARDWARE_VER == WICAN_PRO
#include "stn_j2534.h"
#endif

static const char *TAG = "j2534_ioctl";

/* ===========================================================================
 * IOCTL
 * =========================================================================== */

j2534_error_t j2534_ioctl(uint32_t channel_id, uint32_t ioctl_id,
                          void *input, void *output)
{
    j2534_channel_t *ch = NULL;

    // Some IOCTLs don't need a valid channel
    if (ioctl_id != J2534_IOCTL_READ_VBATT &&
        ioctl_id != J2534_IOCTL_READ_PROG_VOLTAGE) {
        ch = j2534_get_channel(channel_id);
        if (!ch) {
            j2534_set_error(J2534_ERR_INVALID_CHANNEL_ID, "Invalid channel ID");
            return J2534_ERR_INVALID_CHANNEL_ID;
        }
    }

    switch (ioctl_id) {
        case J2534_IOCTL_GET_CONFIG:
            if (!input || !output) {
                return J2534_ERR_NULL_PARAMETER;
            }
            {
                j2534_sconfig_list_t *list = (j2534_sconfig_list_t *)input;
                for (uint32_t i = 0; i < list->num_of_params; i++) {
                    switch (list->config_ptr[i].parameter) {
                        case J2534_CONFIG_DATA_RATE:
                            list->config_ptr[i].value = ch->baudrate;
                            break;
                        case J2534_CONFIG_LOOPBACK:
                            list->config_ptr[i].value = ch->loopback ? 1 : 0;
                            break;
                        case J2534_CONFIG_ISO15765_BS:
                            list->config_ptr[i].value = ch->iso15765_bs;
                            break;
                        case J2534_CONFIG_ISO15765_STMIN:
                            list->config_ptr[i].value = ch->iso15765_stmin;
                            break;
                        case J2534_CONFIG_ISO15765_BS_TX:
                            list->config_ptr[i].value = ch->iso15765_bs_tx;
                            break;
                        case J2534_CONFIG_ISO15765_STMIN_TX:
                            list->config_ptr[i].value = ch->iso15765_stmin_tx;
                            break;
                        case J2534_CONFIG_ISO15765_WFT_MAX:
                            list->config_ptr[i].value = ch->iso15765_wft_max;
                            break;
                        // KWP2000/ISO9141 timing parameters
                        case J2534_CONFIG_P1_MIN:
                            list->config_ptr[i].value = ch->p1_min;
                            break;
                        case J2534_CONFIG_P1_MAX:
                            list->config_ptr[i].value = ch->p1_max;
                            break;
                        case J2534_CONFIG_P2_MIN:
                            list->config_ptr[i].value = ch->p2_min;
                            break;
                        case J2534_CONFIG_P2_MAX:
                            list->config_ptr[i].value = ch->p2_max;
                            break;
                        case J2534_CONFIG_P3_MIN:
                            list->config_ptr[i].value = ch->p3_min;
                            break;
                        case J2534_CONFIG_P3_MAX:
                            list->config_ptr[i].value = ch->p3_max;
                            break;
                        case J2534_CONFIG_P4_MIN:
                            list->config_ptr[i].value = ch->p4_min;
                            break;
                        case J2534_CONFIG_P4_MAX:
                            list->config_ptr[i].value = ch->p4_max;
                            break;
                        default:
                            list->config_ptr[i].value = 0;
                            break;
                    }
                }
            }
            break;

        case J2534_IOCTL_SET_CONFIG:
            if (!input) {
                return J2534_ERR_NULL_PARAMETER;
            }
            {
                j2534_sconfig_list_t *list = (j2534_sconfig_list_t *)input;
                bool timing_changed = false;
                
                for (uint32_t i = 0; i < list->num_of_params; i++) {
                    switch (list->config_ptr[i].parameter) {
                        case J2534_CONFIG_DATA_RATE:
                            ch->baudrate = list->config_ptr[i].value;
                            can_disable();
                            can_set_bitrate(j2534_baudrate_to_can(ch->baudrate));
                            can_enable();
                            break;
                        case J2534_CONFIG_LOOPBACK:
                            ch->loopback = list->config_ptr[i].value ? true : false;
                            if (ch->active) {
                                can_disable();
                                can_set_loopback(ch->loopback ? 1 : 0);
                                can_enable();
                            }
                            break;
                        case J2534_CONFIG_ISO15765_BS:
                            ch->iso15765_bs = list->config_ptr[i].value;
                            ESP_LOGI(TAG, "SET_CONFIG: ISO15765_BS=%u", ch->iso15765_bs);
                            break;
                        case J2534_CONFIG_ISO15765_STMIN:
                            ch->iso15765_stmin = list->config_ptr[i].value;
                            ESP_LOGI(TAG, "SET_CONFIG: ISO15765_STMIN=%u", ch->iso15765_stmin);
                            break;
                        case J2534_CONFIG_ISO15765_BS_TX:
                            ch->iso15765_bs_tx = list->config_ptr[i].value;
                            ESP_LOGI(TAG, "SET_CONFIG: ISO15765_BS_TX=%u (0=use ECU value)", ch->iso15765_bs_tx);
                            break;
                        case J2534_CONFIG_ISO15765_STMIN_TX:
                            ch->iso15765_stmin_tx = list->config_ptr[i].value;
                            ESP_LOGI(TAG, "SET_CONFIG: ISO15765_STMIN_TX=%u (0=use ECU value)", ch->iso15765_stmin_tx);
                            break;
                        case J2534_CONFIG_ISO15765_WFT_MAX:
                            ch->iso15765_wft_max = list->config_ptr[i].value;
                            ESP_LOGI(TAG, "SET_CONFIG: ISO15765_WFT_MAX=%u (0=unlimited)", ch->iso15765_wft_max);
                            break;
                        // KWP2000/ISO9141 timing parameters
                        case J2534_CONFIG_P1_MIN:
                            ch->p1_min = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P1_MAX:
                            ch->p1_max = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P2_MIN:
                            ch->p2_min = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P2_MAX:
                            ch->p2_max = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P3_MIN:
                            ch->p3_min = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P3_MAX:
                            ch->p3_max = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P4_MIN:
                            ch->p4_min = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        case J2534_CONFIG_P4_MAX:
                            ch->p4_max = list->config_ptr[i].value;
                            timing_changed = true;
                            break;
                        default:
                            break;
                    }
                }
                
#if HARDWARE_VER == WICAN_PRO
                // Apply timing to STN chip if legacy protocol
                if (timing_changed && j2534_is_legacy_protocol(ch->protocol_id)) {
                    stn_j2534_set_timing(ch->p1_max, ch->p2_max, ch->p3_max, ch->p4_min);
                }
#endif
            }
            break;

        case J2534_IOCTL_READ_VBATT:
            if (output) {
                *((uint32_t *)output) = 12000;  // 12V placeholder
            }
            break;

        case J2534_IOCTL_CLEAR_TX_BUFFER:
            break;

        case J2534_IOCTL_CLEAR_RX_BUFFER:
            j2534_rx_msg_head = 0;
            j2534_rx_msg_tail = 0;
            break;

        case J2534_IOCTL_CLEAR_MSG_FILTERS:
            if (ch) {
                memset(ch->filters, 0, sizeof(ch->filters));
                ch->filter_count = 0;
            }
            break;

        case J2534_IOCTL_CLEAR_PERIODIC_MSGS:
            {
                int cleared = 0;
                for (int i = 0; i < J2534_MAX_PERIODIC_MSGS_ACTIVE; i++) {
                    if (j2534_periodic_msgs[i].active && j2534_periodic_msgs[i].channel_id == channel_id) {
                        j2534_periodic_msgs[i].active = false;
                        cleared++;
                    }
                }
                ESP_LOGI(TAG, "IOCTL: CLEAR_PERIODIC_MSGS - cleared %d messages for channel %lu", cleared, channel_id);
            }
            break;

        case J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE:
            if (ch) {
                memset(ch->funct_msg_table, 0, sizeof(ch->funct_msg_table));
                ch->funct_msg_count = 0;
                ESP_LOGI(TAG, "IOCTL: CLEAR_FUNCT_MSG_LOOKUP_TABLE - cleared");
            }
            break;

        case J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
            if (ch && input) {
                uint8_t *data = (uint8_t *)input;
                uint32_t num_ids = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                ESP_LOGI(TAG, "IOCTL: ADD_TO_FUNCT_MSG_LOOKUP_TABLE - adding %lu IDs", num_ids);

                for (uint32_t i = 0; i < num_ids && ch->funct_msg_count < J2534_MAX_FUNCT_MSG_IDS; i++) {
                    uint32_t offset = 4 + (i * 4);
                    uint32_t can_id = (data[offset] << 24) | (data[offset+1] << 16) |
                                     (data[offset+2] << 8) | data[offset+3];

                    bool found = false;
                    for (uint32_t j = 0; j < J2534_MAX_FUNCT_MSG_IDS; j++) {
                        if (ch->funct_msg_table[j].active && ch->funct_msg_table[j].can_id == can_id) {
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        for (uint32_t j = 0; j < J2534_MAX_FUNCT_MSG_IDS; j++) {
                            if (!ch->funct_msg_table[j].active) {
                                ch->funct_msg_table[j].can_id = can_id;
                                ch->funct_msg_table[j].active = true;
                                ch->funct_msg_count++;
                                ESP_LOGI(TAG, "  Added functional ID: 0x%lX", can_id);
                                break;
                            }
                        }
                    }
                }
            }
            break;

        case J2534_IOCTL_DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
            if (ch && input) {
                uint8_t *data = (uint8_t *)input;
                uint32_t num_ids = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                ESP_LOGI(TAG, "IOCTL: DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE - removing %lu IDs", num_ids);

                for (uint32_t i = 0; i < num_ids; i++) {
                    uint32_t offset = 4 + (i * 4);
                    uint32_t can_id = (data[offset] << 24) | (data[offset+1] << 16) |
                                     (data[offset+2] << 8) | data[offset+3];

                    for (uint32_t j = 0; j < J2534_MAX_FUNCT_MSG_IDS; j++) {
                        if (ch->funct_msg_table[j].active && ch->funct_msg_table[j].can_id == can_id) {
                            ch->funct_msg_table[j].active = false;
                            ch->funct_msg_count--;
                            ESP_LOGI(TAG, "  Removed functional ID: 0x%lX", can_id);
                            break;
                        }
                    }
                }
            }
            break;

#if HARDWARE_VER == WICAN_PRO
        case J2534_IOCTL_FIVE_BAUD_INIT:
            if (ch && j2534_is_legacy_protocol(ch->protocol_id)) {
                uint8_t target_addr = 0x33;
                uint8_t key_bytes[2] = {0};

                if (input) {
                    uint8_t *data = (uint8_t *)input;
                    if (data[0] >= 1) {
                        target_addr = data[1];
                    }
                }

                ESP_LOGI(TAG, "IOCTL: FIVE_BAUD_INIT to addr 0x%02X", target_addr);

                stn_j2534_status_t status = stn_j2534_five_baud_init(target_addr, key_bytes);

                if (status == STN_J2534_STATUS_OK) {
                    if (output) {
                        uint8_t *out = (uint8_t *)output;
                        out[0] = 2;
                        out[1] = key_bytes[0];
                        out[2] = key_bytes[1];
                    }
                    ESP_LOGI(TAG, "FIVE_BAUD_INIT success: KB1=0x%02X, KB2=0x%02X",
                             key_bytes[0], key_bytes[1]);
                } else {
                    ESP_LOGE(TAG, "FIVE_BAUD_INIT failed: status=%d", status);
                    return J2534_ERR_FAILED;
                }
            } else {
                ESP_LOGW(TAG, "FIVE_BAUD_INIT not supported for this protocol");
                return J2534_ERR_NOT_SUPPORTED;
            }
            break;

        case J2534_IOCTL_FAST_INIT:
            if (ch && (ch->protocol_id == J2534_PROTOCOL_ISO14230 ||
                       ch->protocol_id == J2534_PROTOCOL_ISO14230_PS)) {
                ESP_LOGI(TAG, "IOCTL: FAST_INIT");

                stn_j2534_status_t status = stn_j2534_fast_init();

                if (status == STN_J2534_STATUS_OK) {
                    ESP_LOGI(TAG, "FAST_INIT success");
                } else {
                    ESP_LOGE(TAG, "FAST_INIT failed: status=%d", status);
                    return J2534_ERR_FAILED;
                }
            } else {
                ESP_LOGW(TAG, "FAST_INIT not supported for this protocol");
                return J2534_ERR_NOT_SUPPORTED;
            }
            break;
#endif

        // SWCAN High-Speed Mode (switch from 33.3K to 83.3K)
        case J2534_IOCTL_SW_CAN_HS:
            if (ch && (ch->protocol_id == J2534_PROTOCOL_SW_CAN_PS ||
                       ch->protocol_id == J2534_PROTOCOL_SW_ISO15765_PS)) {
                ESP_LOGI(TAG, "IOCTL: SW_CAN_HS - switching to high-speed mode (83.3K)");
                can_disable();
                can_set_bitrate(CAN_83K);
                can_enable();
            } else {
                ESP_LOGW(TAG, "SW_CAN_HS not applicable for protocol 0x%lX", 
                         ch ? ch->protocol_id : 0);
                return J2534_ERR_NOT_SUPPORTED;
            }
            break;

        // SWCAN Normal-Speed Mode (back to 33.3K)
        case J2534_IOCTL_SW_CAN_NS:
            if (ch && (ch->protocol_id == J2534_PROTOCOL_SW_CAN_PS ||
                       ch->protocol_id == J2534_PROTOCOL_SW_ISO15765_PS)) {
                ESP_LOGI(TAG, "IOCTL: SW_CAN_NS - switching to normal-speed mode (33.3K)");
                can_disable();
                can_set_bitrate(CAN_33K);
                can_enable();
            } else {
                ESP_LOGW(TAG, "SW_CAN_NS not applicable for protocol 0x%lX",
                         ch ? ch->protocol_id : 0);
                return J2534_ERR_NOT_SUPPORTED;
            }
            break;

        default:
            // Compatibility: Vendor IOCTLs (0x8000+) return success to avoid crashing apps
            // that probe for device-specific features. Standard IOCTLs still return error.
            if (ioctl_id >= 0x8000) {
                ESP_LOGI(TAG, "Unknown vendor IOCTL 0x%04lX - returning success (compatibility)", ioctl_id);
                return J2534_STATUS_NOERROR;
            }
            ESP_LOGW(TAG, "Unsupported IOCTL ID: 0x%04lX", ioctl_id);
            j2534_set_error(J2534_ERR_INVALID_IOCTL_ID, "Unsupported IOCTL");
            return J2534_ERR_INVALID_IOCTL_ID;
    }

    return J2534_STATUS_NOERROR;
}
