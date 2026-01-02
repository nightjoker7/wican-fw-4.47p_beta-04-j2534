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
 * @file j2534.h
 * @brief J2534 Pass-Thru Protocol Support
 *
 * This module implements the SAE J2534-1 protocol for vehicle diagnostic
 * communication. It provides a standardized interface for PC applications
 * to communicate with vehicle ECUs through the WiCAN device.
 *
 * Supported protocols:
 * - ISO 15765-4 CAN (11-bit and 29-bit)
 * - ISO 14230-4 KWP2000 (via CAN)
 * - SAE J1850 PWM/VPW (limited support)
 */

#ifndef __J2534_H__
#define __J2534_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * J2534 Error Codes (SAE J2534-1)
 * ============================================================================ */

typedef enum {
    J2534_STATUS_NOERROR            = 0x00,  // Function completed successfully
    J2534_ERR_NOT_SUPPORTED         = 0x01,  // Function not supported
    J2534_ERR_INVALID_CHANNEL_ID    = 0x02,  // Invalid channel ID
    J2534_ERR_INVALID_PROTOCOL_ID   = 0x03,  // Invalid protocol ID
    J2534_ERR_NULL_PARAMETER        = 0x04,  // NULL pointer supplied
    J2534_ERR_INVALID_IOCTL_VALUE   = 0x05,  // Invalid IOCTL value
    J2534_ERR_INVALID_FLAGS         = 0x06,  // Invalid flags parameter
    J2534_ERR_FAILED                = 0x07,  // Undefined error
    J2534_ERR_DEVICE_NOT_CONNECTED  = 0x08,  // Device not connected
    J2534_ERR_TIMEOUT               = 0x09,  // Read/Write timeout
    J2534_ERR_INVALID_MSG           = 0x0A,  // Invalid message structure
    J2534_ERR_INVALID_TIME_INTERVAL = 0x0B,  // Invalid time interval
    J2534_ERR_EXCEEDED_LIMIT        = 0x0C,  // Exceeded limit
    J2534_ERR_INVALID_MSG_ID        = 0x0D,  // Invalid message ID
    J2534_ERR_DEVICE_IN_USE         = 0x0E,  // Device in use
    J2534_ERR_INVALID_IOCTL_ID      = 0x0F,  // Invalid IOCTL ID
    J2534_ERR_BUFFER_EMPTY          = 0x10,  // Buffer is empty
    J2534_ERR_BUFFER_FULL           = 0x11,  // Buffer is full
    J2534_ERR_BUFFER_OVERFLOW       = 0x12,  // Buffer overflow
    J2534_ERR_PIN_INVALID           = 0x13,  // Invalid pin number
    J2534_ERR_CHANNEL_IN_USE        = 0x14,  // Channel in use
    J2534_ERR_MSG_PROTOCOL_ID       = 0x15,  // Message protocol ID mismatch
    J2534_ERR_INVALID_FILTER_ID     = 0x16,  // Invalid filter ID
    J2534_ERR_NO_FLOW_CONTROL       = 0x17,  // No flow control
    J2534_ERR_NOT_UNIQUE            = 0x18,  // Not unique
    J2534_ERR_INVALID_BAUDRATE      = 0x19,  // Invalid baud rate
    J2534_ERR_INVALID_DEVICE_ID     = 0x1A,  // Invalid device ID
} j2534_error_t;

/* ============================================================================
 * J2534 Protocol IDs (SAE J2534-1)
 * ============================================================================ */

typedef enum {
    J2534_PROTOCOL_J1850VPW         = 0x01,  // J1850 VPW
    J2534_PROTOCOL_J1850PWM         = 0x02,  // J1850 PWM
    J2534_PROTOCOL_ISO9141          = 0x03,  // ISO 9141
    J2534_PROTOCOL_ISO14230         = 0x04,  // ISO 14230 (KWP2000)
    J2534_PROTOCOL_CAN              = 0x05,  // CAN (ISO 11898)
    J2534_PROTOCOL_ISO15765         = 0x06,  // ISO 15765 (CAN)
    J2534_PROTOCOL_SCI_A_ENGINE     = 0x07,  // SCI A Engine
    J2534_PROTOCOL_SCI_A_TRANS      = 0x08,  // SCI A Transmission
    J2534_PROTOCOL_SCI_B_ENGINE     = 0x09,  // SCI B Engine
    J2534_PROTOCOL_SCI_B_TRANS      = 0x0A,  // SCI B Transmission
    // J2534-2 Extended protocols
    J2534_PROTOCOL_J1850VPW_PS      = 0x8000,
    J2534_PROTOCOL_J1850PWM_PS      = 0x8001,
    J2534_PROTOCOL_ISO9141_PS       = 0x8002,
    J2534_PROTOCOL_ISO14230_PS      = 0x8003,
    J2534_PROTOCOL_CAN_PS           = 0x8004,
    J2534_PROTOCOL_ISO15765_PS      = 0x8005,
    J2534_PROTOCOL_J2610_PS         = 0x8006,
    J2534_PROTOCOL_SW_ISO15765_PS   = 0x8007,
    J2534_PROTOCOL_SW_CAN_PS        = 0x8008,
    J2534_PROTOCOL_GM_UART_PS       = 0x8009,
    J2534_PROTOCOL_CAN_CH1          = 0x9000,
    J2534_PROTOCOL_CAN_CH2          = 0x9001,
} j2534_protocol_t;

/* ============================================================================
 * J2534 IOCTL IDs (SAE J2534-1)
 * ============================================================================ */

typedef enum {
    J2534_IOCTL_GET_CONFIG          = 0x01,  // Get configuration
    J2534_IOCTL_SET_CONFIG          = 0x02,  // Set configuration
    J2534_IOCTL_READ_VBATT          = 0x03,  // Read battery voltage
    J2534_IOCTL_FIVE_BAUD_INIT      = 0x04,  // 5-baud initialization
    J2534_IOCTL_FAST_INIT           = 0x05,  // Fast initialization
    J2534_IOCTL_CLEAR_TX_BUFFER     = 0x07,  // Clear TX buffer
    J2534_IOCTL_CLEAR_RX_BUFFER     = 0x08,  // Clear RX buffer
    J2534_IOCTL_CLEAR_PERIODIC_MSGS = 0x09,  // Clear periodic messages
    J2534_IOCTL_CLEAR_MSG_FILTERS   = 0x0A,  // Clear message filters
    J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE = 0x0B, // Clear functional msg lookup
    J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE = 0x0C, // Add to functional msg lookup
    J2534_IOCTL_DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE = 0x0D, // Delete from functional msg lookup
    J2534_IOCTL_READ_PROG_VOLTAGE   = 0x0E,  // Read programming voltage
    // J2534-2 Extended IOCTLs
    J2534_IOCTL_SW_CAN_HS           = 0x8000, // Switch to high-speed CAN
    J2534_IOCTL_SW_CAN_NS           = 0x8001, // Switch to normal-speed CAN
    J2534_IOCTL_SET_POLL_RESPONSE   = 0x8002, // Set poll response
    J2534_IOCTL_BECOME_MASTER       = 0x8003, // Become master
} j2534_ioctl_t;

/* ============================================================================
 * J2534 Configuration Parameters
 * ============================================================================ */

typedef enum {
    J2534_CONFIG_DATA_RATE          = 0x01,  // Data rate (baud)
    J2534_CONFIG_LOOPBACK           = 0x03,  // Loopback mode
    J2534_CONFIG_NODE_ADDRESS       = 0x04,  // Node address (ISO9141/14230)
    J2534_CONFIG_NETWORK_LINE       = 0x05,  // Network line
    J2534_CONFIG_P1_MIN             = 0x06,  // P1 minimum time
    J2534_CONFIG_P1_MAX             = 0x07,  // P1 maximum time
    J2534_CONFIG_P2_MIN             = 0x08,  // P2 minimum time
    J2534_CONFIG_P2_MAX             = 0x09,  // P2 maximum time
    J2534_CONFIG_P3_MIN             = 0x0A,  // P3 minimum time
    J2534_CONFIG_P3_MAX             = 0x0B,  // P3 maximum time
    J2534_CONFIG_P4_MIN             = 0x0C,  // P4 minimum time
    J2534_CONFIG_P4_MAX             = 0x0D,  // P4 maximum time
    J2534_CONFIG_W0                 = 0x19,  // W0 time (ISO9141)
    J2534_CONFIG_W1                 = 0x0E,  // W1 time
    J2534_CONFIG_W2                 = 0x0F,  // W2 time
    J2534_CONFIG_W3                 = 0x10,  // W3 time
    J2534_CONFIG_W4                 = 0x11,  // W4 time
    J2534_CONFIG_W5                 = 0x12,  // W5 time
    J2534_CONFIG_TIDLE              = 0x13,  // Tidle time
    J2534_CONFIG_TINIL              = 0x14,  // Tinil time
    J2534_CONFIG_TWUP               = 0x15,  // Twup time
    J2534_CONFIG_PARITY             = 0x16,  // Parity
    J2534_CONFIG_BIT_SAMPLE_POINT   = 0x17,  // Bit sample point
    J2534_CONFIG_SYNC_JUMP_WIDTH    = 0x18,  // Synchronization jump width
    J2534_CONFIG_T1_MAX             = 0x1A,  // T1 maximum
    J2534_CONFIG_T2_MAX             = 0x1B,  // T2 maximum
    J2534_CONFIG_T3_MAX             = 0x24,  // T3 maximum
    J2534_CONFIG_T4_MAX             = 0x1C,  // T4 maximum
    J2534_CONFIG_T5_MAX             = 0x1D,  // T5 maximum
    J2534_CONFIG_ISO15765_BS        = 0x1E,  // ISO15765 block size
    J2534_CONFIG_ISO15765_STMIN     = 0x1F,  // ISO15765 separation time min
    J2534_CONFIG_DATA_BITS          = 0x20,  // Data bits
    J2534_CONFIG_ISO15765_BS_TX     = 0x22,  // ISO15765 TX block size
    J2534_CONFIG_ISO15765_STMIN_TX  = 0x23,  // ISO15765 TX STmin
    J2534_CONFIG_ISO15765_WFT_MAX   = 0x25,  // ISO15765 wait frame max
    J2534_CONFIG_CAN_MIXED_FORMAT   = 0x8000, // CAN mixed format
    J2534_CONFIG_J1962_PINS         = 0x8001, // J1962 pins
    J2534_CONFIG_SW_CAN_HS_DATA_RATE = 0x8010, // SWCAN HS data rate
    J2534_CONFIG_SW_CAN_SPEEDCHANGE_ENABLE = 0x8011, // SWCAN speed change
    J2534_CONFIG_SW_CAN_RES_SWITCH  = 0x8012, // SWCAN resistor switch
} j2534_config_param_t;

/* ============================================================================
 * J2534 Filter Types
 * ============================================================================ */

typedef enum {
    J2534_FILTER_PASS               = 0x01,  // Pass filter
    J2534_FILTER_BLOCK              = 0x02,  // Block filter
    J2534_FILTER_FLOW_CONTROL       = 0x03,  // Flow control filter
} j2534_filter_type_t;

/* ============================================================================
 * J2534 Message Flags
 * ============================================================================ */

#define J2534_TX_MSG_TYPE_MASK      0x0001   // TX message type mask
#define J2534_START_OF_MESSAGE      0x0002   // Start of message
#define J2534_ISO15765_FIRST_FRAME  0x0002   // ISO15765 first frame (same as SOM)
#define J2534_RX_BREAK              0x0004   // Receive break
#define J2534_TX_INDICATION         0x0008   // TX indication
#define J2534_ISO15765_PADDING_OLD  0x0010   // Old/wrong value - kept for reference
#define J2534_ISO15765_FRAME_PAD    0x0040   // ISO15765 frame padding (SAE J2534-1)
#define J2534_ISO15765_PADDING      0x0040   // ISO15765 frame padding (alias)
#define J2534_ISO15765_ADDR_TYPE    0x0080   // ISO15765 address type (29-bit)
#define J2534_CAN_29BIT_ID          0x0100   // CAN 29-bit ID
#define J2534_WAIT_P3_MIN_ONLY      0x0200   // Wait P3 minimum only
#define J2534_SW_CAN_HV_TX          0x0400   // SWCAN high voltage TX
#define J2534_SCI_MODE              0x400000 // SCI mode
#define J2534_SCI_TX_VOLTAGE        0x800000 // SCI TX voltage

/* ============================================================================
 * J2534 Message Structure
 * ============================================================================ */

// Message structure size for queued messages
// Most diagnostic responses fit in 256 bytes (including 4-byte CAN ID header)
// Large ISO-TP messages (programming) use ext_data pointer instead of data[]
#define J2534_MAX_MSG_DATA_SIZE     256

typedef struct {
    uint32_t protocol_id;                    // Protocol ID
    uint32_t rx_status;                      // Receive status flags
    uint32_t tx_flags;                       // Transmit flags
    uint32_t timestamp;                      // Timestamp (microseconds)
    uint32_t data_size;                      // Data size (actual size, may be > 256)
    uint32_t extra_data_index;               // Extra data index
    uint8_t *ext_data;                       // Pointer to external data for large TX (NULL if using data[])
    uint8_t data[J2534_MAX_MSG_DATA_SIZE];   // Message data (used when ext_data is NULL)
} j2534_msg_t;

/* ============================================================================
 * J2534 Configuration Structure
 * ============================================================================ */

typedef struct {
    uint32_t parameter;                      // Configuration parameter ID
    uint32_t value;                          // Configuration value
} j2534_sconfig_t;

typedef struct {
    uint32_t num_of_params;                  // Number of parameters
    j2534_sconfig_t *config_ptr;             // Pointer to config array
} j2534_sconfig_list_t;

/* ============================================================================
 * J2534 Filter Structure
 * ============================================================================ */

typedef struct {
    uint32_t filter_id;                      // Filter ID
    uint32_t filter_type;                    // Filter type (pass/block/flow)
    uint32_t protocol_id;                    // Protocol ID
    uint8_t mask[4];                         // Mask pattern
    uint8_t pattern[4];                      // Pattern to match
    uint8_t flow_control[4];                 // Flow control ID (for ISO15765)
    bool active;                             // Filter active flag
} j2534_filter_t;

/* ============================================================================
 * J2534 Functional Message Lookup Table
 * Used for functional addressing (broadcast to 0x7DF, receive from 0x7E8-0x7EF)
 * ============================================================================ */

#define J2534_MAX_FUNCT_MSG_IDS     16       // Maximum functional message IDs

typedef struct {
    uint32_t can_id;                         // CAN ID to accept
    bool active;                             // Entry active flag
} j2534_funct_msg_entry_t;

/* ============================================================================
 * J2534 Channel Structure
 * ============================================================================ */

#define J2534_MAX_CHANNELS          4        // Maximum number of channels
#define J2534_MAX_FILTERS           10       // Maximum filters per channel
#define J2534_MAX_PERIODIC_MSGS     10       // Maximum periodic messages

typedef struct {
    uint32_t channel_id;                     // Channel ID
    uint32_t protocol_id;                    // Protocol ID
    uint32_t flags;                          // Channel flags
    uint32_t baudrate;                       // Baud rate
    bool active;                             // Channel active flag
    bool loopback;                           // Loopback mode
    j2534_filter_t filters[J2534_MAX_FILTERS]; // Message filters
    uint32_t filter_count;                   // Active filter count
    // Functional message lookup table (for broadcast addressing)
    j2534_funct_msg_entry_t funct_msg_table[J2534_MAX_FUNCT_MSG_IDS];
    uint32_t funct_msg_count;                // Active functional msg count
    // ISO15765 specific
    uint8_t iso15765_bs;                     // Block size
    uint8_t iso15765_stmin;                  // Separation time min
    bool iso15765_ext_addr;                  // Extended addressing mode (first byte = address)
    uint8_t iso15765_ext_addr_byte;          // Extended address byte when in ext mode
    // Timing parameters
    uint32_t p1_min;
    uint32_t p1_max;
    uint32_t p2_min;
    uint32_t p2_max;
    uint32_t p3_min;
    uint32_t p3_max;
    uint32_t p4_min;
    uint32_t p4_max;
} j2534_channel_t;

/* ============================================================================
 * J2534 Binary Protocol Commands (for TCP/WiFi communication)
 * ============================================================================ */

// Command opcodes (sent from PC to device)
typedef enum {
    J2534_CMD_CONNECT               = 0x01,  // PassThruConnect
    J2534_CMD_DISCONNECT            = 0x02,  // PassThruDisconnect
    J2534_CMD_READ_MSGS             = 0x03,  // PassThruReadMsgs
    J2534_CMD_WRITE_MSGS            = 0x04,  // PassThruWriteMsgs
    J2534_CMD_START_PERIODIC_MSG    = 0x05,  // PassThruStartPeriodicMsg
    J2534_CMD_STOP_PERIODIC_MSG     = 0x06,  // PassThruStopPeriodicMsg
    J2534_CMD_START_MSG_FILTER      = 0x07,  // PassThruStartMsgFilter
    J2534_CMD_STOP_MSG_FILTER       = 0x08,  // PassThruStopMsgFilter
    J2534_CMD_SET_PROGRAMMING_VOLTAGE = 0x09, // PassThruSetProgrammingVoltage
    J2534_CMD_READ_VERSION          = 0x0A,  // PassThruReadVersion
    J2534_CMD_GET_LAST_ERROR        = 0x0B,  // PassThruGetLastError
    J2534_CMD_IOCTL                 = 0x0C,  // PassThruIoctl
    J2534_CMD_OPEN                  = 0x0D,  // PassThruOpen
    J2534_CMD_CLOSE                 = 0x0E,  // PassThruClose
} j2534_cmd_t;

/* ============================================================================
 * J2534 Binary Protocol Packet Structure
 *
 * Request packet format:
 *   [SYNC1][SYNC2][CMD][LEN_H][LEN_L][DATA...][CHECKSUM]
 *
 * Response packet format:
 *   [SYNC1][SYNC2][CMD|0x80][STATUS][LEN_H][LEN_L][DATA...][CHECKSUM]
 * ============================================================================ */

#define J2534_SYNC1                 0x55
#define J2534_SYNC2                 0xAA
#define J2534_RESPONSE_FLAG         0x80
// Packet buffer size - must fit largest single WriteMsgs packet
// ECU programming sends 4KB+ Transfer Data (0x36) in single J2534 packet
// WiCAN then segments into ISO-TP frames (FF + CFs)
// Need: 4098 data + 20 header + 10 packet overhead = ~4150 bytes
#define J2534_MAX_PACKET_SIZE       4500

typedef struct {
    uint8_t sync1;                           // Sync byte 1 (0x55)
    uint8_t sync2;                           // Sync byte 2 (0xAA)
    uint8_t command;                         // Command opcode
    uint16_t length;                         // Data length
    uint8_t *data;                           // Packet data
    uint8_t checksum;                        // XOR checksum
} j2534_packet_t;

/* ============================================================================
 * J2534 Device Information
 * ============================================================================ */

#define J2534_FW_VERSION            "04.47"  // Firmware version
#define J2534_DLL_VERSION           "04.47"  // DLL version (for compatibility)
#define J2534_API_VERSION           "04.04"  // J2534-1 API version
#define J2534_DEVICE_NAME           "WiCAN"  // Device name

/* ============================================================================
 * J2534 API Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize the J2534 module
 * @param send_callback Callback function to send data to host
 * @param rx_queue Queue for receiving CAN messages
 */
void j2534_init(void (*send_callback)(char*, uint32_t, QueueHandle_t *q, char* cmd_str),
                QueueHandle_t *rx_queue);

/**
 * @brief Process incoming J2534 data from host
 * @param buf Input buffer
 * @param len Buffer length
 * @param frame CAN frame structure for TX operations
 * @param q Response queue
 * @return Processed command type or error
 */
int8_t j2534_process(uint8_t *buf, uint32_t len, twai_message_t *frame, QueueHandle_t *q);

/**
 * @brief Convert received CAN frame to J2534 message and send to host
 * @param frame Received CAN frame
 * @param output_buf Output buffer
 * @return Length of formatted message
 */
int32_t j2534_can_to_msg(twai_message_t *frame, uint8_t *output_buf);

/**
 * @brief Open a J2534 device connection
 * @param device_id Pointer to store device ID
 * @return J2534 error code
 */
j2534_error_t j2534_open(uint32_t *device_id);

/**
 * @brief Close a J2534 device connection
 * @param device_id Device ID to close
 * @return J2534 error code
 */
j2534_error_t j2534_close(uint32_t device_id);

/**
 * @brief Connect to a protocol channel
 * @param device_id Device ID
 * @param protocol_id Protocol to connect
 * @param flags Connection flags
 * @param baudrate Baud rate
 * @param channel_id Pointer to store channel ID
 * @return J2534 error code
 */
j2534_error_t j2534_connect(uint32_t device_id, uint32_t protocol_id,
                            uint32_t flags, uint32_t baudrate,
                            uint32_t *channel_id);

/**
 * @brief Disconnect from a protocol channel
 * @param channel_id Channel ID to disconnect
 * @return J2534 error code
 */
j2534_error_t j2534_disconnect(uint32_t channel_id);

/**
 * @brief Read messages from a channel
 * @param channel_id Channel ID
 * @param msgs Array of message structures
 * @param num_msgs Pointer to number of messages (in/out)
 * @param timeout Timeout in milliseconds
 * @return J2534 error code
 */
j2534_error_t j2534_read_msgs(uint32_t channel_id, j2534_msg_t *msgs,
                              uint32_t *num_msgs, uint32_t timeout);

/**
 * @brief Write messages to a channel
 * @param channel_id Channel ID
 * @param msgs Array of message structures
 * @param num_msgs Pointer to number of messages (in/out)
 * @param timeout Timeout in milliseconds
 * @return J2534 error code
 */
j2534_error_t j2534_write_msgs(uint32_t channel_id, j2534_msg_t *msgs,
                               uint32_t *num_msgs, uint32_t timeout);

/**
 * @brief Start a message filter
 * @param channel_id Channel ID
 * @param filter_type Filter type
 * @param mask_msg Mask message
 * @param pattern_msg Pattern message
 * @param flow_control_msg Flow control message (for ISO15765)
 * @param filter_id Pointer to store filter ID
 * @return J2534 error code
 */
j2534_error_t j2534_start_msg_filter(uint32_t channel_id,
                                     uint32_t filter_type,
                                     j2534_msg_t *mask_msg,
                                     j2534_msg_t *pattern_msg,
                                     j2534_msg_t *flow_control_msg,
                                     uint32_t *filter_id);

/**
 * @brief Stop a message filter
 * @param channel_id Channel ID
 * @param filter_id Filter ID to stop
 * @return J2534 error code
 */
j2534_error_t j2534_stop_msg_filter(uint32_t channel_id, uint32_t filter_id);

/**
 * @brief Execute an IOCTL command
 * @param channel_id Channel ID
 * @param ioctl_id IOCTL command ID
 * @param input Input parameter
 * @param output Output parameter
 * @return J2534 error code
 */
j2534_error_t j2534_ioctl(uint32_t channel_id, uint32_t ioctl_id,
                          void *input, void *output);

/**
 * @brief Read device/API version information
 * @param device_id Device ID
 * @param firmware_version Firmware version string (32 bytes)
 * @param dll_version DLL version string (32 bytes)
 * @param api_version API version string (32 bytes)
 * @return J2534 error code
 */
j2534_error_t j2534_read_version(uint32_t device_id,
                                 char *firmware_version,
                                 char *dll_version,
                                 char *api_version);

/**
 * @brief Get last error description
 * @param error_description Error description buffer (80 bytes)
 * @return J2534 error code
 */
j2534_error_t j2534_get_last_error(char *error_description);

/**
 * @brief Check if J2534 channel is active
 * @return true if active, false otherwise
 */
bool j2534_is_active(void);

/**
 * @brief Get current channel protocol
 * @return Protocol ID or 0 if no active channel
 */
uint32_t j2534_get_protocol(void);

/**
 * @brief Reset J2534 state (called on TCP disconnect)
 * 
 * This function resets the device to initial state, closing all channels
 * and marking the device as not open. Should be called when the TCP
 * connection to the host is lost to prevent "device in use" errors.
 */
void j2534_reset(void);

/* ============================================================================
 * Callback Registration for Legacy Protocol Support
 *
 * These callbacks allow the main component to register functions that the
 * J2534 component can call for legacy protocol (J1850, ISO9141, etc.) support.
 * This avoids circular dependencies between j2534 and main components.
 * ============================================================================ */

/**
 * @brief Callback type for legacy protocol deinit
 *
 * This callback is called when a legacy protocol channel is disconnected
 * or when connection fails, to clean up STN chip state and re-enable
 * the elm327_read_task.
 */
typedef void (*j2534_legacy_deinit_callback_t)(void);

/**
 * @brief Register callback for legacy protocol deinitialization
 * @param callback Function to call when legacy protocol needs cleanup
 */
void j2534_register_legacy_deinit_callback(j2534_legacy_deinit_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __J2534_H__ */
