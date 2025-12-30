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
 * @file wican_comm.h
 * @brief WiCAN Communication Module
 * 
 * Handles TCP/IP and USB Serial communication with WiCAN Pro device
 */

#ifndef WICAN_COMM_H
#define WICAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Prevent winsock.h from being included */
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Constants
 * ============================================================================ */
#define WICAN_DEFAULT_PORT          3333
#define WICAN_DEFAULT_IP            "192.168.80.1"
#define WICAN_DEFAULT_BAUDRATE      2000000  /* USB serial baud rate */
#define WICAN_MAX_PACKET_SIZE       4200   /* ISO-TP max: 4128 + headers */
#define WICAN_CONNECT_TIMEOUT_MS    5000
#define WICAN_READ_TIMEOUT_MS       1000

/* Robustness settings */
#define WICAN_MAX_CONNECT_RETRIES   3       /* Maximum connection attempts */
#define WICAN_RETRY_DELAY_BASE_MS   500     /* Base delay between retries (exponential backoff) */
#define WICAN_RETRY_DELAY_MAX_MS    4000    /* Maximum retry delay */
#define WICAN_HEARTBEAT_INTERVAL_MS 5000    /* Heartbeat check interval */
#define WICAN_HEARTBEAT_TIMEOUT_MS  2000    /* Heartbeat response timeout */
#define WICAN_MAX_RESYNC_ATTEMPTS   5       /* Max attempts to resync protocol after error */
#define WICAN_RECONNECT_COOLDOWN_MS 1000    /* Cooldown between reconnection attempts */

/* Transport types */
#define WICAN_TRANSPORT_TCP         0
#define WICAN_TRANSPORT_USB         1

/* Protocol sync bytes */
#define WICAN_SYNC_BYTE1            0x55
#define WICAN_SYNC_BYTE2            0xAA

/* ============================================================================
 * WiCAN J2534 Command IDs (must match firmware j2534.h)
 * ============================================================================ */
#define WICAN_CMD_CONNECT           0x01  // PassThruConnect
#define WICAN_CMD_DISCONNECT        0x02  // PassThruDisconnect
#define WICAN_CMD_READ_MSGS         0x03  // PassThruReadMsgs
#define WICAN_CMD_WRITE_MSGS        0x04  // PassThruWriteMsgs
#define WICAN_CMD_START_PERIODIC    0x05  // PassThruStartPeriodicMsg
#define WICAN_CMD_STOP_PERIODIC     0x06  // PassThruStopPeriodicMsg
#define WICAN_CMD_START_FILTER      0x07  // PassThruStartMsgFilter
#define WICAN_CMD_STOP_FILTER       0x08  // PassThruStopMsgFilter
#define WICAN_CMD_SET_PROG_VOLTAGE  0x09  // PassThruSetProgrammingVoltage
#define WICAN_CMD_READ_VERSION      0x0A  // PassThruReadVersion
#define WICAN_CMD_GET_LAST_ERROR    0x0B  // PassThruGetLastError
#define WICAN_CMD_IOCTL             0x0C  // PassThruIoctl
#define WICAN_CMD_OPEN              0x0D  // PassThruOpen
#define WICAN_CMD_CLOSE             0x0E  // PassThruClose
#define WICAN_CMD_WRITE_MSGS_BATCH  0x0F  // PassThruWriteMsgs batch mode for raw CAN
#define WICAN_CMD_RESPONSE          0x80  // Response flag

/* Response status codes (J2534 error codes) */
#define WICAN_RESP_OK               0x00  // STATUS_NOERROR
#define WICAN_RESP_NOT_SUPPORTED    0x01  // ERR_NOT_SUPPORTED
#define WICAN_RESP_INVALID_CHANNEL  0x02  // ERR_INVALID_CHANNEL_ID
#define WICAN_RESP_INVALID_PROTOCOL 0x03  // ERR_INVALID_PROTOCOL_ID
#define WICAN_RESP_NULL_PARAM       0x04  // ERR_NULL_PARAMETER
#define WICAN_RESP_INVALID_IOCTL    0x05  // ERR_INVALID_IOCTL_VALUE
#define WICAN_RESP_INVALID_FLAGS    0x06  // ERR_INVALID_FLAGS
#define WICAN_RESP_FAILED           0x07  // ERR_FAILED
#define WICAN_RESP_DEVICE_NOT_CONN  0x08  // ERR_DEVICE_NOT_CONNECTED
#define WICAN_RESP_TIMEOUT          0x09  // ERR_TIMEOUT
#define WICAN_RESP_INVALID_MSG      0x0A  // ERR_INVALID_MSG
#define WICAN_RESP_BUFFER_EMPTY     0x10  // ERR_BUFFER_EMPTY
#define WICAN_RESP_DEVICE_IN_USE    0x14  // ERR_DEVICE_IN_USE

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/* WiCAN connection context */
typedef struct {
    /* Transport type */
    uint8_t transport_type;       /* WICAN_TRANSPORT_TCP or WICAN_TRANSPORT_USB */
    
    /* TCP connection */
    SOCKET socket;
    char ip_address[64];
    uint16_t port;
    
    /* USB Serial connection */
    HANDLE hSerial;
    char com_port[16];            /* e.g., "COM12" */
    uint32_t serial_baudrate;
    
    /* Common */
    bool connected;
    CRITICAL_SECTION cs_socket;
    uint32_t device_id;
    
    /* Robustness - connection state */
    uint32_t connect_attempts;    /* Number of connection attempts made */
    uint32_t reconnect_count;     /* Total reconnections since init */
    DWORD last_activity_time;     /* GetTickCount() of last successful I/O */
    DWORD last_reconnect_time;    /* GetTickCount() of last reconnect attempt */
    bool auto_reconnect;          /* Enable automatic reconnection */
    
    /* Robustness - error tracking */
    uint32_t consecutive_errors;  /* Consecutive communication errors */
    uint32_t total_errors;        /* Total errors since init */
    uint32_t resync_count;        /* Number of protocol resyncs performed */
    
    /* Debug logging */
    HANDLE hLogFile;              /* File handle for debug logging (NULL = disabled) */
    bool log_to_file;             /* Enable file logging */
} wican_context_t;

/* WiCAN message structure (matches firmware) */
#pragma pack(push, 1)
typedef struct {
    uint32_t can_id;
    uint16_t data_len;   /* Actual data length (can be up to 4124 for ISO-TP) */
    uint8_t  flags;
    uint32_t timestamp;
    uint32_t rx_status;  /* J2534 RxStatus - includes TX_MSG_TYPE (0x01) for TX echoes */
    uint8_t  data[4128]; /* ISO-TP max: 4 bytes CAN ID + 4124 bytes payload */
} wican_can_msg_t;

typedef struct {
    uint8_t  sync1;
    uint8_t  sync2;
    uint8_t  cmd;
    uint16_t length;
    uint8_t  data[WICAN_MAX_PACKET_SIZE - 6];
    /* Checksum is at data[length] */
} wican_packet_t;
#pragma pack(pop)

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize WiCAN communication module
 * @return true on success
 */
bool wican_init(void);

/**
 * @brief Cleanup WiCAN communication module
 */
void wican_cleanup(void);

/**
 * @brief Connect to WiCAN device via TCP
 * @param ctx Pointer to context structure
 * @param ip_address IP address of device (NULL for default)
 * @param port Port number (0 for default)
 * @return true on success
 */
bool wican_connect(wican_context_t *ctx, const char *ip_address, uint16_t port);

/**
 * @brief Connect to WiCAN device via USB Serial
 * @param ctx Pointer to context structure
 * @param com_port COM port name (e.g., "COM12") or NULL for auto-detect
 * @param baudrate Baud rate (0 for default 2000000)
 * @return true on success
 */
bool wican_connect_usb(wican_context_t *ctx, const char *com_port, uint32_t baudrate);

/**
 * @brief Auto-detect and connect to WiCAN device (tries USB first, then TCP)
 * @param ctx Pointer to context structure
 * @return true on success
 */
bool wican_connect_auto(wican_context_t *ctx);

/**
 * @brief Find WiCAN USB devices
 * @param ports Array to store COM port names (each 16 chars)
 * @param max_ports Maximum number of ports to find
 * @return Number of WiCAN devices found
 */
int wican_find_usb_devices(char ports[][16], int max_ports);

/**
 * @brief Disconnect from WiCAN device
 * @param ctx Pointer to context structure
 */
void wican_disconnect(wican_context_t *ctx);

/**
 * @brief Check if connected to WiCAN device
 * @param ctx Pointer to context structure
 * @return true if connected
 */
bool wican_is_connected(wican_context_t *ctx);

/**
 * @brief Send command to WiCAN device
 * @param ctx Pointer to context structure
 * @param cmd Command ID
 * @param data Command data
 * @param data_len Length of data
 * @return true on success
 */
bool wican_send_command(wican_context_t *ctx, uint8_t cmd, const uint8_t *data, uint16_t data_len);

/**
 * @brief Receive response from WiCAN device
 * @param ctx Pointer to context structure
 * @param cmd Pointer to receive command ID (response flag stripped)
 * @param status Pointer to receive J2534 status code
 * @param data Buffer for response data
 * @param data_len Pointer to data length (in: max, out: actual)
 * @param timeout_ms Timeout in milliseconds
 * @return true on success
 */
bool wican_receive_response(wican_context_t *ctx, uint8_t *cmd, uint8_t *status, uint8_t *data, uint16_t *data_len, uint32_t timeout_ms);

/**
 * @brief Open J2534 device
 * @param ctx Pointer to context structure
 * @param device_id Pointer to receive device ID
 * @return true on success
 */
bool wican_device_open(wican_context_t *ctx, uint32_t *device_id);

/**
 * @brief Close J2534 device
 * @param ctx Pointer to context structure
 * @param device_id Device ID to close
 * @return true on success
 */
bool wican_device_close(wican_context_t *ctx, uint32_t device_id);

/**
 * @brief Send CAN connect command
 * @param ctx Pointer to context structure
 * @param protocol Protocol ID
 * @param flags Connection flags
 * @param baudrate Baud rate
 * @param channel_id Pointer to receive channel ID from firmware
 * @return true on success
 */
bool wican_can_connect(wican_context_t *ctx, uint32_t protocol, uint32_t flags, 
                       uint32_t baudrate, uint32_t *channel_id);

/**
 * @brief Send CAN disconnect command
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID to disconnect
 * @return true on success
 */
bool wican_can_disconnect(wican_context_t *ctx, uint32_t channel_id);

/**
 * @brief Set configuration parameter
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID
 * @param param_id Parameter ID
 * @param value Parameter value
 * @return true on success
 */
bool wican_set_config(wican_context_t *ctx, uint32_t channel_id, uint32_t param_id, uint32_t value);

/**
 * @brief Read CAN messages
 * @param ctx Pointer to context structure
 * @param msgs Array to receive messages
 * @param max_msgs Maximum number of messages to read
 * @param num_msgs Pointer to receive actual number of messages
 * @param timeout_ms Timeout in milliseconds
 * @return true on success
 */
bool wican_read_messages(wican_context_t *ctx, uint32_t channel_id, wican_can_msg_t *msgs, 
                         uint32_t max_msgs, uint32_t *num_msgs, uint32_t timeout_ms);

/**
 * @brief Write CAN messages
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID from PassThruConnect
 * @param protocol_id Protocol ID (CAN=5, ISO15765=6)
 * @param msgs Array of messages to send
 * @param num_msgs Number of messages to send
 * @param timeout_ms Timeout in milliseconds
 * @param num_sent Pointer to receive number of messages actually sent
 * @return true on success
 */
bool wican_write_messages(wican_context_t *ctx, uint32_t channel_id, uint32_t protocol_id,
                          const wican_can_msg_t *msgs, uint32_t num_msgs, 
                          uint32_t timeout_ms, uint32_t *num_sent);

/**
 * @brief Write multiple raw CAN messages in batch mode
 * 
 * This function sends multiple CAN frames in a single command packet,
 * allowing the firmware to queue them all before transmitting. This is
 * critical for ECU reprogramming where consecutive frames must be sent
 * rapidly without TCP round-trip delays between each frame.
 * 
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID from PassThruConnect
 * @param msgs Array of raw CAN messages (max 12 bytes each: 4-byte ID + 8-byte data)
 * @param num_msgs Number of messages to send
 * @param timeout_ms Timeout in milliseconds
 * @param num_sent Pointer to receive number of messages actually sent
 * @return true on success
 */
bool wican_write_messages_batch(wican_context_t *ctx, uint32_t channel_id,
                                const wican_can_msg_t *msgs, uint32_t num_msgs, 
                                uint32_t timeout_ms, uint32_t *num_sent);

/**
 * @brief Set CAN filter
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID from PassThruConnect
 * @param filter_type Filter type (1=PASS, 2=BLOCK, 3=FLOW_CONTROL)
 * @param mask Filter mask
 * @param pattern Filter pattern
 * @param flow_control Flow control TX ID (for FLOW_CONTROL_FILTER type)
 * @param filter_id Pointer to receive filter ID
 * @return true on success
 */
bool wican_set_filter(wican_context_t *ctx, uint32_t channel_id, uint32_t filter_type, 
                      uint32_t mask, uint32_t pattern, uint32_t flow_control, uint32_t *filter_id);

/**
 * @brief Clear a specific CAN filter
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID
 * @param filter_id Filter ID to clear
 * @return true on success
 */
bool wican_clear_filter(wican_context_t *ctx, uint32_t channel_id, uint32_t filter_id);

/**
 * @brief Set CAN baud rate via IOCTL
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID
 * @param baudrate Baud rate
 * @return true on success
 */
bool wican_set_baudrate(wican_context_t *ctx, uint32_t channel_id, uint32_t baudrate);

/**
 * @brief Get device information
 * @param ctx Pointer to context structure
 * @param fw_version Buffer for firmware version (80 chars)
 * @param hw_version Buffer for hardware version (80 chars)
 * @return true on success
 */
bool wican_get_info(wican_context_t *ctx, char *fw_version, char *hw_version);

/**
 * @brief Calculate XOR checksum
 * @param data Data buffer
 * @param len Data length
 * @return Checksum value
 */
uint8_t wican_calc_checksum(const uint8_t *data, uint16_t len);

/**
 * @brief Send generic IOCTL command to firmware
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID
 * @param ioctl_id IOCTL ID
 * @param input_data Optional input data
 * @param input_len Input data length
 * @return true on success
 */
bool wican_ioctl(wican_context_t *ctx, uint32_t channel_id, uint32_t ioctl_id,
                 const uint8_t *input_data, uint16_t input_len);

/**
 * @brief Start a periodic message on the firmware
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID from PassThruConnect
 * @param msg Message to send periodically
 * @param interval_ms Interval in milliseconds (5-65535)
 * @param msg_id Pointer to receive message ID from firmware
 * @return true on success
 */
bool wican_start_periodic_msg(wican_context_t *ctx, uint32_t channel_id,
                              const wican_can_msg_t *msg, uint32_t interval_ms,
                              uint32_t *msg_id);

/**
 * @brief Stop a periodic message on the firmware
 * @param ctx Pointer to context structure
 * @param channel_id Channel ID from PassThruConnect
 * @param msg_id Message ID returned from wican_start_periodic_msg
 * @return true on success
 */
bool wican_stop_periodic_msg(wican_context_t *ctx, uint32_t channel_id, uint32_t msg_id);

/* ============================================================================
 * Robustness Functions
 * ============================================================================ */

/**
 * @brief Connect with automatic retry and exponential backoff
 * @param ctx Pointer to context structure  
 * @param ip_address IP address (NULL for default/registry)
 * @param port Port (0 for default/registry)
 * @param max_retries Maximum retry attempts (0 = use default)
 * @return true on success
 */
bool wican_connect_with_retry(wican_context_t *ctx, const char *ip_address, 
                               uint16_t port, int max_retries);

/**
 * @brief Enable/disable automatic reconnection on connection loss
 * @param ctx Pointer to context structure
 * @param enable true to enable auto-reconnect
 */
void wican_set_auto_reconnect(wican_context_t *ctx, bool enable);

/**
 * @brief Attempt to reconnect after connection loss
 * @param ctx Pointer to context structure
 * @return true if reconnection successful
 */
bool wican_reconnect(wican_context_t *ctx);

/**
 * @brief Check connection health with heartbeat
 * @param ctx Pointer to context structure
 * @return true if connection is healthy
 */
bool wican_check_connection(wican_context_t *ctx);

/**
 * @brief Attempt to resync protocol after communication error
 * @param ctx Pointer to context structure
 * @return true if resync successful
 */
bool wican_resync(wican_context_t *ctx);

/**
 * @brief Get connection statistics
 * @param ctx Pointer to context structure
 * @param reconnect_count Pointer to receive reconnect count (optional)
 * @param error_count Pointer to receive total error count (optional)
 * @param resync_count Pointer to receive resync count (optional)
 */
void wican_get_stats(wican_context_t *ctx, uint32_t *reconnect_count,
                     uint32_t *error_count, uint32_t *resync_count);

/**
 * @brief Enable debug logging to file
 * @param ctx Pointer to context structure
 * @param log_path Path to log file (NULL to disable)
 * @return true on success
 */
bool wican_enable_logging(wican_context_t *ctx, const char *log_path);

/**
 * @brief Disable debug logging
 * @param ctx Pointer to context structure
 */
void wican_disable_logging(wican_context_t *ctx);

/**
 * @brief Log a message (internal use, but exposed for J2534 layer)
 * @param ctx Pointer to context structure
 * @param format Printf-style format string
 */
void wican_log(wican_context_t *ctx, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* WICAN_COMM_H */
