/**
 * @file wican_comm.h
 * @brief WiCAN TCP Communication Module
 * 
 * Handles TCP/IP communication with WiCAN Pro device
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
#define WICAN_MAX_PACKET_SIZE       4200   /* ISO-TP max: 4128 + headers */
#define WICAN_CONNECT_TIMEOUT_MS    5000
#define WICAN_READ_TIMEOUT_MS       1000

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
    SOCKET socket;
    bool connected;
    char ip_address[64];
    uint16_t port;
    CRITICAL_SECTION cs_socket;
    uint32_t device_id;
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
 * @brief Connect to WiCAN device
 * @param ctx Pointer to context structure
 * @param ip_address IP address of device (NULL for default)
 * @param port Port number (0 for default)
 * @return true on success
 */
bool wican_connect(wican_context_t *ctx, const char *ip_address, uint16_t port);

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

#ifdef __cplusplus
}
#endif

#endif /* WICAN_COMM_H */
