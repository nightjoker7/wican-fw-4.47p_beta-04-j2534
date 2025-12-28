/**
 * @file j2534_api.h
 * @brief SAE J2534-1 Pass-Thru API Definitions
 * 
 * Standard J2534 API header for Windows DLL driver
 * Compatible with SAE J2534-1 specification
 */

#ifndef J2534_API_H
#define J2534_API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Prevent old winsock.h from being included by windows.h */
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdint.h>

/* ============================================================================
 * J2534 API Version
 * ============================================================================ */
#define J2534_API_VERSION           "04.04"

/* ============================================================================
 * Protocol IDs (Table 2 - SAE J2534-1)
 * ============================================================================ */
#define J1850VPW                    0x01
#define J1850PWM                    0x02
#define ISO9141                     0x03
#define ISO14230                    0x04
#define CAN                         0x05
#define ISO15765                    0x06
#define SCI_A_ENGINE                0x07
#define SCI_A_TRANS                 0x08
#define SCI_B_ENGINE                0x09
#define SCI_B_TRANS                 0x0A

/* ============================================================================
 * J2534-2 Extended Protocol IDs (GM, Ford, etc.)
 * ============================================================================ */
#define J1850VPW_PS                 0x8000
#define J1850PWM_PS                 0x8001
#define ISO9141_PS                  0x8002
#define ISO14230_PS                 0x8003
#define CAN_PS                      0x8004
#define ISO15765_PS                 0x8005
#define J2610_PS                    0x8006
#define SW_ISO15765_PS              0x8007
#define SW_CAN_PS                   0x8008  /* Single-Wire CAN */
#define GM_UART_PS                  0x8009
#define CAN_CH1                     0x9000
#define CAN_CH2                     0x9001
#define J2610                       0x800B  /* Chrysler CCD */
#define ISO15765_LOGIC              0x800C
#define ANALOG_IN_1                 0x800D
#define ANALOG_IN_2                 0x800E

/* ============================================================================
 * Error Codes (Table 3)
 * ============================================================================ */
#define STATUS_NOERROR              0x00
#define ERR_NOT_SUPPORTED           0x01
#define ERR_INVALID_CHANNEL_ID      0x02
#define ERR_INVALID_PROTOCOL_ID     0x03
#define ERR_NULL_PARAMETER          0x04
#define ERR_INVALID_IOCTL_VALUE     0x05
#define ERR_INVALID_FLAGS           0x06
#define ERR_FAILED                  0x07
#define ERR_DEVICE_NOT_CONNECTED    0x08
#define ERR_TIMEOUT                 0x09
#define ERR_INVALID_MSG             0x0A
#define ERR_INVALID_TIME_INTERVAL   0x0B
#define ERR_EXCEEDED_LIMIT          0x0C
#define ERR_INVALID_MSG_ID          0x0D
#define ERR_DEVICE_IN_USE           0x0E
#define ERR_INVALID_IOCTL_ID        0x0F
#define ERR_BUFFER_EMPTY            0x10
#define ERR_BUFFER_FULL             0x11
#define ERR_BUFFER_OVERFLOW         0x12
#define ERR_PIN_INVALID             0x13
#define ERR_CHANNEL_IN_USE          0x14
#define ERR_MSG_PROTOCOL_ID         0x15
#define ERR_INVALID_FILTER_ID       0x16
#define ERR_NO_FLOW_CONTROL         0x17
#define ERR_NOT_UNIQUE              0x18
#define ERR_INVALID_BAUDRATE        0x19
#define ERR_INVALID_DEVICE_ID       0x1A

/* ============================================================================
 * IOCTL IDs (Table 4)
 * ============================================================================ */
#define GET_CONFIG                  0x01
#define SET_CONFIG                  0x02
#define READ_VBATT                  0x03
#define FIVE_BAUD_INIT              0x04
#define FAST_INIT                   0x05
#define CLEAR_TX_BUFFER             0x07
#define CLEAR_RX_BUFFER             0x08
#define CLEAR_PERIODIC_MSGS         0x09
#define CLEAR_MSG_FILTERS           0x0A
#define CLEAR_FUNCT_MSG_LOOKUP_TABLE 0x0B
#define ADD_TO_FUNCT_MSG_LOOKUP_TABLE 0x0C
#define DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE 0x0D
#define READ_PROG_VOLTAGE           0x0E

/* ============================================================================
 * Configuration Parameter IDs (Table 5)
 * ============================================================================ */
#define DATA_RATE                   0x01
#define LOOPBACK                    0x03
#define NODE_ADDRESS                0x04
#define NETWORK_LINE                0x05
#define P1_MIN                      0x06
#define P1_MAX                      0x07
#define P2_MIN                      0x08
#define P2_MAX                      0x09
#define P3_MIN                      0x0A
#define P3_MAX                      0x0B
#define P4_MIN                      0x0C
#define P4_MAX                      0x0D
#define W1                          0x0E
#define W2                          0x0F
#define W3                          0x10
#define W4                          0x11
#define W5                          0x12
#define TIDLE                       0x13
#define TINIL                       0x14
#define TWUP                        0x15
#define PARITY                      0x16
#define BIT_SAMPLE_POINT            0x17
#define SYNC_JUMP_WIDTH             0x18
#define W0                          0x19
#define T1_MAX                      0x1A
#define T2_MAX                      0x1B
#define T4_MAX                      0x1C
#define T5_MAX                      0x1D
#define ISO15765_BS                 0x1E
#define ISO15765_STMIN              0x1F
#define DATA_BITS                   0x20
#define FIVE_BAUD_MOD               0x21
#define BS_TX                       0x22
#define STMIN_TX                    0x23
#define T3_MAX                      0x24
#define ISO15765_WFT_MAX            0x25
#define CAN_MIXED_FORMAT            0x8000
#define J1962_PINS                  0x8001
#define SW_CAN_HS_DATA_RATE         0x8010
#define SW_CAN_SPEEDCHANGE_ENABLE   0x8011
#define SW_CAN_RES_SWITCH           0x8012
#define ACTIVE_CHANNELS             0x8020
#define SAMPLE_RATE                 0x8021
#define SAMPLES_PER_READING         0x8022
#define READINGS_PER_MSG            0x8023
#define AVERAGING_METHOD            0x8024
#define SAMPLE_RESOLUTION           0x8025
#define INPUT_RANGE_LOW             0x8026
#define INPUT_RANGE_HIGH            0x8027

/* ============================================================================
 * Filter Types (Table 6)
 * ============================================================================ */
#define PASS_FILTER                 0x01
#define BLOCK_FILTER                0x02
#define FLOW_CONTROL_FILTER         0x03

/* ============================================================================
 * Message Flags - RxStatus (Table 7)
 * ============================================================================ */
#define TX_MSG_TYPE                 0x0001
#define START_OF_MESSAGE            0x0002
#define ISO15765_FIRST_FRAME        0x0002
#define ISO15765_EXT_ADDR           0x0080
#define RX_BREAK                    0x0100
#define TX_INDICATION               0x0200
#define ISO15765_PADDING_ERROR      0x0400
#define ISO15765_ADDR_TYPE          0x0080
#define CAN_29BIT_ID                0x0100

/* ============================================================================
 * Message Flags - TxFlags (Table 8)
 * ============================================================================ */
#define ISO15765_FRAME_PAD          0x0040
#define TX_NORMAL_TRANSMIT          0x0000
#define CAN_29BIT_ID_TX             0x0100
#define WAIT_P3_MIN_ONLY            0x0200
#define SW_CAN_HV_TX                0x0400
#define SCI_MODE                    0x400000
#define SCI_TX_VOLTAGE              0x800000

/* ============================================================================
 * Connect Flags (Table 9)
 * ============================================================================ */
#define CAN_29BIT_ID_CONNECT        0x0100
#define ISO9141_NO_CHECKSUM         0x0200
#define CAN_ID_BOTH                 0x0800
#define ISO9141_K_LINE_ONLY         0x1000

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/* PASSTHRU_MSG structure */
#pragma pack(push, 1)
typedef struct {
    unsigned long ProtocolID;       /* Protocol type */
    unsigned long RxStatus;         /* Receive message status */
    unsigned long TxFlags;          /* Transmit message flags */
    unsigned long Timestamp;        /* Receive message timestamp (microseconds) */
    unsigned long DataSize;         /* Number of bytes in Data array */
    unsigned long ExtraDataIndex;   /* Start of extra data in Data array */
    unsigned char Data[4128];       /* Message data */
} PASSTHRU_MSG;
#pragma pack(pop)

/* SCONFIG structure */
typedef struct {
    unsigned long Parameter;        /* Configuration parameter ID */
    unsigned long Value;            /* Configuration parameter value */
} SCONFIG;

/* SCONFIG_LIST structure */
typedef struct {
    unsigned long NumOfParams;      /* Number of SCONFIG items */
    SCONFIG *ConfigPtr;             /* Pointer to SCONFIG array */
} SCONFIG_LIST;

/* SBYTE_ARRAY structure */
typedef struct {
    unsigned long NumOfBytes;       /* Number of bytes in BytePtr */
    unsigned char *BytePtr;         /* Pointer to byte array */
} SBYTE_ARRAY;

/* ============================================================================
 * J2534 API Function Prototypes
 * ============================================================================ */

/**
 * @brief Open a connection to a J2534 device
 * @param pName Device name (optional, can be NULL)
 * @param pDeviceID Pointer to receive device ID
 * @return J2534 error code
 */
long __stdcall PassThruOpen(
    void *pName,
    unsigned long *pDeviceID
);

/**
 * @brief Close a connection to a J2534 device
 * @param DeviceID Device ID from PassThruOpen
 * @return J2534 error code
 */
long __stdcall PassThruClose(
    unsigned long DeviceID
);

/**
 * @brief Connect to a vehicle network protocol
 * @param DeviceID Device ID from PassThruOpen
 * @param ProtocolID Protocol to use
 * @param Flags Connection flags
 * @param BaudRate Baud rate for the protocol
 * @param pChannelID Pointer to receive channel ID
 * @return J2534 error code
 */
long __stdcall PassThruConnect(
    unsigned long DeviceID,
    unsigned long ProtocolID,
    unsigned long Flags,
    unsigned long BaudRate,
    unsigned long *pChannelID
);

/**
 * @brief Disconnect from a vehicle network protocol
 * @param ChannelID Channel ID from PassThruConnect
 * @return J2534 error code
 */
long __stdcall PassThruDisconnect(
    unsigned long ChannelID
);

/**
 * @brief Read messages from receive buffer
 * @param ChannelID Channel ID from PassThruConnect
 * @param pMsg Pointer to message array
 * @param pNumMsgs Pointer to number of messages (in: max, out: actual)
 * @param Timeout Timeout in milliseconds
 * @return J2534 error code
 */
long __stdcall PassThruReadMsgs(
    unsigned long ChannelID,
    PASSTHRU_MSG *pMsg,
    unsigned long *pNumMsgs,
    unsigned long Timeout
);

/**
 * @brief Write messages to transmit buffer
 * @param ChannelID Channel ID from PassThruConnect
 * @param pMsg Pointer to message array
 * @param pNumMsgs Pointer to number of messages (in: to send, out: actual sent)
 * @param Timeout Timeout in milliseconds
 * @return J2534 error code
 */
long __stdcall PassThruWriteMsgs(
    unsigned long ChannelID,
    PASSTHRU_MSG *pMsg,
    unsigned long *pNumMsgs,
    unsigned long Timeout
);

/**
 * @brief Start a periodic message
 * @param ChannelID Channel ID from PassThruConnect
 * @param pMsg Pointer to message to send periodically
 * @param pMsgID Pointer to receive message ID
 * @param TimeInterval Interval in milliseconds
 * @return J2534 error code
 */
long __stdcall PassThruStartPeriodicMsg(
    unsigned long ChannelID,
    PASSTHRU_MSG *pMsg,
    unsigned long *pMsgID,
    unsigned long TimeInterval
);

/**
 * @brief Stop a periodic message
 * @param ChannelID Channel ID from PassThruConnect
 * @param MsgID Message ID from PassThruStartPeriodicMsg
 * @return J2534 error code
 */
long __stdcall PassThruStopPeriodicMsg(
    unsigned long ChannelID,
    unsigned long MsgID
);

/**
 * @brief Start a message filter
 * @param ChannelID Channel ID from PassThruConnect
 * @param FilterType Type of filter
 * @param pMaskMsg Pointer to mask message
 * @param pPatternMsg Pointer to pattern message
 * @param pFlowControlMsg Pointer to flow control message (ISO15765 only)
 * @param pFilterID Pointer to receive filter ID
 * @return J2534 error code
 */
long __stdcall PassThruStartMsgFilter(
    unsigned long ChannelID,
    unsigned long FilterType,
    PASSTHRU_MSG *pMaskMsg,
    PASSTHRU_MSG *pPatternMsg,
    PASSTHRU_MSG *pFlowControlMsg,
    unsigned long *pFilterID
);

/**
 * @brief Stop a message filter
 * @param ChannelID Channel ID from PassThruConnect
 * @param FilterID Filter ID from PassThruStartMsgFilter
 * @return J2534 error code
 */
long __stdcall PassThruStopMsgFilter(
    unsigned long ChannelID,
    unsigned long FilterID
);

/**
 * @brief Set programming voltage
 * @param DeviceID Device ID from PassThruOpen
 * @param PinNumber Pin number
 * @param Voltage Voltage in millivolts
 * @return J2534 error code
 */
long __stdcall PassThruSetProgrammingVoltage(
    unsigned long DeviceID,
    unsigned long PinNumber,
    unsigned long Voltage
);

/**
 * @brief Read version information
 * @param DeviceID Device ID from PassThruOpen
 * @param pFirmwareVersion Firmware version string (80 chars)
 * @param pDllVersion DLL version string (80 chars)
 * @param pApiVersion API version string (80 chars)
 * @return J2534 error code
 */
long __stdcall PassThruReadVersion(
    unsigned long DeviceID,
    char *pFirmwareVersion,
    char *pDllVersion,
    char *pApiVersion
);

/**
 * @brief Get last error description
 * @param pErrorDescription Error description string (80 chars)
 * @return J2534 error code
 */
long __stdcall PassThruGetLastError(
    char *pErrorDescription
);

/**
 * @brief Perform IOCTL operation
 * @param ChannelID Channel ID (or Device ID for some operations)
 * @param IoctlID IOCTL command ID
 * @param pInput Pointer to input data
 * @param pOutput Pointer to output data
 * @return J2534 error code
 */
long __stdcall PassThruIoctl(
    unsigned long ChannelID,
    unsigned long IoctlID,
    void *pInput,
    void *pOutput
);

#ifdef __cplusplus
}
#endif

#endif /* J2534_API_H */
