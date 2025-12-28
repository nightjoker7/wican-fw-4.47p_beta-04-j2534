/**
 * @file wican_j2534.c
 * @brief WiCAN J2534 Pass-Thru Driver Implementation
 * 
 * SAE J2534-1 compliant pass-thru driver for WiCAN Pro
 */

#include "j2534_api.h"
#include "wican_comm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * Version Information
 * ============================================================================ */
#define DLL_VERSION         "1.0.0"
#define FIRMWARE_VERSION    "4.47"

/* ============================================================================
 * Constants
 * ============================================================================ */
#define MAX_DEVICES         4
#define MAX_CHANNELS        16
#define MAX_FILTERS         16
#define MAX_PERIODIC_MSGS   16

/* ============================================================================
 * Protocol ID Mapping
 * Maps J2534-2 extended protocols to base CAN/ISO15765 when applicable
 * ============================================================================ */
static uint32_t map_protocol_to_can(uint32_t protocol_id, bool *is_iso15765)
{
    *is_iso15765 = false;
    
    switch (protocol_id) {
        /* Standard J2534-1 protocols */
        case CAN:           /* 0x05 */
            return CAN;
        case ISO15765:      /* 0x06 */
            *is_iso15765 = true;
            return ISO15765;
            
        /* J2534-2 Extended CAN protocols - map to CAN */
        case CAN_PS:        /* 0x8004 */
        case SW_CAN_PS:     /* 0x8008 - GM Single-Wire CAN */
        case CAN_CH1:       /* 0x9000 */
        case CAN_CH2:       /* 0x9001 */
            return CAN;
            
        /* J2534-2 Extended ISO15765 protocols - map to ISO15765 */
        case ISO15765_PS:       /* 0x8005 */
        case SW_ISO15765_PS:    /* 0x8007 */
        case ISO15765_LOGIC:    /* 0x800C */
            *is_iso15765 = true;
            return ISO15765;
            
        /* Unsupported protocols */
        default:
            return 0;
    }
}

/* ============================================================================
 * Data Structures
 * ============================================================================ */

typedef struct {
    bool in_use;
    uint32_t channel_id;
    uint32_t device_id;
    uint32_t protocol_id;
    uint32_t flags;
    uint32_t baudrate;
    bool connected;
    uint32_t fw_channel_id;  /* Channel ID from firmware */
    uint32_t loopback;       /* Loopback enabled */
} j2534_channel_t;

typedef struct {
    bool in_use;
    uint32_t filter_id;
    uint32_t channel_id;
    uint32_t filter_type;
    uint32_t mask;
    uint32_t pattern;
    uint32_t flow_control;  /* Flow control TX ID for ISO15765 */
    uint32_t fw_filter_id;  /* Filter ID from firmware */
} j2534_filter_t;

typedef struct {
    bool in_use;
    uint32_t device_id;
    wican_context_t wican_ctx;
    j2534_channel_t channels[MAX_CHANNELS];
    j2534_filter_t filters[MAX_FILTERS];
    char fw_version[80];
    char hw_version[80];
} j2534_device_t;

/* ============================================================================
 * Periodic Message Structure
 * ============================================================================ */
typedef struct {
    bool active;
    uint32_t msg_id;
    uint32_t channel_id;
    uint32_t interval_ms;
    PASSTHRU_MSG msg;
    HANDLE thread_handle;
    HANDLE stop_event;
} periodic_msg_t;

static periodic_msg_t g_periodic_msgs[MAX_PERIODIC_MSGS];
static uint32_t g_next_periodic_id = 1;

/* ============================================================================
 * Global Variables
 * ============================================================================ */
static j2534_device_t g_devices[MAX_DEVICES];
static char g_last_error[80] = "";
static CRITICAL_SECTION g_cs_driver;
static bool g_driver_initialized = false;
static uint32_t g_next_device_id = 1;
static uint32_t g_next_channel_id = 1;
static uint32_t g_next_filter_id = 1;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void set_error(const char *error)
{
    strncpy(g_last_error, error, sizeof(g_last_error) - 1);
    g_last_error[sizeof(g_last_error) - 1] = '\0';
}

static j2534_device_t* get_device(unsigned long device_id)
{
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (g_devices[i].in_use && g_devices[i].device_id == device_id) {
            return &g_devices[i];
        }
    }
    return NULL;
}

static j2534_channel_t* get_channel(unsigned long channel_id, j2534_device_t **out_device)
{
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!g_devices[i].in_use) continue;
        
        for (int j = 0; j < MAX_CHANNELS; j++) {
            if (g_devices[i].channels[j].in_use && 
                g_devices[i].channels[j].channel_id == channel_id) {
                if (out_device) *out_device = &g_devices[i];
                return &g_devices[i].channels[j];
            }
        }
    }
    return NULL;
}

static j2534_device_t* allocate_device(void)
{
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!g_devices[i].in_use) {
            memset(&g_devices[i], 0, sizeof(j2534_device_t));
            g_devices[i].in_use = true;
            g_devices[i].device_id = g_next_device_id++;
            return &g_devices[i];
        }
    }
    return NULL;
}

static j2534_channel_t* allocate_channel(j2534_device_t *device)
{
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!device->channels[i].in_use) {
            memset(&device->channels[i], 0, sizeof(j2534_channel_t));
            device->channels[i].in_use = true;
            device->channels[i].channel_id = g_next_channel_id++;
            device->channels[i].device_id = device->device_id;
            return &device->channels[i];
        }
    }
    return NULL;
}

static j2534_filter_t* allocate_filter(j2534_device_t *device, uint32_t channel_id)
{
    for (int i = 0; i < MAX_FILTERS; i++) {
        if (!device->filters[i].in_use) {
            memset(&device->filters[i], 0, sizeof(j2534_filter_t));
            device->filters[i].in_use = true;
            device->filters[i].filter_id = g_next_filter_id++;
            device->filters[i].channel_id = channel_id;
            return &device->filters[i];
        }
    }
    return NULL;
}

static void init_driver(void)
{
    if (!g_driver_initialized) {
        InitializeCriticalSection(&g_cs_driver);
        memset(g_devices, 0, sizeof(g_devices));
        memset(g_periodic_msgs, 0, sizeof(g_periodic_msgs));
        wican_init();
        g_driver_initialized = true;
    }
}

/* ============================================================================
 * DLL Entry Point
 * ============================================================================ */

BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            DisableThreadLibraryCalls(hinstDLL);
            init_driver();
            break;
            
        case DLL_PROCESS_DETACH:
            if (g_driver_initialized) {
                for (int i = 0; i < MAX_DEVICES; i++) {
                    if (g_devices[i].in_use) {
                        wican_disconnect(&g_devices[i].wican_ctx);
                    }
                }
                wican_cleanup();
                DeleteCriticalSection(&g_cs_driver);
                g_driver_initialized = false;
            }
            break;
    }
    return TRUE;
}

/* ============================================================================
 * J2534 API Implementation
 * ============================================================================ */

long __stdcall PassThruOpen(void *pName, unsigned long *pDeviceID)
{
    j2534_device_t *device;
    const char *ip_address = NULL;
    uint32_t firmware_device_id = 0;
    int retry_count = 0;
    const int max_retries = 5;  /* Increased for better reliability */
    
    init_driver();
    
    OutputDebugStringA("[J2534] PassThruOpen called\n");
    
    if (!pDeviceID) {
        set_error("Null parameter: pDeviceID");
        return ERR_NULL_PARAMETER;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    if (pName) {
        ip_address = (const char*)pName;
        char dbg[256];
        sprintf(dbg, "[J2534] PassThruOpen: pName='%s'\n", ip_address);
        OutputDebugStringA(dbg);
    }
    
    device = allocate_device();
    if (!device) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("No available device slots");
        OutputDebugStringA("[J2534] PassThruOpen: No available device slots\n");
        return ERR_DEVICE_IN_USE;
    }
    
    /* Retry connection with delays - WiCAN may need time to release previous connection */
    while (retry_count < max_retries) {
        char dbg[128];
        sprintf(dbg, "[J2534] PassThruOpen: connection attempt %d/%d\n", retry_count + 1, max_retries);
        OutputDebugStringA(dbg);
        
        if (wican_connect(&device->wican_ctx, ip_address, 0)) {
            OutputDebugStringA("[J2534] PassThruOpen: connection successful\n");
            break;  /* Connected successfully */
        }
        retry_count++;
        if (retry_count < max_retries) {
            sprintf(dbg, "[J2534] PassThruOpen: connect failed, waiting 1000ms before retry %d/%d\n", retry_count + 1, max_retries);
            OutputDebugStringA(dbg);
            Sleep(1000);  /* Wait 1000ms before retry - increased for ESP32 recovery */
        }
    }
    
    if (retry_count >= max_retries) {
        device->in_use = false;
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to connect to WiCAN device");
        OutputDebugStringA("[J2534] PassThruOpen: Failed to connect after retries\n");
        return ERR_DEVICE_NOT_CONNECTED;
    }
    
    /* Send J2534 OPEN command to firmware */
    if (!wican_device_open(&device->wican_ctx, &firmware_device_id)) {
        wican_disconnect(&device->wican_ctx);
        device->in_use = false;
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to open J2534 device on WiCAN");
        OutputDebugStringA("[J2534] PassThruOpen: wican_device_open failed\n");
        return ERR_FAILED;
    }
    
    wican_get_info(&device->wican_ctx, device->fw_version, device->hw_version);
    
    *pDeviceID = device->device_id;
    
    {
        char dbg[128];
        sprintf(dbg, "[J2534] PassThruOpen: SUCCESS, DeviceID=%lu\n", device->device_id);
        OutputDebugStringA(dbg);
    }
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruClose(unsigned long DeviceID)
{
    j2534_device_t *device;
    
    EnterCriticalSection(&g_cs_driver);
    
    device = get_device(DeviceID);
    if (!device) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid device ID");
        return ERR_INVALID_DEVICE_ID;
    }
    
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (device->channels[i].in_use && device->channels[i].connected) {
            wican_can_disconnect(&device->wican_ctx, device->channels[i].channel_id);
        }
    }
    
    /* Send J2534 CLOSE command to firmware */
    wican_device_close(&device->wican_ctx, device->wican_ctx.device_id);
    
    wican_disconnect(&device->wican_ctx);
    
    device->in_use = false;
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID,
                               unsigned long Flags, unsigned long BaudRate,
                               unsigned long *pChannelID)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    bool is_iso15765 = false;
    uint32_t mapped_protocol;
    
    char dbg[256];
    sprintf(dbg, "[J2534] PassThruConnect: DeviceID=%lu ProtocolID=0x%lX Flags=0x%lX BaudRate=%lu\n",
            DeviceID, ProtocolID, Flags, BaudRate);
    OutputDebugStringA(dbg);
    
    if (!pChannelID) {
        set_error("Null parameter: pChannelID");
        return ERR_NULL_PARAMETER;
    }
    
    /* Map extended protocol IDs to base CAN/ISO15765 */
    mapped_protocol = map_protocol_to_can(ProtocolID, &is_iso15765);
    if (mapped_protocol == 0) {
        sprintf(dbg, "[J2534] PassThruConnect: Unsupported protocol 0x%lX\n", ProtocolID);
        OutputDebugStringA(dbg);
        set_error("Unsupported protocol");
        return ERR_INVALID_PROTOCOL_ID;
    }
    
    sprintf(dbg, "[J2534] PassThruConnect: Mapped protocol 0x%lX -> 0x%lX (ISO15765=%d)\n",
            ProtocolID, mapped_protocol, is_iso15765);
    OutputDebugStringA(dbg);
    
    EnterCriticalSection(&g_cs_driver);
    
    device = get_device(DeviceID);
    if (!device) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid device ID");
        return ERR_INVALID_DEVICE_ID;
    }
    
    channel = allocate_channel(device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("No available channel slots");
        return ERR_EXCEEDED_LIMIT;
    }
    
    uint32_t fw_channel_id = 0;
    if (!wican_can_connect(&device->wican_ctx, ProtocolID, Flags, BaudRate, &fw_channel_id)) {
        channel->in_use = false;
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to connect to CAN bus");
        return ERR_FAILED;
    }
    
    channel->protocol_id = ProtocolID;
    channel->flags = Flags;
    channel->baudrate = BaudRate;
    channel->connected = true;
    channel->fw_channel_id = fw_channel_id;  /* Store firmware's channel ID */
    channel->loopback = 0;
    
    *pChannelID = channel->channel_id;
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruDisconnect(unsigned long ChannelID)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    if (channel->connected) {
        wican_can_disconnect(&device->wican_ctx, channel->fw_channel_id);
    }
    
    for (int i = 0; i < MAX_FILTERS; i++) {
        if (device->filters[i].in_use && device->filters[i].channel_id == ChannelID) {
            device->filters[i].in_use = false;
        }
    }
    
    channel->in_use = false;
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                                unsigned long *pNumMsgs, unsigned long Timeout)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    wican_can_msg_t can_msgs[32];
    uint32_t num_msgs;
    
    if (!pMsg || !pNumMsgs) {
        set_error("Null parameter");
        return ERR_NULL_PARAMETER;
    }
    
    if (*pNumMsgs == 0) {
        return STATUS_NOERROR;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    if (!channel->connected) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Channel not connected");
        return ERR_DEVICE_NOT_CONNECTED;
    }
    
    uint32_t max_msgs = (*pNumMsgs > 32) ? 32 : *pNumMsgs;
    
    if (!wican_read_messages(&device->wican_ctx, channel->fw_channel_id, can_msgs, 
                             max_msgs, &num_msgs, Timeout)) {
        *pNumMsgs = 0;
        LeaveCriticalSection(&g_cs_driver);
        return ERR_BUFFER_EMPTY;
    }
    
    for (uint32_t i = 0; i < num_msgs; i++) {
        memset(&pMsg[i], 0, sizeof(PASSTHRU_MSG));
        pMsg[i].ProtocolID = channel->protocol_id;
        pMsg[i].Timestamp = can_msgs[i].timestamp;
        pMsg[i].DataSize = 4 + can_msgs[i].data_len;
        pMsg[i].ExtraDataIndex = pMsg[i].DataSize;
        
        /* Pass through full RxStatus from firmware - critical for TX_MSG_TYPE (0x01) */
        /* TX echoes have RxStatus=0x01 which diagnostic software needs for subnet discovery */
        pMsg[i].RxStatus = can_msgs[i].rx_status;
        
        pMsg[i].Data[0] = (can_msgs[i].can_id >> 24) & 0xFF;
        pMsg[i].Data[1] = (can_msgs[i].can_id >> 16) & 0xFF;
        pMsg[i].Data[2] = (can_msgs[i].can_id >> 8) & 0xFF;
        pMsg[i].Data[3] = can_msgs[i].can_id & 0xFF;
        
        memcpy(&pMsg[i].Data[4], can_msgs[i].data, can_msgs[i].data_len);
    }
    
    *pNumMsgs = num_msgs;
    
    LeaveCriticalSection(&g_cs_driver);
    
    if (num_msgs == 0) {
        return ERR_BUFFER_EMPTY;
    }
    
    return STATUS_NOERROR;
}

long __stdcall PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                                 unsigned long *pNumMsgs, unsigned long Timeout)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    wican_can_msg_t can_msgs[32];
    
    if (!pMsg || !pNumMsgs) {
        set_error("Null parameter");
        return ERR_NULL_PARAMETER;
    }
    
    if (*pNumMsgs == 0) {
        return STATUS_NOERROR;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    if (!channel->connected) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Channel not connected");
        return ERR_DEVICE_NOT_CONNECTED;
    }
    
    uint32_t num_msgs = (*pNumMsgs > 32) ? 32 : *pNumMsgs;
    
    for (uint32_t i = 0; i < num_msgs; i++) {
        memset(&can_msgs[i], 0, sizeof(wican_can_msg_t));
        
        can_msgs[i].can_id = ((uint32_t)pMsg[i].Data[0] << 24) |
                             ((uint32_t)pMsg[i].Data[1] << 16) |
                             ((uint32_t)pMsg[i].Data[2] << 8) |
                             pMsg[i].Data[3];
        
        /* For ISO-TP, DataSize can be up to 4128 (4 byte CAN ID + 4124 payload) */
        can_msgs[i].data_len = (pMsg[i].DataSize > 4) ? (pMsg[i].DataSize - 4) : 0;
        if (can_msgs[i].data_len > 4124) can_msgs[i].data_len = 4124;  /* ISO-TP max */
        
        if (can_msgs[i].data_len > 0) {
            memcpy(can_msgs[i].data, &pMsg[i].Data[4], can_msgs[i].data_len);
        }
        
        if (pMsg[i].TxFlags & CAN_29BIT_ID_TX) {
            can_msgs[i].flags |= 0x01;
        }
    }
    
    uint32_t num_sent = 0;
    if (!wican_write_messages(&device->wican_ctx, channel->fw_channel_id, 
                              channel->protocol_id, can_msgs, num_msgs, Timeout, &num_sent)) {
        *pNumMsgs = num_sent;
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to write messages");
        return ERR_FAILED;
    }
    
    *pNumMsgs = num_sent;
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                                        unsigned long *pMsgID, unsigned long TimeInterval)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    periodic_msg_t *pmsg = NULL;
    int slot = -1;
    char dbg[256];
    
    sprintf(dbg, "[J2534] PassThruStartPeriodicMsg: ChannelID=%lu Interval=%lu ms\n",
            ChannelID, TimeInterval);
    OutputDebugStringA(dbg);
    
    if (!pMsg || !pMsgID) {
        set_error("Null parameter");
        return ERR_NULL_PARAMETER;
    }
    
    /* J2534 spec: TimeInterval must be between 5ms and 65535ms */
    if (TimeInterval < 5 || TimeInterval > 65535) {
        set_error("Invalid time interval");
        return ERR_INVALID_TIME_INTERVAL;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    /* Find free periodic message slot */
    for (int i = 0; i < MAX_PERIODIC_MSGS; i++) {
        if (!g_periodic_msgs[i].active) {
            slot = i;
            pmsg = &g_periodic_msgs[i];
            break;
        }
    }
    
    if (!pmsg) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("No free periodic message slots");
        return ERR_EXCEEDED_LIMIT;
    }
    
    /* Convert PASSTHRU_MSG to wican_can_msg_t */
    wican_can_msg_t can_msg;
    memset(&can_msg, 0, sizeof(can_msg));
    
    if (pMsg->DataSize >= 4) {
        can_msg.can_id = ((uint32_t)pMsg->Data[0] << 24) |
                         ((uint32_t)pMsg->Data[1] << 16) |
                         ((uint32_t)pMsg->Data[2] << 8) |
                         pMsg->Data[3];
        /* Periodic messages are limited to standard CAN (8 bytes max) */
        can_msg.data_len = (pMsg->DataSize > 4) ? (pMsg->DataSize - 4) : 0;
        if (can_msg.data_len > 8) can_msg.data_len = 8;
        if (can_msg.data_len > 0) {
            memcpy(can_msg.data, &pMsg->Data[4], can_msg.data_len);
        }
    }
    
    if (pMsg->TxFlags & CAN_29BIT_ID_TX) {
        can_msg.flags |= 0x01;
    }
    
    sprintf(dbg, "[J2534] PassThruStartPeriodicMsg: CAN_ID=0x%08lX data_len=%u data=[",
            can_msg.can_id, can_msg.data_len);
    OutputDebugStringA(dbg);
    for (int i = 0; i < can_msg.data_len && i < 8; i++) {
        sprintf(dbg, "%02X ", can_msg.data[i]);
        OutputDebugStringA(dbg);
    }
    OutputDebugStringA("]\n");
    
    /* Send to firmware */
    uint32_t fw_msg_id = 0;
    if (!wican_start_periodic_msg(&device->wican_ctx, channel->fw_channel_id,
                                   &can_msg, TimeInterval, &fw_msg_id)) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to start periodic message on firmware");
        return ERR_FAILED;
    }
    
    /* Store the periodic message info locally with firmware msg_id */
    pmsg->active = true;
    pmsg->msg_id = fw_msg_id;  /* Use firmware's msg_id */
    pmsg->channel_id = ChannelID;
    pmsg->interval_ms = TimeInterval;
    memcpy(&pmsg->msg, pMsg, sizeof(PASSTHRU_MSG));
    pmsg->thread_handle = NULL;
    pmsg->stop_event = NULL;
    
    *pMsgID = pmsg->msg_id;
    
    sprintf(dbg, "[J2534] PassThruStartPeriodicMsg: Started MsgID=%lu (firmware-side)\n",
            pmsg->msg_id);
    OutputDebugStringA(dbg);
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    char dbg[128];
    
    sprintf(dbg, "[J2534] PassThruStopPeriodicMsg: ChannelID=%lu MsgID=%lu\n", ChannelID, MsgID);
    OutputDebugStringA(dbg);
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    for (int i = 0; i < MAX_PERIODIC_MSGS; i++) {
        if (g_periodic_msgs[i].active && 
            g_periodic_msgs[i].msg_id == MsgID &&
            g_periodic_msgs[i].channel_id == ChannelID) {
            
            /* Stop on firmware */
            if (!wican_stop_periodic_msg(&device->wican_ctx, channel->fw_channel_id, MsgID)) {
                OutputDebugStringA("[J2534] PassThruStopPeriodicMsg: firmware stop failed\n");
                /* Continue anyway to clean up local state */
            }
            
            g_periodic_msgs[i].active = false;
            
            LeaveCriticalSection(&g_cs_driver);
            OutputDebugStringA("[J2534] PassThruStopPeriodicMsg: Stopped\n");
            return STATUS_NOERROR;
        }
    }
    
    LeaveCriticalSection(&g_cs_driver);
    set_error("Invalid message ID");
    return ERR_INVALID_MSG_ID;
}

long __stdcall PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType,
                                      PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
                                      PASSTHRU_MSG *pFlowControlMsg, unsigned long *pFilterID)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    j2534_filter_t *filter;
    uint32_t mask = 0, pattern = 0, flow_control = 0;
    uint32_t wican_filter_id;
    char dbg[512];
    
    sprintf(dbg, "[WICAN] PassThruStartMsgFilter: ChannelID=%lu FilterType=%lu\n", 
            ChannelID, FilterType);
    OutputDebugStringA(dbg);
    printf("%s", dbg);
    
    if (pMaskMsg) {
        sprintf(dbg, "[WICAN]   MaskMsg: ProtocolID=%lu DataSize=%lu Data=", 
                pMaskMsg->ProtocolID, pMaskMsg->DataSize);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
        for (unsigned long i = 0; i < pMaskMsg->DataSize && i < 16; i++) {
            sprintf(dbg, "%02X ", pMaskMsg->Data[i]);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
        OutputDebugStringA("\n");
        printf("\n");
    }
    
    if (pPatternMsg) {
        sprintf(dbg, "[WICAN]   PatternMsg: ProtocolID=%lu DataSize=%lu Data=", 
                pPatternMsg->ProtocolID, pPatternMsg->DataSize);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
        for (unsigned long i = 0; i < pPatternMsg->DataSize && i < 16; i++) {
            sprintf(dbg, "%02X ", pPatternMsg->Data[i]);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
        OutputDebugStringA("\n");
        printf("\n");
    }
    
    if (pFlowControlMsg) {
        sprintf(dbg, "[WICAN]   FlowControlMsg: ProtocolID=%lu DataSize=%lu Data=", 
                pFlowControlMsg->ProtocolID, pFlowControlMsg->DataSize);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
        for (unsigned long i = 0; i < pFlowControlMsg->DataSize && i < 16; i++) {
            sprintf(dbg, "%02X ", pFlowControlMsg->Data[i]);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
        OutputDebugStringA("\n");
        printf("\n");
    }
    
    if (!pFilterID) {
        set_error("Null parameter: pFilterID");
        return ERR_NULL_PARAMETER;
    }
    
    if (FilterType != PASS_FILTER && FilterType != BLOCK_FILTER &&
        FilterType != FLOW_CONTROL_FILTER) {
        set_error("Invalid filter type");
        return ERR_INVALID_FLAGS;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    if (pMaskMsg && pMaskMsg->DataSize >= 4) {
        mask = ((uint32_t)pMaskMsg->Data[0] << 24) |
               ((uint32_t)pMaskMsg->Data[1] << 16) |
               ((uint32_t)pMaskMsg->Data[2] << 8) |
               pMaskMsg->Data[3];
    }
    
    if (pPatternMsg && pPatternMsg->DataSize >= 4) {
        pattern = ((uint32_t)pPatternMsg->Data[0] << 24) |
                  ((uint32_t)pPatternMsg->Data[1] << 16) |
                  ((uint32_t)pPatternMsg->Data[2] << 8) |
                  pPatternMsg->Data[3];
    }
    
    /* Extract flow control TX ID for ISO 15765 */
    if (pFlowControlMsg && pFlowControlMsg->DataSize >= 4) {
        flow_control = ((uint32_t)pFlowControlMsg->Data[0] << 24) |
                       ((uint32_t)pFlowControlMsg->Data[1] << 16) |
                       ((uint32_t)pFlowControlMsg->Data[2] << 8) |
                       pFlowControlMsg->Data[3];
        {
            char dbg[128];
            sprintf(dbg, "[WICAN]   FlowControl TX ID: 0x%08lX\n", flow_control);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
    }
    
    filter = allocate_filter(device, ChannelID);
    if (!filter) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("No available filter slots");
        return ERR_EXCEEDED_LIMIT;
    }
    
    if (!wican_set_filter(&device->wican_ctx, channel->fw_channel_id, FilterType, 
                          mask, pattern, flow_control, &wican_filter_id)) {
        filter->in_use = false;
        LeaveCriticalSection(&g_cs_driver);
        set_error("Failed to set filter");
        return ERR_FAILED;
    }
    
    filter->filter_type = FilterType;
    filter->mask = mask;
    filter->pattern = pattern;
    filter->flow_control = flow_control;
    filter->fw_filter_id = wican_filter_id;
    
    *pFilterID = filter->filter_id;
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(ChannelID, &device);
    if (!channel) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid channel ID");
        return ERR_INVALID_CHANNEL_ID;
    }
    
    for (int i = 0; i < MAX_FILTERS; i++) {
        if (device->filters[i].in_use && 
            device->filters[i].filter_id == FilterID &&
            device->filters[i].channel_id == ChannelID) {
            
            /* Call firmware to clear filter */
            wican_clear_filter(&device->wican_ctx, channel->fw_channel_id, 
                               device->filters[i].fw_filter_id);
            
            device->filters[i].in_use = false;
            LeaveCriticalSection(&g_cs_driver);
            return STATUS_NOERROR;
        }
    }
    
    LeaveCriticalSection(&g_cs_driver);
    set_error("Invalid filter ID");
    return ERR_INVALID_FILTER_ID;
}

long __stdcall PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber,
                                             unsigned long Voltage)
{
    set_error("Programming voltage not supported");
    return ERR_NOT_SUPPORTED;
}

long __stdcall PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion,
                                   char *pDllVersion, char *pApiVersion)
{
    j2534_device_t *device;
    
    if (!pFirmwareVersion || !pDllVersion || !pApiVersion) {
        set_error("Null parameter");
        return ERR_NULL_PARAMETER;
    }
    
    EnterCriticalSection(&g_cs_driver);
    
    device = get_device(DeviceID);
    if (!device) {
        LeaveCriticalSection(&g_cs_driver);
        set_error("Invalid device ID");
        return ERR_INVALID_DEVICE_ID;
    }
    
    strncpy(pFirmwareVersion, device->fw_version[0] ? device->fw_version : FIRMWARE_VERSION, 79);
    strncpy(pDllVersion, DLL_VERSION, 79);
    strncpy(pApiVersion, J2534_API_VERSION, 79);
    
    pFirmwareVersion[79] = '\0';
    pDllVersion[79] = '\0';
    pApiVersion[79] = '\0';
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}

long __stdcall PassThruGetLastError(char *pErrorDescription)
{
    if (!pErrorDescription) {
        return ERR_NULL_PARAMETER;
    }
    
    strncpy(pErrorDescription, g_last_error, 79);
    pErrorDescription[79] = '\0';
    
    return STATUS_NOERROR;
}

long __stdcall PassThruIoctl(unsigned long HandleID, unsigned long IoctlID,
                             void *pInput, void *pOutput)
{
    j2534_device_t *device;
    j2534_channel_t *channel;
    
    EnterCriticalSection(&g_cs_driver);
    
    channel = get_channel(HandleID, &device);
    if (!channel) {
        device = get_device(HandleID);
        if (!device) {
            LeaveCriticalSection(&g_cs_driver);
            set_error("Invalid handle ID");
            return ERR_INVALID_CHANNEL_ID;
        }
    }
    
    switch (IoctlID) {
        case GET_CONFIG:
            if (!pInput) {
                LeaveCriticalSection(&g_cs_driver);
                return ERR_NULL_PARAMETER;
            }
            {
                SCONFIG_LIST *cfg_list = (SCONFIG_LIST*)pInput;
                for (unsigned long i = 0; i < cfg_list->NumOfParams; i++) {
                    switch (cfg_list->ConfigPtr[i].Parameter) {
                        case DATA_RATE:
                            if (channel) {
                                cfg_list->ConfigPtr[i].Value = channel->baudrate;
                            }
                            break;
                        case LOOPBACK:
                            if (channel) {
                                cfg_list->ConfigPtr[i].Value = channel->loopback;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
            
        case SET_CONFIG:
            if (!pInput) {
                LeaveCriticalSection(&g_cs_driver);
                return ERR_NULL_PARAMETER;
            }
            {
                SCONFIG_LIST *cfg_list = (SCONFIG_LIST*)pInput;
                for (unsigned long i = 0; i < cfg_list->NumOfParams; i++) {
                    switch (cfg_list->ConfigPtr[i].Parameter) {
                        case DATA_RATE:
                            if (channel && device) {
                                if (wican_set_baudrate(&device->wican_ctx, channel->fw_channel_id, 
                                                       cfg_list->ConfigPtr[i].Value)) {
                                    channel->baudrate = cfg_list->ConfigPtr[i].Value;
                                }
                            }
                            break;
                        case LOOPBACK:
                            /* Store loopback setting locally - firmware handles via CAN frame self flag */
                            if (channel) {
                                channel->loopback = cfg_list->ConfigPtr[i].Value;
                                {
                                    char dbg[128];
                                    sprintf(dbg, "[J2534] SET_CONFIG LOOPBACK=%lu for channel %lu\n",
                                            cfg_list->ConfigPtr[i].Value, channel->channel_id);
                                    OutputDebugStringA(dbg);
                                }
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
            
        case READ_VBATT:
            if (!pOutput) {
                LeaveCriticalSection(&g_cs_driver);
                return ERR_NULL_PARAMETER;
            }
            *(unsigned long*)pOutput = 12000;
            break;
            
        case CLEAR_TX_BUFFER:
        case CLEAR_RX_BUFFER:
            break;
            
        case CLEAR_MSG_FILTERS:
            if (device && channel) {
                /* Clear all filters for this channel */
                for (int i = 0; i < MAX_FILTERS; i++) {
                    if (device->filters[i].in_use && 
                        device->filters[i].channel_id == channel->channel_id) {
                        wican_clear_filter(&device->wican_ctx, channel->fw_channel_id,
                                           device->filters[i].fw_filter_id);
                        device->filters[i].in_use = false;
                    }
                }
            }
            break;
            
        case CLEAR_PERIODIC_MSGS:
            /* Periodic messages not currently implemented, but return success */
            OutputDebugStringA("[J2534] IOCTL: CLEAR_PERIODIC_MSGS (no-op)\n");
            break;
            
        case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
            /* Clear functional message lookup table - send to firmware */
            OutputDebugStringA("[J2534] IOCTL: CLEAR_FUNCT_MSG_LOOKUP_TABLE\n");
            if (device && channel) {
                wican_ioctl(&device->wican_ctx, channel->fw_channel_id, 
                            CLEAR_FUNCT_MSG_LOOKUP_TABLE, NULL, 0);
            }
            break;
            
        case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
            /* Add CAN IDs to functional message lookup table for subnet discovery */
            /* Used for functional addressing (broadcast to 0x7DF) */
            if (pInput && device && channel) {
                SBYTE_ARRAY *pInputArray = (SBYTE_ARRAY *)pInput;
                char dbg[256];
                sprintf(dbg, "[J2534] IOCTL: ADD_TO_FUNCT_MSG_LOOKUP_TABLE - %lu bytes\n", 
                        pInputArray->NumOfBytes);
                OutputDebugStringA(dbg);
                printf("%s", dbg);
                
                /* Parse the input - it contains CAN IDs to accept (4 bytes each) */
                /* Format: NumIDs(4) + [CAN_ID(4)]... */
                if (pInputArray->NumOfBytes >= 4) {
                    uint32_t num_ids = pInputArray->NumOfBytes / 4;
                    uint8_t ioctl_data[132];  /* Max 32 IDs * 4 bytes + 4 byte count */
                    uint16_t ioctl_len = 0;
                    
                    /* Store count of IDs */
                    ioctl_data[0] = (num_ids >> 24) & 0xFF;
                    ioctl_data[1] = (num_ids >> 16) & 0xFF;
                    ioctl_data[2] = (num_ids >> 8) & 0xFF;
                    ioctl_data[3] = num_ids & 0xFF;
                    ioctl_len = 4;
                    
                    /* Copy CAN IDs */
                    for (uint32_t i = 0; i < num_ids && i < 32; i++) {
                        uint32_t offset = i * 4;
                        /* IDs are stored as 4-byte big-endian */
                        memcpy(&ioctl_data[4 + offset], &pInputArray->BytePtr[offset], 4);
                        ioctl_len += 4;
                        
                        uint32_t can_id = ((uint32_t)pInputArray->BytePtr[offset] << 24) |
                                          ((uint32_t)pInputArray->BytePtr[offset+1] << 16) |
                                          ((uint32_t)pInputArray->BytePtr[offset+2] << 8) |
                                          pInputArray->BytePtr[offset+3];
                        sprintf(dbg, "  ID[%lu] = 0x%08lX\n", i, can_id);
                        OutputDebugStringA(dbg);
                        printf("%s", dbg);
                    }
                    
                    wican_ioctl(&device->wican_ctx, channel->fw_channel_id,
                                ADD_TO_FUNCT_MSG_LOOKUP_TABLE, ioctl_data, ioctl_len);
                }
            } else {
                OutputDebugStringA("[J2534] IOCTL: ADD_TO_FUNCT_MSG_LOOKUP_TABLE - no input\n");
            }
            break;
            
        case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
            /* Delete CAN IDs from functional message lookup table */
            if (pInput && device && channel) {
                SBYTE_ARRAY *pInputArray = (SBYTE_ARRAY *)pInput;
                char dbg[128];
                sprintf(dbg, "[J2534] IOCTL: DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE - %lu bytes\n", 
                        pInputArray->NumOfBytes);
                OutputDebugStringA(dbg);
                
                if (pInputArray->NumOfBytes >= 4) {
                    uint32_t num_ids = pInputArray->NumOfBytes / 4;
                    uint8_t ioctl_data[132];
                    uint16_t ioctl_len = 0;
                    
                    ioctl_data[0] = (num_ids >> 24) & 0xFF;
                    ioctl_data[1] = (num_ids >> 16) & 0xFF;
                    ioctl_data[2] = (num_ids >> 8) & 0xFF;
                    ioctl_data[3] = num_ids & 0xFF;
                    ioctl_len = 4;
                    
                    for (uint32_t i = 0; i < num_ids && i < 32; i++) {
                        uint32_t offset = i * 4;
                        memcpy(&ioctl_data[4 + offset], &pInputArray->BytePtr[offset], 4);
                        ioctl_len += 4;
                    }
                    
                    wican_ioctl(&device->wican_ctx, channel->fw_channel_id,
                                DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE, ioctl_data, ioctl_len);
                }
            } else {
                OutputDebugStringA("[J2534] IOCTL: DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE - no input\n");
            }
            break;
            
        default:
            {
                char dbg[128];
                sprintf(dbg, "[J2534] Unsupported IOCTL ID: 0x%lX\n", IoctlID);
                OutputDebugStringA(dbg);
            }
            LeaveCriticalSection(&g_cs_driver);
            set_error("Unsupported IOCTL");
            return ERR_INVALID_IOCTL_ID;
    }
    
    LeaveCriticalSection(&g_cs_driver);
    return STATUS_NOERROR;
}
