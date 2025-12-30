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
 * @file wican_comm.c
 * @brief WiCAN TCP and USB Serial Communication Implementation
 */

#include "wican_comm.h"
#include <stdio.h>
#include <string.h>
#include <setupapi.h>

#pragma comment(lib, "setupapi.lib")

/* GUID for USB Serial devices (COM ports) */
static const GUID GUID_DEVINTERFACE_COMPORT = 
    {0x86E0D1E0L, 0x8089, 0x11D0, {0x9C, 0xE4, 0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73}};

/* ============================================================================
 * Private Variables
 * ============================================================================ */
static bool g_winsock_initialized = false;
static WSADATA g_wsa_data;
static char g_config_ip[64] = "";
static uint16_t g_config_port = 0;
static char g_config_com_port[16] = "";
static uint8_t g_config_transport = WICAN_TRANSPORT_TCP;

/* ============================================================================
 * Registry Functions
 * ============================================================================ */
static void load_config_from_registry(void)
{
    HKEY hKey;
    DWORD type, size;
    
    /* Try 64-bit registry first, then 32-bit */
    const char *paths[] = {
        "SOFTWARE\\WiCAN",
        "SOFTWARE\\WOW6432Node\\WiCAN"
    };
    
    for (int i = 0; i < 2; i++) {
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, paths[i], 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
            size = sizeof(g_config_ip);
            type = REG_SZ;
            RegQueryValueExA(hKey, "IPAddress", NULL, &type, (LPBYTE)g_config_ip, &size);
            
            DWORD port = 0;
            size = sizeof(port);
            type = REG_DWORD;
            if (RegQueryValueExA(hKey, "Port", NULL, &type, (LPBYTE)&port, &size) == ERROR_SUCCESS) {
                g_config_port = (uint16_t)port;
            }
            
            /* Read USB Serial settings */
            size = sizeof(g_config_com_port);
            type = REG_SZ;
            RegQueryValueExA(hKey, "COMPort", NULL, &type, (LPBYTE)g_config_com_port, &size);
            
            DWORD transport = WICAN_TRANSPORT_TCP;
            size = sizeof(transport);
            type = REG_DWORD;
            if (RegQueryValueExA(hKey, "Transport", NULL, &type, (LPBYTE)&transport, &size) == ERROR_SUCCESS) {
                g_config_transport = (uint8_t)transport;
            }
            
            RegCloseKey(hKey);
            if (g_config_ip[0] != '\0' || g_config_com_port[0] != '\0') break;
        }
    }
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static bool set_socket_timeouts(SOCKET sock, uint32_t recv_timeout_ms, uint32_t send_timeout_ms)
{
    DWORD timeout;
    
    timeout = recv_timeout_ms;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) < 0) {
        return false;
    }
    
    timeout = send_timeout_ms;
    if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout)) < 0) {
        return false;
    }
    
    return true;
}

static bool set_socket_nonblocking(SOCKET sock, bool nonblocking)
{
    u_long mode = nonblocking ? 1 : 0;
    return (ioctlsocket(sock, FIONBIO, &mode) == 0);
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

bool wican_init(void)
{
    if (g_winsock_initialized) {
        return true;
    }
    
    if (WSAStartup(MAKEWORD(2, 2), &g_wsa_data) != 0) {
        return false;
    }
    
    /* Load IP/port from registry */
    load_config_from_registry();
    
    g_winsock_initialized = true;
    return true;
}

void wican_cleanup(void)
{
    if (g_winsock_initialized) {
        WSACleanup();
        g_winsock_initialized = false;
    }
}

bool wican_connect(wican_context_t *ctx, const char *ip_address, uint16_t port)
{
    struct sockaddr_in server_addr;
    struct timeval tv;
    fd_set fdset;
    int result;
    const char *use_ip;
    uint16_t use_port;
    
    OutputDebugStringA("[WICAN] wican_connect: START\n");
    
    if (!ctx || !g_winsock_initialized) {
        OutputDebugStringA("[WICAN] wican_connect: ctx or winsock not initialized\n");
        return false;
    }
    
    memset(ctx, 0, sizeof(wican_context_t));
    InitializeCriticalSection(&ctx->cs_socket);
    
    /* Check if ip_address looks like an IP address (contains a dot) */
    bool is_valid_ip = false;
    if (ip_address && ip_address[0]) {
        /* Simple check: if it contains a '.', assume it's an IP address */
        for (const char *p = ip_address; *p; p++) {
            if (*p == '.') {
                is_valid_ip = true;
                break;
            }
        }
    }
    
    /* Priority: 1) Valid IP parameter, 2) Registry config, 3) Default */
    if (is_valid_ip) {
        use_ip = ip_address;
    } else if (g_config_ip[0]) {
        use_ip = g_config_ip;
    } else {
        use_ip = WICAN_DEFAULT_IP;
    }
    
    {
        char dbg[256];
        sprintf(dbg, "[WICAN] connect: param='%s' is_valid_ip=%d using='%s'\n", 
                ip_address ? ip_address : "(null)", is_valid_ip, use_ip);
        OutputDebugStringA(dbg);
    }
    
    if (port) {
        use_port = port;
    } else if (g_config_port) {
        use_port = g_config_port;
    } else {
        use_port = WICAN_DEFAULT_PORT;
    }
    
    strncpy(ctx->ip_address, use_ip, sizeof(ctx->ip_address) - 1);
    ctx->port = use_port;
    
    ctx->socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ctx->socket == INVALID_SOCKET) {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_connect: socket() failed, error=%d\n", WSAGetLastError());
        OutputDebugStringA(dbg);
        DeleteCriticalSection(&ctx->cs_socket);
        return false;
    }
    
    OutputDebugStringA("[WICAN] wican_connect: socket created, setting non-blocking\n");
    set_socket_nonblocking(ctx->socket, true);
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(ctx->port);
    server_addr.sin_addr.s_addr = inet_addr(ctx->ip_address);
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_connect: connecting to %s:%d\n", ctx->ip_address, ctx->port);
        OutputDebugStringA(dbg);
    }
    
    result = connect(ctx->socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if (result == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            char dbg[128];
            sprintf(dbg, "[WICAN] wican_connect: connect() failed immediately, error=%d\n", err);
            OutputDebugStringA(dbg);
            closesocket(ctx->socket);
            DeleteCriticalSection(&ctx->cs_socket);
            return false;
        }
        
        OutputDebugStringA("[WICAN] wican_connect: waiting for connection (select)...\n");
        FD_ZERO(&fdset);
        FD_SET(ctx->socket, &fdset);
        tv.tv_sec = WICAN_CONNECT_TIMEOUT_MS / 1000;
        tv.tv_usec = (WICAN_CONNECT_TIMEOUT_MS % 1000) * 1000;
        
        result = select((int)ctx->socket + 1, NULL, &fdset, NULL, &tv);
        if (result <= 0) {
            char dbg[128];
            sprintf(dbg, "[WICAN] wican_connect: select() timeout/error, result=%d, error=%d\n", result, WSAGetLastError());
            OutputDebugStringA(dbg);
            closesocket(ctx->socket);
            DeleteCriticalSection(&ctx->cs_socket);
            return false;
        }
    }
    
    OutputDebugStringA("[WICAN] wican_connect: connection established, configuring socket\n");
    set_socket_nonblocking(ctx->socket, false);
    set_socket_timeouts(ctx->socket, WICAN_READ_TIMEOUT_MS, 1000);

    int flag = 1;
    setsockopt(ctx->socket, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
    
    /* Increase socket buffer sizes for better throughput during ECU reprogramming.
     * Default Windows buffers are ~8KB, which can cause bottlenecks when receiving
     * large ISO-TP responses or sending rapid consecutive frames. 64KB provides
     * headroom for WiFi latency variations without blocking. */
    int buf_size = 65536;  /* 64KB */
    setsockopt(ctx->socket, SOL_SOCKET, SO_RCVBUF, (char*)&buf_size, sizeof(buf_size));
    setsockopt(ctx->socket, SOL_SOCKET, SO_SNDBUF, (char*)&buf_size, sizeof(buf_size));

    ctx->transport_type = WICAN_TRANSPORT_TCP;
    ctx->connected = true;
    return true;
}

/* ============================================================================
 * USB Serial Functions
 * ============================================================================ */

int wican_find_usb_devices(char ports[][16], int max_ports)
{
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA devInfoData;
    int count = 0;
    char friendlyName[256];
    char portName[16];
    DWORD size;
    HKEY hDevKey;
    
    OutputDebugStringA("[WICAN] wican_find_usb_devices: scanning for WiCAN devices...\n");
    
    hDevInfo = SetupDiGetClassDevsA(&GUID_DEVINTERFACE_COMPORT, NULL, NULL, 
                                     DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        OutputDebugStringA("[WICAN] wican_find_usb_devices: SetupDiGetClassDevs failed\n");
        return 0;
    }
    
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    
    for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        /* Get friendly name */
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                                               NULL, (PBYTE)friendlyName, sizeof(friendlyName), &size)) {
            
            /* Look for CH342 (WiCAN Pro USB chip) */
            if (strstr(friendlyName, "CH342") != NULL || 
                strstr(friendlyName, "CH34") != NULL ||
                strstr(friendlyName, "WiCAN") != NULL) {
                
                /* Get the COM port name from device registry */
                hDevKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 
                                                0, DIREG_DEV, KEY_READ);
                if (hDevKey != INVALID_HANDLE_VALUE) {
                    size = sizeof(portName);
                    if (RegQueryValueExA(hDevKey, "PortName", NULL, NULL, 
                                         (LPBYTE)portName, &size) == ERROR_SUCCESS) {
                        if (count < max_ports) {
                            strncpy(ports[count], portName, 15);
                            ports[count][15] = '\0';
                            
                            char dbg[256];
                            sprintf(dbg, "[WICAN] Found WiCAN device: %s (%s)\n", portName, friendlyName);
                            OutputDebugStringA(dbg);
                            
                            count++;
                        }
                    }
                    RegCloseKey(hDevKey);
                }
            }
        }
    }
    
    SetupDiDestroyDeviceInfoList(hDevInfo);
    
    char dbg[64];
    sprintf(dbg, "[WICAN] wican_find_usb_devices: found %d devices\n", count);
    OutputDebugStringA(dbg);
    
    return count;
}

bool wican_connect_usb(wican_context_t *ctx, const char *com_port, uint32_t baudrate)
{
    char portPath[32];
    DCB dcb;
    COMMTIMEOUTS timeouts;
    const char *use_port;
    uint32_t use_baudrate;
    char found_ports[8][16];
    
    OutputDebugStringA("[WICAN] wican_connect_usb: START\n");
    
    if (!ctx) {
        OutputDebugStringA("[WICAN] wican_connect_usb: ctx is NULL\n");
        return false;
    }
    
    memset(ctx, 0, sizeof(wican_context_t));
    InitializeCriticalSection(&ctx->cs_socket);
    ctx->socket = INVALID_SOCKET;
    ctx->hSerial = INVALID_HANDLE_VALUE;
    
    /* Determine which COM port to use */
    if (com_port && com_port[0]) {
        use_port = com_port;
    } else if (g_config_com_port[0]) {
        use_port = g_config_com_port;
    } else {
        /* Auto-detect */
        int num_found = wican_find_usb_devices(found_ports, 8);
        if (num_found > 0) {
            use_port = found_ports[0];
            OutputDebugStringA("[WICAN] wican_connect_usb: auto-detected port\n");
        } else {
            OutputDebugStringA("[WICAN] wican_connect_usb: no WiCAN USB devices found\n");
            DeleteCriticalSection(&ctx->cs_socket);
            return false;
        }
    }
    
    use_baudrate = baudrate ? baudrate : WICAN_DEFAULT_BAUDRATE;
    
    strncpy(ctx->com_port, use_port, sizeof(ctx->com_port) - 1);
    ctx->serial_baudrate = use_baudrate;
    
    /* Build the port path */
    sprintf(portPath, "\\\\.\\%s", use_port);
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_connect_usb: opening %s at %lu baud\n", portPath, use_baudrate);
        OutputDebugStringA(dbg);
    }
    
    /* Open the serial port */
    ctx->hSerial = CreateFileA(portPath, GENERIC_READ | GENERIC_WRITE, 
                                0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (ctx->hSerial == INVALID_HANDLE_VALUE) {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_connect_usb: CreateFile failed, error=%lu\n", GetLastError());
        OutputDebugStringA(dbg);
        DeleteCriticalSection(&ctx->cs_socket);
        return false;
    }
    
    /* Configure the serial port - get current state first to preserve driver-specific flags */
    dcb.DCBlength = sizeof(dcb);
    
    if (!GetCommState(ctx->hSerial, &dcb)) {
        OutputDebugStringA("[WICAN] wican_connect_usb: GetCommState failed\n");
        CloseHandle(ctx->hSerial);
        ctx->hSerial = INVALID_HANDLE_VALUE;
        DeleteCriticalSection(&ctx->cs_socket);
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] USB port current settings: Baud=%lu, ByteSize=%d\n", dcb.BaudRate, dcb.ByteSize);
        OutputDebugStringA(dbg);
    }
    
    /* Try to set the baud rate if it's not already at 2Mbaud */
    if (dcb.BaudRate != use_baudrate) {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_connect_usb: baud rate is %lu, trying to set %lu\n", dcb.BaudRate, use_baudrate);
        OutputDebugStringA(dbg);
        
        dcb.BaudRate = use_baudrate;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        
        if (!SetCommState(ctx->hSerial, &dcb)) {
            DWORD err = GetLastError();
            sprintf(dbg, "[WICAN] wican_connect_usb: SetCommState failed (error %lu) - CH342 quirk, continuing\n", err);
            OutputDebugStringA(dbg);
            /* CH342 driver returns error but may still work - continue anyway */
        } else {
            OutputDebugStringA("[WICAN] wican_connect_usb: SetCommState succeeded\n");
        }
        
        /* Verify the baud rate was actually set */
        DCB dcb_verify;
        dcb_verify.DCBlength = sizeof(dcb_verify);
        if (GetCommState(ctx->hSerial, &dcb_verify)) {
            sprintf(dbg, "[WICAN] wican_connect_usb: baud rate is now %lu\n", dcb_verify.BaudRate);
            OutputDebugStringA(dbg);
            
            if (dcb_verify.BaudRate != use_baudrate && dcb_verify.BaudRate != 2000000) {
                sprintf(dbg, "[WICAN] WARNING: Baud rate mismatch! Port=%lu, Expected=%lu\n", dcb_verify.BaudRate, use_baudrate);
                OutputDebugStringA(dbg);
                /* Connection may fail - but try anyway */
            }
        }
    }
    
    /* Skip SetCommState - CH342 driver doesn't support it properly */
    
    /* Set timeouts */
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = WICAN_READ_TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 1000;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(ctx->hSerial, &timeouts)) {
        OutputDebugStringA("[WICAN] wican_connect_usb: SetCommTimeouts failed\n");
        CloseHandle(ctx->hSerial);
        ctx->hSerial = INVALID_HANDLE_VALUE;
        DeleteCriticalSection(&ctx->cs_socket);
        return false;
    }
    
    /* Clear any pending data */
    PurgeComm(ctx->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    
    ctx->transport_type = WICAN_TRANSPORT_USB;
    ctx->connected = true;
    
    OutputDebugStringA("[WICAN] wican_connect_usb: SUCCESS\n");
    return true;
}

bool wican_connect_auto(wican_context_t *ctx)
{
    char found_ports[8][16];
    int num_found;
    
    OutputDebugStringA("[WICAN] wican_connect_auto: trying USB first...\n");
    
    /* First, check registry for preferred transport */
    if (g_config_transport == WICAN_TRANSPORT_USB || g_config_com_port[0]) {
        if (wican_connect_usb(ctx, g_config_com_port, 0)) {
            return true;
        }
    }
    
    /* Try to find USB devices */
    num_found = wican_find_usb_devices(found_ports, 8);
    if (num_found > 0) {
        if (wican_connect_usb(ctx, found_ports[0], 0)) {
            return true;
        }
    }
    
    OutputDebugStringA("[WICAN] wican_connect_auto: USB failed, trying TCP...\n");
    
    /* Fall back to TCP */
    return wican_connect(ctx, NULL, 0);
}

void wican_disconnect(wican_context_t *ctx)
{
    if (!ctx) {
        return;
    }
    
    EnterCriticalSection(&ctx->cs_socket);
    
    if (ctx->connected) {
        if (ctx->transport_type == WICAN_TRANSPORT_USB) {
            /* Close USB serial */
            if (ctx->hSerial != INVALID_HANDLE_VALUE) {
                CloseHandle(ctx->hSerial);
                ctx->hSerial = INVALID_HANDLE_VALUE;
            }
        } else {
            /* Close TCP socket */
            if (ctx->socket != INVALID_SOCKET) {
                shutdown(ctx->socket, SD_BOTH);
                closesocket(ctx->socket);
                ctx->socket = INVALID_SOCKET;
            }
        }
    }
    ctx->connected = false;
    
    LeaveCriticalSection(&ctx->cs_socket);
    DeleteCriticalSection(&ctx->cs_socket);
}

bool wican_is_connected(wican_context_t *ctx)
{
    if (!ctx) {
        return false;
    }
    return ctx->connected;
}

uint8_t wican_calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool wican_send_command(wican_context_t *ctx, uint8_t cmd, const uint8_t *data, uint16_t data_len)
{
    uint8_t packet[WICAN_MAX_PACKET_SIZE];
    uint16_t packet_len;
    int sent;
    
    if (!ctx || !ctx->connected) {
        OutputDebugStringA("wican_send_command: not connected\n");
        return false;
    }
    
    if (data_len > WICAN_MAX_PACKET_SIZE - 6) {
        return false;
    }
    
    EnterCriticalSection(&ctx->cs_socket);
    
    packet[0] = WICAN_SYNC_BYTE1;
    packet[1] = WICAN_SYNC_BYTE2;
    packet[2] = cmd;
    packet[3] = (uint8_t)(data_len >> 8);
    packet[4] = (uint8_t)(data_len & 0xFF);
    
    if (data && data_len > 0) {
        memcpy(&packet[5], data, data_len);
    }
    
    /* Firmware expects checksum over entire packet (bytes 0 to 4+data_len) */
    packet[5 + data_len] = wican_calc_checksum(packet, 5 + data_len);
    packet_len = 6 + data_len;
    
    {
        char dbg[256];
        sprintf(dbg, "wican_send_command: cmd=0x%02X len=%d transport=%d packet=", cmd, packet_len, ctx->transport_type);
        OutputDebugStringA(dbg);
        for (int i = 0; i < packet_len && i < 20; i++) {
            sprintf(dbg, "%02X ", packet[i]);
            OutputDebugStringA(dbg);
        }
        OutputDebugStringA("\n");
    }
    
    if (ctx->transport_type == WICAN_TRANSPORT_USB) {
        /* USB Serial transport */
        DWORD bytes_written = 0;
        BOOL result = WriteFile(ctx->hSerial, packet, packet_len, &bytes_written, NULL);
        LeaveCriticalSection(&ctx->cs_socket);
        
        if (!result || bytes_written != packet_len) {
            char dbg[128];
            sprintf(dbg, "wican_send_command: WriteFile failed, written=%lu, error=%lu\n", bytes_written, GetLastError());
            OutputDebugStringA(dbg);
            return false;
        }
        return true;
    } else {
        /* TCP transport */
        sent = send(ctx->socket, (const char*)packet, packet_len, 0);
        LeaveCriticalSection(&ctx->cs_socket);
        
        if (sent != packet_len) {
            char dbg[128];
            sprintf(dbg, "wican_send_command: send failed, sent=%d, error=%d\n", sent, WSAGetLastError());
            OutputDebugStringA(dbg);
            return false;
        }
        return true;
    }
}

bool wican_receive_response(wican_context_t *ctx, uint8_t *cmd, uint8_t *status,
                            uint8_t *data, uint16_t *data_len, uint32_t timeout_ms)
{
    uint8_t header[6];  /* SYNC1, SYNC2, CMD, STATUS, LEN_H, LEN_L */
    uint8_t checksum;
    uint16_t payload_len;
    int received;
    DWORD old_timeout;
    int opt_len = sizeof(old_timeout);
    COMMTIMEOUTS old_timeouts, new_timeouts;
    
    if (!ctx || !ctx->connected) {
        OutputDebugStringA("wican_receive_response: not connected\n");
        return false;
    }
    
    EnterCriticalSection(&ctx->cs_socket);
    
    if (ctx->transport_type == WICAN_TRANSPORT_USB) {
        /* USB Serial transport - set read timeout */
        GetCommTimeouts(ctx->hSerial, &old_timeouts);
        new_timeouts = old_timeouts;
        new_timeouts.ReadIntervalTimeout = 50;
        new_timeouts.ReadTotalTimeoutConstant = timeout_ms;
        new_timeouts.ReadTotalTimeoutMultiplier = 10;
        SetCommTimeouts(ctx->hSerial, &new_timeouts);
        
        /* Read header */
        DWORD bytes_read = 0;
        if (!ReadFile(ctx->hSerial, header, 6, &bytes_read, NULL) || bytes_read != 6) {
            char dbg[128];
            sprintf(dbg, "wican_receive_response: ReadFile header failed, got %lu bytes, error %lu\n", bytes_read, GetLastError());
            OutputDebugStringA(dbg);
            goto error_usb;
        }
    } else {
        /* TCP transport */
        getsockopt(ctx->socket, SOL_SOCKET, SO_RCVTIMEO, (char*)&old_timeout, &opt_len);
        DWORD new_timeout = timeout_ms;
        setsockopt(ctx->socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&new_timeout, sizeof(new_timeout));
        
        /* Read header: SYNC1, SYNC2, CMD|0x80, STATUS, LEN_H, LEN_L */
        received = recv(ctx->socket, (char*)header, 6, MSG_WAITALL);
        if (received != 6) {
            char dbg[128];
            sprintf(dbg, "wican_receive_response: recv header failed, got %d bytes, error %d\n", received, WSAGetLastError());
            OutputDebugStringA(dbg);
            goto error_tcp;
        }
    }
    
    {
        char dbg[128];
        sprintf(dbg, "wican_receive_response: header=%02X %02X %02X %02X %02X %02X\n", 
                header[0], header[1], header[2], header[3], header[4], header[5]);
        OutputDebugStringA(dbg);
    }
    
    if (header[0] != WICAN_SYNC_BYTE1 || header[1] != WICAN_SYNC_BYTE2) {
        OutputDebugStringA("wican_receive_response: sync mismatch\n");
        if (ctx->transport_type == WICAN_TRANSPORT_USB) goto error_usb;
        else goto error_tcp;
    }
    
    if (cmd) *cmd = header[2] & 0x7F;  /* Strip response flag */
    if (status) *status = header[3];   /* J2534 status code */
    payload_len = ((uint16_t)header[4] << 8) | header[5];
    
    if (payload_len > *data_len) {
        OutputDebugStringA("wican_receive_response: payload too large\n");
        if (ctx->transport_type == WICAN_TRANSPORT_USB) goto error_usb;
        else goto error_tcp;
    }
    
    /* Calculate checksum starting from header */
    uint8_t calc_checksum = wican_calc_checksum(header, 6);
    
    if (payload_len > 0) {
        if (ctx->transport_type == WICAN_TRANSPORT_USB) {
            DWORD bytes_read = 0;
            if (!ReadFile(ctx->hSerial, data, payload_len, &bytes_read, NULL) || bytes_read != payload_len) {
                OutputDebugStringA("wican_receive_response: ReadFile data failed\n");
                goto error_usb;
            }
        } else {
            received = recv(ctx->socket, (char*)data, payload_len, MSG_WAITALL);
            if (received != payload_len) {
                OutputDebugStringA("wican_receive_response: recv data failed\n");
                goto error_tcp;
            }
        }
        calc_checksum ^= wican_calc_checksum(data, payload_len);
    }
    
    /* Read checksum byte */
    if (ctx->transport_type == WICAN_TRANSPORT_USB) {
        DWORD bytes_read = 0;
        if (!ReadFile(ctx->hSerial, &checksum, 1, &bytes_read, NULL) || bytes_read != 1) {
            OutputDebugStringA("wican_receive_response: ReadFile checksum failed\n");
            goto error_usb;
        }
    } else {
        received = recv(ctx->socket, (char*)&checksum, 1, MSG_WAITALL);
        if (received != 1) {
            OutputDebugStringA("wican_receive_response: recv checksum failed\n");
            goto error_tcp;
        }
    }
    
    {
        char dbg[128];
        sprintf(dbg, "wican_receive_response: checksum recv=%02X calc=%02X\n", checksum, calc_checksum);
        OutputDebugStringA(dbg);
    }
    
    if (calc_checksum != checksum) {
        OutputDebugStringA("wican_receive_response: checksum mismatch\n");
        if (ctx->transport_type == WICAN_TRANSPORT_USB) goto error_usb;
        else goto error_tcp;
    }
    
    *data_len = payload_len;
    
    /* Restore timeouts and return success */
    if (ctx->transport_type == WICAN_TRANSPORT_USB) {
        SetCommTimeouts(ctx->hSerial, &old_timeouts);
    } else {
        setsockopt(ctx->socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&old_timeout, sizeof(old_timeout));
    }
    LeaveCriticalSection(&ctx->cs_socket);
    OutputDebugStringA("wican_receive_response: success\n");
    return true;
    
error_usb:
    SetCommTimeouts(ctx->hSerial, &old_timeouts);
    LeaveCriticalSection(&ctx->cs_socket);
    return false;
    
error_tcp:
    setsockopt(ctx->socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&old_timeout, sizeof(old_timeout));
    LeaveCriticalSection(&ctx->cs_socket);
    return false;
}

bool wican_device_open(wican_context_t *ctx, uint32_t *device_id)
{
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    OutputDebugStringA("[WICAN] wican_device_open: sending OPEN command\n");
    
    /* Send OPEN command with no data */
    if (!wican_send_command(ctx, WICAN_CMD_OPEN, NULL, 0)) {
        OutputDebugStringA("[WICAN] wican_device_open: wican_send_command failed\n");
        return false;
    }
    
    OutputDebugStringA("[WICAN] wican_device_open: waiting for response (2000ms timeout)\n");
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        OutputDebugStringA("[WICAN] wican_device_open: wican_receive_response failed (timeout or bad packet)\n");
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_device_open: resp_cmd=0x%02X resp_status=0x%02X resp_len=%u\n",
                resp_cmd, resp_status, resp_len);
        OutputDebugStringA(dbg);
    }
    
    if (resp_status != WICAN_RESP_OK) {
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_device_open: device returned error status 0x%02X\n", resp_status);
        OutputDebugStringA(dbg);
        return false;
    }
    
    /* Response contains device ID (4 bytes) */
    if (resp_len >= 4 && device_id) {
        *device_id = ((uint32_t)resp_data[0] << 24) | ((uint32_t)resp_data[1] << 16) |
                     ((uint32_t)resp_data[2] << 8) | resp_data[3];
        ctx->device_id = *device_id;
        
        char dbg[128];
        sprintf(dbg, "[WICAN] wican_device_open: SUCCESS, device_id=%lu\n", *device_id);
        OutputDebugStringA(dbg);
    }
    
    return true;
}

bool wican_device_close(wican_context_t *ctx, uint32_t device_id)
{
    uint8_t data[4];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    data[0] = (device_id >> 24) & 0xFF;
    data[1] = (device_id >> 16) & 0xFF;
    data[2] = (device_id >> 8) & 0xFF;
    data[3] = device_id & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_CLOSE, data, 4)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_can_connect(wican_context_t *ctx, uint32_t protocol, uint32_t flags, 
                       uint32_t baudrate, uint32_t *channel_id)
{
    uint8_t data[16];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    /* Determine timeout based on protocol type:
     * - CAN/ISO15765: 2 seconds (fast, direct ESP32 TWAI)
     * - J1850/ISO9141/ISO14230: 10 seconds (slow, STN chip init + bus wake)
     * Legacy protocols require longer timeout because:
     *   1. STN chip must be woken up (GPIO signals)
     *   2. ATZ reset + ATD defaults + configuration commands
     *   3. ATSP protocol selection
     *   4. Bus initialization (tester present message)
     */
    uint32_t connect_timeout_ms = 2000;  /* Default for CAN protocols */
    
    /* Check for legacy protocols (J1850, ISO9141, ISO14230) */
    switch (protocol) {
        case 0x01:  /* J1850VPW */
        case 0x02:  /* J1850PWM */
        case 0x03:  /* ISO9141 */
        case 0x04:  /* ISO14230 */
        case 0x8000: /* J1850VPW_PS */
        case 0x8001: /* J1850PWM_PS */
        case 0x8002: /* ISO9141_PS */
        case 0x8003: /* ISO14230_PS */
            connect_timeout_ms = 10000;  /* 10 seconds for legacy protocols */
            OutputDebugStringA("[WICAN] can_connect: legacy protocol detected, using 10 second timeout\n");
            break;
        default:
            break;
    }
    
    {
        char dbg[256];
        sprintf(dbg, "[WICAN] can_connect: device_id=%lu protocol=%lu flags=%lu baud=%lu timeout=%lu\n",
                ctx->device_id, protocol, flags, baudrate, connect_timeout_ms);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
    }
    
    /* Firmware expects: device_id(4) + protocol(4) + flags(4) + baudrate(4) = 16 bytes */
    data[0] = (ctx->device_id >> 24) & 0xFF;
    data[1] = (ctx->device_id >> 16) & 0xFF;
    data[2] = (ctx->device_id >> 8) & 0xFF;
    data[3] = ctx->device_id & 0xFF;
    
    data[4] = (protocol >> 24) & 0xFF;
    data[5] = (protocol >> 16) & 0xFF;
    data[6] = (protocol >> 8) & 0xFF;
    data[7] = protocol & 0xFF;
    
    data[8] = (flags >> 24) & 0xFF;
    data[9] = (flags >> 16) & 0xFF;
    data[10] = (flags >> 8) & 0xFF;
    data[11] = flags & 0xFF;
    
    data[12] = (baudrate >> 24) & 0xFF;
    data[13] = (baudrate >> 16) & 0xFF;
    data[14] = (baudrate >> 8) & 0xFF;
    data[15] = baudrate & 0xFF;
    
    {
        char dbg[256];
        sprintf(dbg, "[WICAN] can_connect: sending %02X %02X %02X %02X | %02X %02X %02X %02X | %02X %02X %02X %02X | %02X %02X %02X %02X\n",
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
        OutputDebugStringA(dbg);
    }
    
    if (!wican_send_command(ctx, WICAN_CMD_CONNECT, data, 16)) {
        OutputDebugStringA("[WICAN] can_connect: send_command FAILED\n");
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] can_connect: waiting for response (timeout=%lums)...\n", connect_timeout_ms);
        OutputDebugStringA(dbg);
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, connect_timeout_ms)) {
        OutputDebugStringA("[WICAN] can_connect: receive_response FAILED (timeout or error)\n");
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] can_connect: resp_cmd=0x%02X resp_status=0x%02X resp_len=%u\n",
                resp_cmd, resp_status, resp_len);
        OutputDebugStringA(dbg);
    }
    
    if (resp_status != WICAN_RESP_OK) {
        char dbg[128];
        sprintf(dbg, "[WICAN] can_connect: bad status 0x%02X\n", resp_status);
        OutputDebugStringA(dbg);
        return false;
    }
    
    /* Response contains channel_id (4 bytes) */
    if (resp_len >= 4 && channel_id) {
        *channel_id = ((uint32_t)resp_data[0] << 24) | ((uint32_t)resp_data[1] << 16) |
                      ((uint32_t)resp_data[2] << 8) | resp_data[3];
        {
            char dbg[128];
            sprintf(dbg, "[WICAN] can_connect: resp_len=%u, channel_id=%lu (0x%02X %02X %02X %02X)\n",
                    resp_len, *channel_id, resp_data[0], resp_data[1], resp_data[2], resp_data[3]);
            printf("%s", dbg);
        }
    }
    
    return true;
}

bool wican_can_disconnect(wican_context_t *ctx, uint32_t channel_id)
{
    uint8_t data[4];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_DISCONNECT, data, 4)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_read_messages(wican_context_t *ctx, uint32_t channel_id, wican_can_msg_t *msgs, 
                         uint32_t max_msgs, uint32_t *num_msgs, uint32_t timeout_ms)
{
    uint8_t req_data[12];
    uint8_t resp_data[WICAN_MAX_PACKET_SIZE];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    if (!msgs || !num_msgs || max_msgs == 0) {
        return false;
    }
    
    /* Request: channel_id(4) + max_msgs(4) + timeout(4) */
    req_data[0] = (channel_id >> 24) & 0xFF;
    req_data[1] = (channel_id >> 16) & 0xFF;
    req_data[2] = (channel_id >> 8) & 0xFF;
    req_data[3] = channel_id & 0xFF;
    
    req_data[4] = (max_msgs >> 24) & 0xFF;
    req_data[5] = (max_msgs >> 16) & 0xFF;
    req_data[6] = (max_msgs >> 8) & 0xFF;
    req_data[7] = max_msgs & 0xFF;
    
    req_data[8] = (timeout_ms >> 24) & 0xFF;
    req_data[9] = (timeout_ms >> 16) & 0xFF;
    req_data[10] = (timeout_ms >> 8) & 0xFF;
    req_data[11] = timeout_ms & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_READ_MSGS, req_data, 12)) {
        return false;
    }
    
    /* Wait longer for response because firmware may extend wait for ISO-TP multi-frame
     * completion (up to 5 seconds for large programming responses).
     * PassThruReadMsgs blocks
     * until the complete ISO-TP message is reassembled, regardless of timeout.
     */
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, timeout_ms + 6000)) {
        *num_msgs = 0;
        return true;
    }
    
    /* BUFFER_EMPTY (0x10) is not an error, just means no messages */
    if (resp_status == WICAN_RESP_BUFFER_EMPTY) {
        *num_msgs = 0;
        return true;
    }
    
    if (resp_status != WICAN_RESP_OK) {
        return false;
    }
    
    /* Response: num_msgs(4) + for each msg: protocol(4) + rx_status(4) + timestamp(4) + data_size(4) + extra(4) + data */
    if (resp_len < 4) {
        *num_msgs = 0;
        return true;
    }
    
    /* Debug: hex dump first 64 bytes of response to file */
    {
        char dbg[512];
        int dbg_len = sprintf(dbg, "ReadMsgs response (len=%u): ", resp_len);
        for (int k = 0; k < resp_len && k < 64; k++) {
            dbg_len += sprintf(dbg + dbg_len, "%02X ", resp_data[k]);
        }
        strcat(dbg, "\n");
        OutputDebugStringA(dbg);
        
        /* Also write to debug file on Desktop */
        FILE *df = fopen("C:\\Users\\night\\OneDrive\\Desktop\\wican_wire_debug.txt", "a");
        if (df) {
            fprintf(df, "%s", dbg);
            fclose(df);
        }
    }
    
    uint32_t msg_count = ((uint32_t)resp_data[0] << 24) |
                         ((uint32_t)resp_data[1] << 16) |
                         ((uint32_t)resp_data[2] << 8) |
                         resp_data[3];
    if (msg_count > max_msgs) {
        msg_count = max_msgs;
    }
    
    uint16_t offset = 4;
    for (uint32_t i = 0; i < msg_count && offset < resp_len; i++) {
        if (offset + 20 > resp_len) break;
        
        /* Protocol ID (4 bytes) - needed to detect legacy protocols */
        uint32_t protocol_id = ((uint32_t)resp_data[offset] << 24) |
                               ((uint32_t)resp_data[offset + 1] << 16) |
                               ((uint32_t)resp_data[offset + 2] << 8) |
                               resp_data[offset + 3];
        offset += 4;
        
        /* Check if this is a legacy protocol (J1850, ISO9141, ISO14230) */
        bool is_legacy_proto = (protocol_id == 0x01 || protocol_id == 0x02 ||
                                protocol_id == 0x03 || protocol_id == 0x04 ||
                                protocol_id == 0x8000 || protocol_id == 0x8001 ||
                                protocol_id == 0x8002 || protocol_id == 0x8003);
        
        /* RX status (4 bytes) */
        uint32_t rx_status = ((uint32_t)resp_data[offset] << 24) |
                             ((uint32_t)resp_data[offset + 1] << 16) |
                             ((uint32_t)resp_data[offset + 2] << 8) |
                             resp_data[offset + 3];
        offset += 4;
        
        /* Timestamp (4 bytes) */
        msgs[i].timestamp = ((uint32_t)resp_data[offset] << 24) |
                            ((uint32_t)resp_data[offset + 1] << 16) |
                            ((uint32_t)resp_data[offset + 2] << 8) |
                            resp_data[offset + 3];
        offset += 4;
        
        /* Data size (4 bytes) - J2534 order: timestamp, data_size, extra */
        uint32_t data_size = ((uint32_t)resp_data[offset] << 24) |
                             ((uint32_t)resp_data[offset + 1] << 16) |
                             ((uint32_t)resp_data[offset + 2] << 8) |
                             resp_data[offset + 3];
        
        {
            char dbg[256];
            sprintf(dbg, "MSG[%lu] raw: offset=%u proto=0x%lX rx_status=0x%lX data_size_bytes=%02X%02X%02X%02X (parsed=%lu)\n", 
                    i, offset, protocol_id, rx_status,
                    resp_data[offset], resp_data[offset+1], resp_data[offset+2], resp_data[offset+3], data_size);
            OutputDebugStringA(dbg);
            FILE *df = fopen("C:\\Users\\night\\OneDrive\\Desktop\\wican_wire_debug.txt", "a");
            if (df) { fprintf(df, "%s", dbg); fclose(df); }
        }
        
        offset += 4;
        
        /* Skip extra (4 bytes) */
        offset += 4;
        
        if (offset + data_size > resp_len) break;
        
        if (is_legacy_proto) {
            /* Legacy protocols: Data is raw message bytes (no CAN ID prefix) */
            msgs[i].can_id = 0;  /* Not used for legacy */
            msgs[i].data_len = (data_size > sizeof(msgs[i].data)) ? sizeof(msgs[i].data) : data_size;
            memcpy(msgs[i].data, &resp_data[offset], msgs[i].data_len);
            offset += data_size;
            msgs[i].flags = 0x80;  /* Flag legacy protocol */
            
            {
                char dbg[256];
                sprintf(dbg, "[WICAN] read_messages: LEGACY data_len=%u offset=%u data=%02X%02X%02X%02X%02X%02X\n", 
                        msgs[i].data_len, offset - data_size,
                        resp_data[offset - data_size], resp_data[offset - data_size + 1], 
                        resp_data[offset - data_size + 2], resp_data[offset - data_size + 3],
                        resp_data[offset - data_size + 4], resp_data[offset - data_size + 5]);
                OutputDebugStringA(dbg);
            }
        } else {
            /* CAN protocols: First 4 bytes of data are CAN ID */
            if (data_size >= 4) {
                msgs[i].can_id = ((uint32_t)resp_data[offset] << 24) |
                                 ((uint32_t)resp_data[offset + 1] << 16) |
                                 ((uint32_t)resp_data[offset + 2] << 8) |
                                 resp_data[offset + 3];
                offset += 4;
                data_size -= 4;
            }
            
            /* Remaining is message data (can be up to 4124 for ISO-TP) */
            msgs[i].data_len = (data_size > 4124) ? 4124 : data_size;
            memcpy(msgs[i].data, &resp_data[offset], msgs[i].data_len);
            offset += data_size;
            
            /* Set flags based on rx_status */
            msgs[i].flags = 0;
            if (rx_status & 0x100) {  /* CAN_29BIT_ID */
                msgs[i].flags |= 0x01;
            }
        }
        
        /* Preserve full rx_status for J2534 (includes TX_MSG_TYPE=0x01 for TX echoes) */
        msgs[i].rx_status = rx_status;
    }
    
    *num_msgs = msg_count;
    return true;
}

bool wican_write_messages(wican_context_t *ctx, uint32_t channel_id, uint32_t protocol_id,
                          const wican_can_msg_t *msgs, uint32_t num_msgs, 
                          uint32_t timeout_ms, uint32_t *num_sent)
{
    uint8_t data[WICAN_MAX_PACKET_SIZE];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    uint16_t offset = 0;
    
    if (!msgs || num_msgs == 0) {
        return false;
    }
    
    /* Firmware expects for each message:
     * channel_id(4) + timeout(4) + protocol_id(4) + tx_flags(4) + data_size(4) + data...
     * We'll send one message at a time
     */
    for (uint32_t i = 0; i < num_msgs; i++) {
        offset = 0;
        
        /* Channel ID */
        data[offset++] = (channel_id >> 24) & 0xFF;
        data[offset++] = (channel_id >> 16) & 0xFF;
        data[offset++] = (channel_id >> 8) & 0xFF;
        data[offset++] = channel_id & 0xFF;
        
        /* Timeout */
        data[offset++] = (timeout_ms >> 24) & 0xFF;
        data[offset++] = (timeout_ms >> 16) & 0xFF;
        data[offset++] = (timeout_ms >> 8) & 0xFF;
        data[offset++] = timeout_ms & 0xFF;
        
        /* Protocol ID */
        data[offset++] = (protocol_id >> 24) & 0xFF;
        data[offset++] = (protocol_id >> 16) & 0xFF;
        data[offset++] = (protocol_id >> 8) & 0xFF;
        data[offset++] = protocol_id & 0xFF;
        
        /* TX Flags */
        uint32_t tx_flags = msgs[i].flags;
        data[offset++] = (tx_flags >> 24) & 0xFF;
        data[offset++] = (tx_flags >> 16) & 0xFF;
        data[offset++] = (tx_flags >> 8) & 0xFF;
        data[offset++] = tx_flags & 0xFF;
        
        /* Check if this is a legacy protocol message (flag 0x80) */
        bool is_legacy_msg = (msgs[i].flags & 0x80) != 0;
        
        if (is_legacy_msg) {
            /* Legacy protocols (J1850, ISO9141, ISO14230):
             * Data is raw message bytes (no CAN ID prefix)
             * data_size = just the data length, no CAN ID */
            uint32_t data_size = msgs[i].data_len;
            data[offset++] = (data_size >> 24) & 0xFF;
            data[offset++] = (data_size >> 16) & 0xFF;
            data[offset++] = (data_size >> 8) & 0xFF;
            data[offset++] = data_size & 0xFF;
            
            /* Copy raw data directly (no CAN ID prefix) */
            uint16_t copy_len = msgs[i].data_len;
            if (copy_len > 4124) copy_len = 4124;
            memcpy(&data[offset], msgs[i].data, copy_len);
            offset += copy_len;
            
            {
                char dbg[256];
                sprintf(dbg, "[WICAN] write_messages: LEGACY ch=%lu data_len=%u raw_data=%02X%02X%02X%02X\n", 
                        channel_id, copy_len, 
                        msgs[i].data[0], msgs[i].data[1], msgs[i].data[2], msgs[i].data[3]);
                OutputDebugStringA(dbg);
                printf("%s", dbg);
            }
        } else {
            /* CAN protocols: Include 4-byte CAN ID prefix */
            uint32_t data_size = 4 + msgs[i].data_len;
            data[offset++] = (data_size >> 24) & 0xFF;
            data[offset++] = (data_size >> 16) & 0xFF;
            data[offset++] = (data_size >> 8) & 0xFF;
            data[offset++] = data_size & 0xFF;
            
            /* CAN ID (first 4 bytes of J2534 message data) */
            data[offset++] = (msgs[i].can_id >> 24) & 0xFF;
            data[offset++] = (msgs[i].can_id >> 16) & 0xFF;
            data[offset++] = (msgs[i].can_id >> 8) & 0xFF;
            data[offset++] = msgs[i].can_id & 0xFF;
            
            /* Message data (can be up to 4124 bytes for ISO-TP multi-frame) */
            uint16_t copy_len = msgs[i].data_len;
            if (copy_len > 4124) copy_len = 4124;
            memcpy(&data[offset], msgs[i].data, copy_len);
            offset += copy_len;
            
            {
                char dbg[256];
                sprintf(dbg, "[WICAN] write_messages: CAN ch=%lu CAN_ID=0x%08lX data_len=%u offset=%u\n", 
                        channel_id, msgs[i].can_id, copy_len, offset);
                OutputDebugStringA(dbg);
                printf("%s", dbg);
            }
        }
        
        if (!wican_send_command(ctx, WICAN_CMD_WRITE_MSGS, data, offset)) {
            OutputDebugStringA("[WICAN] write_messages: send_command failed\n");
            printf("[WICAN] write_messages: send_command failed\n");
            if (num_sent) *num_sent = i;
            return false;
        }
        
        /* CRITICAL: For ISO-TP multi-frame messages, the firmware must drain the TX queue
         * before responding. At 500kbps, a 4KB message (585 frames) takes ~175ms to transmit.
         * Use a minimum wait time to allow the firmware to complete TX drain.
         * J2534 timeout of 0 means "no timeout" but we still need to wait for firmware.
         */
        uint32_t wait_timeout = timeout_ms;
        if (wait_timeout < 6000) {
            wait_timeout = 6000;  /* Minimum 6 seconds for large ISO-TP TX to complete */
        }
        
        if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, wait_timeout)) {
            OutputDebugStringA("[WICAN] write_messages: receive_response failed\n");
            printf("[WICAN] write_messages: receive_response failed\n");
            if (num_sent) *num_sent = i;
            return false;
        }
        
        {
            char dbg[128];
            sprintf(dbg, "[WICAN] write_messages: resp_cmd=0x%02X resp_status=0x%02X\n",
                    resp_cmd, resp_status);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
        
        if (resp_status != WICAN_RESP_OK) {
            char dbg[128];
            sprintf(dbg, "[WICAN] write_messages: bad status 0x%02X\n", resp_status);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
            if (num_sent) *num_sent = i;
            return false;
        }
    }
    
    if (num_sent) *num_sent = num_msgs;
    return true;
}

bool wican_write_messages_batch(wican_context_t *ctx, uint32_t channel_id,
                                const wican_can_msg_t *msgs, uint32_t num_msgs, 
                                uint32_t timeout_ms, uint32_t *num_sent)
{
    /**
     * Batch write for raw CAN frames - critical for ECU reprogramming
     * 
     * Packet format:
     *   channel_id(4) + timeout(4) + num_msgs(4) + [msg_len(2) + msg_data(msg_len)]...
     * 
     * Each raw CAN message is: msg_len(2) + CAN_ID(4) + data(8) = 2 + 12 bytes
     * Max packet size is WICAN_MAX_PACKET_SIZE (4200 bytes)
     * So max messages per batch is about (4200 - 12) / 14 = ~299 messages
     */
    uint8_t data[WICAN_MAX_PACKET_SIZE];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    uint16_t offset = 0;
    
    if (!msgs || num_msgs == 0) {
        return false;
    }
    
    /* Header: channel_id(4) + timeout(4) + num_msgs(4) */
    data[offset++] = (channel_id >> 24) & 0xFF;
    data[offset++] = (channel_id >> 16) & 0xFF;
    data[offset++] = (channel_id >> 8) & 0xFF;
    data[offset++] = channel_id & 0xFF;
    
    data[offset++] = (timeout_ms >> 24) & 0xFF;
    data[offset++] = (timeout_ms >> 16) & 0xFF;
    data[offset++] = (timeout_ms >> 8) & 0xFF;
    data[offset++] = timeout_ms & 0xFF;
    
    data[offset++] = (num_msgs >> 24) & 0xFF;
    data[offset++] = (num_msgs >> 16) & 0xFF;
    data[offset++] = (num_msgs >> 8) & 0xFF;
    data[offset++] = num_msgs & 0xFF;
    
    /* Pack each message: msg_len(2) + CAN_ID(4) + data(up to 8) */
    for (uint32_t i = 0; i < num_msgs; i++) {
        uint16_t msg_len = 4 + msgs[i].data_len;  /* CAN ID + data */
        if (msg_len > 12) msg_len = 12;  /* Max 4 + 8 for raw CAN */
        
        /* Check if we have room for this message */
        if (offset + 2 + msg_len > WICAN_MAX_PACKET_SIZE - 10) {
            /* Packet full, send what we have and return partial success */
            {
                char dbg[128];
                sprintf(dbg, "[WICAN] write_batch: packet full at msg %lu/%lu\n", i, num_msgs);
                OutputDebugStringA(dbg);
                printf("%s", dbg);
            }
            break;
        }
        
        /* Message length */
        data[offset++] = (msg_len >> 8) & 0xFF;
        data[offset++] = msg_len & 0xFF;
        
        /* CAN ID */
        data[offset++] = (msgs[i].can_id >> 24) & 0xFF;
        data[offset++] = (msgs[i].can_id >> 16) & 0xFF;
        data[offset++] = (msgs[i].can_id >> 8) & 0xFF;
        data[offset++] = msgs[i].can_id & 0xFF;
        
        /* Data */
        uint8_t copy_len = msgs[i].data_len;
        if (copy_len > 8) copy_len = 8;
        memcpy(&data[offset], msgs[i].data, copy_len);
        offset += copy_len;
    }
    
    {
        char dbg[256];
        sprintf(dbg, "[WICAN] write_batch: ch=%lu msgs=%lu total_size=%u\n", 
                channel_id, num_msgs, offset);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
    }
    
    if (!wican_send_command(ctx, WICAN_CMD_WRITE_MSGS_BATCH, data, offset)) {
        OutputDebugStringA("[WICAN] write_batch: send_command failed\n");
        printf("[WICAN] write_batch: send_command failed\n");
        if (num_sent) *num_sent = 0;
        return false;
    }
    
    /* Wait for response - firmware will queue all frames first then respond */
    uint32_t wait_timeout = timeout_ms;
    if (wait_timeout < 6000) {
        wait_timeout = 6000;  /* Minimum 6 seconds for large batch TX */
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, wait_timeout)) {
        OutputDebugStringA("[WICAN] write_batch: receive_response failed\n");
        printf("[WICAN] write_batch: receive_response failed\n");
        if (num_sent) *num_sent = 0;
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "[WICAN] write_batch: resp_cmd=0x%02X resp_status=0x%02X\n",
                resp_cmd, resp_status);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
    }
    
    if (resp_status != WICAN_RESP_OK) {
        char dbg[128];
        sprintf(dbg, "[WICAN] write_batch: bad status 0x%02X\n", resp_status);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
        if (num_sent) *num_sent = 0;
        return false;
    }
    
    /* Parse response: number of messages sent */
    if (resp_len >= 4) {
        uint32_t sent_count = ((uint32_t)resp_data[0] << 24) |
                              ((uint32_t)resp_data[1] << 16) |
                              ((uint32_t)resp_data[2] << 8) |
                              resp_data[3];
        if (num_sent) *num_sent = sent_count;
        {
            char dbg[128];
            sprintf(dbg, "[WICAN] write_batch: sent %lu/%lu msgs\n", sent_count, num_msgs);
            OutputDebugStringA(dbg);
            printf("%s", dbg);
        }
    } else {
        if (num_sent) *num_sent = num_msgs;
    }
    
    return true;
}

bool wican_set_filter(wican_context_t *ctx, uint32_t channel_id, uint32_t filter_type, 
                      uint32_t mask, uint32_t pattern, uint32_t flow_control, uint32_t *filter_id)
{
    uint8_t data[20];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    uint16_t data_len;
    
    {
        char dbg[256];
        sprintf(dbg, "wican_set_filter: ch=%lu type=%lu mask=0x%08lX pattern=0x%08lX flow_ctrl=0x%08lX\n", 
                channel_id, filter_type, mask, pattern, flow_control);
        OutputDebugStringA(dbg);
        printf("%s", dbg);
    }
    
    /* Firmware expects: channel_id(4) + filter_type(4) + mask(4) + pattern(4) [+ flow_control(4)] */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    data[4] = (filter_type >> 24) & 0xFF;
    data[5] = (filter_type >> 16) & 0xFF;
    data[6] = (filter_type >> 8) & 0xFF;
    data[7] = filter_type & 0xFF;
    
    /* Mask as 4 bytes (CAN ID format) */
    data[8] = (mask >> 24) & 0xFF;
    data[9] = (mask >> 16) & 0xFF;
    data[10] = (mask >> 8) & 0xFF;
    data[11] = mask & 0xFF;
    
    /* Pattern as 4 bytes (CAN ID format) */
    data[12] = (pattern >> 24) & 0xFF;
    data[13] = (pattern >> 16) & 0xFF;
    data[14] = (pattern >> 8) & 0xFF;
    data[15] = pattern & 0xFF;
    
    /* For FLOW_CONTROL_FILTER, add flow control TX ID */
    if (filter_type == 0x03) {  /* FLOW_CONTROL_FILTER */
        data[16] = (flow_control >> 24) & 0xFF;
        data[17] = (flow_control >> 16) & 0xFF;
        data[18] = (flow_control >> 8) & 0xFF;
        data[19] = flow_control & 0xFF;
        data_len = 20;
    } else {
        data_len = 16;
    }
    
    if (!wican_send_command(ctx, WICAN_CMD_START_FILTER, data, data_len)) {
        OutputDebugStringA("wican_set_filter: send_command failed\n");
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        OutputDebugStringA("wican_set_filter: receive_response failed\n");
        return false;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "wican_set_filter: resp_cmd=0x%02X resp_status=0x%02X resp_len=%u\n",
                resp_cmd, resp_status, resp_len);
        OutputDebugStringA(dbg);
    }
    
    if (resp_status == WICAN_RESP_OK && resp_len >= 4) {
        if (filter_id) {
            *filter_id = ((uint32_t)resp_data[0] << 24) |
                         ((uint32_t)resp_data[1] << 16) |
                         ((uint32_t)resp_data[2] << 8) |
                         resp_data[3];
        }
        OutputDebugStringA("wican_set_filter: success\n");
        return true;
    }
    
    {
        char dbg[128];
        sprintf(dbg, "wican_set_filter: failed - status=0x%02X len=%u\n", resp_status, resp_len);
        OutputDebugStringA(dbg);
    }
    return false;
}

bool wican_clear_filter(wican_context_t *ctx, uint32_t channel_id, uint32_t filter_id)
{
    uint8_t data[8];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    /* Firmware expects: channel_id(4) + filter_id(4) = 8 bytes */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    data[4] = (filter_id >> 24) & 0xFF;
    data[5] = (filter_id >> 16) & 0xFF;
    data[6] = (filter_id >> 8) & 0xFF;
    data[7] = filter_id & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_STOP_FILTER, data, 8)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_set_config(wican_context_t *ctx, uint32_t channel_id, uint32_t param_id, uint32_t value)
{
    uint8_t data[20];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    /* IOCTL command: channel_id(4) + ioctl_id(4) + num_params(4) + param(4) + value(4) */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    /* IOCTL ID for SET_CONFIG = 0x02 */
    uint32_t ioctl_id = 0x02;
    data[4] = (ioctl_id >> 24) & 0xFF;
    data[5] = (ioctl_id >> 16) & 0xFF;
    data[6] = (ioctl_id >> 8) & 0xFF;
    data[7] = ioctl_id & 0xFF;
    
    /* Num Params = 1 */
    uint32_t num_params = 1;
    data[8] = (num_params >> 24) & 0xFF;
    data[9] = (num_params >> 16) & 0xFF;
    data[10] = (num_params >> 8) & 0xFF;
    data[11] = num_params & 0xFF;
    
    /* Param ID */
    data[12] = (param_id >> 24) & 0xFF;
    data[13] = (param_id >> 16) & 0xFF;
    data[14] = (param_id >> 8) & 0xFF;
    data[15] = param_id & 0xFF;
    
    /* Value */
    data[16] = (value >> 24) & 0xFF;
    data[17] = (value >> 16) & 0xFF;
    data[18] = (value >> 8) & 0xFF;
    data[19] = value & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_IOCTL, data, 20)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_set_baudrate(wican_context_t *ctx, uint32_t channel_id, uint32_t baudrate)
{
    uint8_t data[12];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    /* OLD format IOCTL: channel_id(4) + ioctl_id(4) + value(4) */
    /* This is backwards compatible with stock firmware */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    /* SET_DATA_RATE IOCTL (uses old single-value format for compatibility) */
    uint32_t ioctl_id = 0x01;  /* SET_DATA_RATE in old format */
    data[4] = (ioctl_id >> 24) & 0xFF;
    data[5] = (ioctl_id >> 16) & 0xFF;
    data[6] = (ioctl_id >> 8) & 0xFF;
    data[7] = ioctl_id & 0xFF;
    
    /* Baudrate value */
    data[8] = (baudrate >> 24) & 0xFF;
    data[9] = (baudrate >> 16) & 0xFF;
    data[10] = (baudrate >> 8) & 0xFF;
    data[11] = baudrate & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_IOCTL, data, 12)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_get_info(wican_context_t *ctx, char *fw_version, char *hw_version)
{
    uint8_t data[4];
    uint8_t resp_data[WICAN_MAX_PACKET_SIZE];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    /* READ_VERSION needs device_id */
    data[0] = (ctx->device_id >> 24) & 0xFF;
    data[1] = (ctx->device_id >> 16) & 0xFF;
    data[2] = (ctx->device_id >> 8) & 0xFF;
    data[3] = ctx->device_id & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_READ_VERSION, data, 4)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    if (resp_status == WICAN_RESP_OK) {
        /* Response contains 3 null-terminated strings: fw, dll, api */
        if (fw_version && resp_len > 0) {
            strncpy(fw_version, (char*)resp_data, 32);
            fw_version[31] = '\0';
        }
        if (hw_version) {
            /* Find second string */
            uint16_t offset = (uint16_t)strlen((char*)resp_data) + 1;
            if (offset < resp_len) {
                strncpy(hw_version, (char*)&resp_data[offset], 32);
                hw_version[31] = '\0';
            }
        }
        return true;
    }
    
    return false;
}

bool wican_ioctl(wican_context_t *ctx, uint32_t channel_id, uint32_t ioctl_id,
                 const uint8_t *input_data, uint16_t input_len)
{
    uint8_t data[WICAN_MAX_PACKET_SIZE - 16];  /* Leave room for header */
    uint8_t resp_data[32];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    uint16_t data_len = 8;  /* channel_id(4) + ioctl_id(4) */
    
    char dbg[256];
    sprintf(dbg, "[WICAN] wican_ioctl: channel=%lu ioctl=0x%lX input_len=%d\n", 
            channel_id, ioctl_id, input_len);
    OutputDebugStringA(dbg);
    printf("%s", dbg);
    
    /* Format: channel_id(4) + ioctl_id(4) + [optional input data] */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    data[4] = (ioctl_id >> 24) & 0xFF;
    data[5] = (ioctl_id >> 16) & 0xFF;
    data[6] = (ioctl_id >> 8) & 0xFF;
    data[7] = ioctl_id & 0xFF;
    
    /* Add optional input data */
    if (input_data && input_len > 0) {
        if (input_len > sizeof(data) - 8) {
            input_len = sizeof(data) - 8;
        }
        memcpy(&data[8], input_data, input_len);
        data_len += input_len;
    }
    
    if (!wican_send_command(ctx, WICAN_CMD_IOCTL, data, data_len)) {
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        return false;
    }
    
    return (resp_status == WICAN_RESP_OK);
}

bool wican_start_periodic_msg(wican_context_t *ctx, uint32_t channel_id, 
                              const wican_can_msg_t *msg, uint32_t interval_ms,
                              uint32_t *msg_id)
{
    uint8_t data[128];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    uint16_t offset = 0;
    
    char dbg[256];
    sprintf(dbg, "[WICAN] wican_start_periodic_msg: ch=%lu interval=%lu ms CAN_ID=0x%08lX\n",
            channel_id, interval_ms, msg ? msg->can_id : 0);
    OutputDebugStringA(dbg);
    printf("%s", dbg);
    
    if (!msg || !msg_id) {
        return false;
    }
    
    /* Format: channel_id(4) + interval_ms(4) + protocol_id(4) + tx_flags(4) + data_size(4) + data */
    
    /* Channel ID */
    data[offset++] = (channel_id >> 24) & 0xFF;
    data[offset++] = (channel_id >> 16) & 0xFF;
    data[offset++] = (channel_id >> 8) & 0xFF;
    data[offset++] = channel_id & 0xFF;
    
    /* Interval in ms */
    data[offset++] = (interval_ms >> 24) & 0xFF;
    data[offset++] = (interval_ms >> 16) & 0xFF;
    data[offset++] = (interval_ms >> 8) & 0xFF;
    data[offset++] = interval_ms & 0xFF;
    
    /* Protocol ID (CAN=5, ISO15765=6) */
    uint32_t protocol_id = 6;  /* Default to ISO15765 for TesterPresent */
    data[offset++] = (protocol_id >> 24) & 0xFF;
    data[offset++] = (protocol_id >> 16) & 0xFF;
    data[offset++] = (protocol_id >> 8) & 0xFF;
    data[offset++] = protocol_id & 0xFF;
    
    /* TX Flags */
    uint32_t tx_flags = msg->flags;
    data[offset++] = (tx_flags >> 24) & 0xFF;
    data[offset++] = (tx_flags >> 16) & 0xFF;
    data[offset++] = (tx_flags >> 8) & 0xFF;
    data[offset++] = tx_flags & 0xFF;
    
    /* Data size: 4 bytes CAN ID + data bytes (limited to 8 for periodic) */
    uint8_t msg_data_len = (msg->data_len > 8) ? 8 : msg->data_len;
    uint32_t data_size = 4 + msg_data_len;
    data[offset++] = (data_size >> 24) & 0xFF;
    data[offset++] = (data_size >> 16) & 0xFF;
    data[offset++] = (data_size >> 8) & 0xFF;
    data[offset++] = data_size & 0xFF;
    
    /* CAN ID (first 4 bytes of J2534 message data) */
    data[offset++] = (msg->can_id >> 24) & 0xFF;
    data[offset++] = (msg->can_id >> 16) & 0xFF;
    data[offset++] = (msg->can_id >> 8) & 0xFF;
    data[offset++] = msg->can_id & 0xFF;
    
    /* CAN data (limited to 8 bytes for periodic messages) */
    memcpy(&data[offset], msg->data, msg_data_len);
    offset += msg_data_len;
    
    if (!wican_send_command(ctx, WICAN_CMD_START_PERIODIC, data, offset)) {
        OutputDebugStringA("[WICAN] wican_start_periodic_msg: send_command failed\n");
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        OutputDebugStringA("[WICAN] wican_start_periodic_msg: receive_response failed\n");
        return false;
    }
    
    if (resp_status != WICAN_RESP_OK) {
        sprintf(dbg, "[WICAN] wican_start_periodic_msg: bad status 0x%02X\n", resp_status);
        OutputDebugStringA(dbg);
        return false;
    }
    
    /* Extract msg_id from response */
    if (resp_len >= 4) {
        *msg_id = ((uint32_t)resp_data[0] << 24) |
                  ((uint32_t)resp_data[1] << 16) |
                  ((uint32_t)resp_data[2] << 8) |
                  resp_data[3];
    } else {
        *msg_id = 0;
    }
    
    sprintf(dbg, "[WICAN] wican_start_periodic_msg: success, msg_id=%lu\n", *msg_id);
    OutputDebugStringA(dbg);
    printf("%s", dbg);
    
    return true;
}

bool wican_stop_periodic_msg(wican_context_t *ctx, uint32_t channel_id, uint32_t msg_id)
{
    uint8_t data[8];
    uint8_t resp_data[16];
    uint16_t resp_len = sizeof(resp_data);
    uint8_t resp_cmd, resp_status;
    
    char dbg[128];
    sprintf(dbg, "[WICAN] wican_stop_periodic_msg: ch=%lu msg_id=%lu\n", channel_id, msg_id);
    OutputDebugStringA(dbg);
    printf("%s", dbg);
    
    /* Format: channel_id(4) + msg_id(4) */
    data[0] = (channel_id >> 24) & 0xFF;
    data[1] = (channel_id >> 16) & 0xFF;
    data[2] = (channel_id >> 8) & 0xFF;
    data[3] = channel_id & 0xFF;
    
    data[4] = (msg_id >> 24) & 0xFF;
    data[5] = (msg_id >> 16) & 0xFF;
    data[6] = (msg_id >> 8) & 0xFF;
    data[7] = msg_id & 0xFF;
    
    if (!wican_send_command(ctx, WICAN_CMD_STOP_PERIODIC, data, 8)) {
        OutputDebugStringA("[WICAN] wican_stop_periodic_msg: send_command failed\n");
        return false;
    }
    
    if (!wican_receive_response(ctx, &resp_cmd, &resp_status, resp_data, &resp_len, 2000)) {
        OutputDebugStringA("[WICAN] wican_stop_periodic_msg: receive_response failed\n");
        return false;
    }
    
    if (resp_status != WICAN_RESP_OK) {
        sprintf(dbg, "[WICAN] wican_stop_periodic_msg: bad status 0x%02X\n", resp_status);
        OutputDebugStringA(dbg);
        return false;
    }
    
    OutputDebugStringA("[WICAN] wican_stop_periodic_msg: success\n");
    return true;
}
