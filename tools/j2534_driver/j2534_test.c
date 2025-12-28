/**
 * @file j2534_test.c
 * @brief Simple test application for WiCAN J2534 Driver
 * 
 * Compile: cl j2534_test.c /Fe:j2534_test.exe ws2_32.lib advapi32.lib
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _WINSOCKAPI_
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "j2534_api.h"

/* Function pointers for J2534 API */
typedef long (__stdcall *PTOPEN)(void*, unsigned long*);
typedef long (__stdcall *PTCLOSE)(unsigned long);
typedef long (__stdcall *PTCONNECT)(unsigned long, unsigned long, unsigned long, unsigned long, unsigned long*);
typedef long (__stdcall *PTDISCONNECT)(unsigned long);
typedef long (__stdcall *PTREADMSGS)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
typedef long (__stdcall *PTWRITEMSGS)(unsigned long, PASSTHRU_MSG*, unsigned long*, unsigned long);
typedef long (__stdcall *PTREADVERSION)(unsigned long, char*, char*, char*);
typedef long (__stdcall *PTGETLASTERROR)(char*);
typedef long (__stdcall *PTSTARTMSGFILTER)(unsigned long, unsigned long, PASSTHRU_MSG*, PASSTHRU_MSG*, PASSTHRU_MSG*, unsigned long*);
typedef long (__stdcall *PTSTOPMSGFILTER)(unsigned long, unsigned long);

/* Global function pointers */
PTOPEN g_PassThruOpen = NULL;
PTCLOSE g_PassThruClose = NULL;
PTCONNECT g_PassThruConnect = NULL;
PTDISCONNECT g_PassThruDisconnect = NULL;
PTREADMSGS g_PassThruReadMsgs = NULL;
PTWRITEMSGS g_PassThruWriteMsgs = NULL;
PTREADVERSION g_PassThruReadVersion = NULL;
PTGETLASTERROR g_PassThruGetLastError = NULL;
PTSTARTMSGFILTER g_PassThruStartMsgFilter = NULL;
PTSTOPMSGFILTER g_PassThruStopMsgFilter = NULL;

HMODULE g_hDll = NULL;

const char* get_error_string(long error_code)
{
    switch (error_code) {
        case STATUS_NOERROR: return "No Error";
        case ERR_NOT_SUPPORTED: return "Not Supported";
        case ERR_INVALID_CHANNEL_ID: return "Invalid Channel ID";
        case ERR_INVALID_PROTOCOL_ID: return "Invalid Protocol ID";
        case ERR_NULL_PARAMETER: return "Null Parameter";
        case ERR_INVALID_IOCTL_VALUE: return "Invalid IOCTL Value";
        case ERR_INVALID_FLAGS: return "Invalid Flags";
        case ERR_FAILED: return "Failed";
        case ERR_DEVICE_NOT_CONNECTED: return "Device Not Connected";
        case ERR_TIMEOUT: return "Timeout";
        case ERR_INVALID_MSG: return "Invalid Message";
        case ERR_INVALID_TIME_INTERVAL: return "Invalid Time Interval";
        case ERR_EXCEEDED_LIMIT: return "Exceeded Limit";
        case ERR_INVALID_MSG_ID: return "Invalid Message ID";
        case ERR_DEVICE_IN_USE: return "Device In Use";
        case ERR_INVALID_IOCTL_ID: return "Invalid IOCTL ID";
        case ERR_BUFFER_EMPTY: return "Buffer Empty";
        case ERR_BUFFER_FULL: return "Buffer Full";
        case ERR_BUFFER_OVERFLOW: return "Buffer Overflow";
        case ERR_PIN_INVALID: return "Invalid Pin";
        case ERR_CHANNEL_IN_USE: return "Channel In Use";
        case ERR_MSG_PROTOCOL_ID: return "Message Protocol ID Error";
        case ERR_INVALID_FILTER_ID: return "Invalid Filter ID";
        case ERR_NO_FLOW_CONTROL: return "No Flow Control";
        case ERR_NOT_UNIQUE: return "Not Unique";
        case ERR_INVALID_BAUDRATE: return "Invalid Baudrate";
        case ERR_INVALID_DEVICE_ID: return "Invalid Device ID";
        default: return "Unknown Error";
    }
}

int load_driver(const char *dll_path)
{
    g_hDll = LoadLibraryA(dll_path);
    if (!g_hDll) {
        printf("ERROR: Failed to load DLL: %s (error %lu)\n", dll_path, GetLastError());
        return 0;
    }
    
    g_PassThruOpen = (PTOPEN)GetProcAddress(g_hDll, "PassThruOpen");
    g_PassThruClose = (PTCLOSE)GetProcAddress(g_hDll, "PassThruClose");
    g_PassThruConnect = (PTCONNECT)GetProcAddress(g_hDll, "PassThruConnect");
    g_PassThruDisconnect = (PTDISCONNECT)GetProcAddress(g_hDll, "PassThruDisconnect");
    g_PassThruReadMsgs = (PTREADMSGS)GetProcAddress(g_hDll, "PassThruReadMsgs");
    g_PassThruWriteMsgs = (PTWRITEMSGS)GetProcAddress(g_hDll, "PassThruWriteMsgs");
    g_PassThruReadVersion = (PTREADVERSION)GetProcAddress(g_hDll, "PassThruReadVersion");
    g_PassThruGetLastError = (PTGETLASTERROR)GetProcAddress(g_hDll, "PassThruGetLastError");
    g_PassThruStartMsgFilter = (PTSTARTMSGFILTER)GetProcAddress(g_hDll, "PassThruStartMsgFilter");
    g_PassThruStopMsgFilter = (PTSTOPMSGFILTER)GetProcAddress(g_hDll, "PassThruStopMsgFilter");
    
    if (!g_PassThruOpen || !g_PassThruClose || !g_PassThruConnect || !g_PassThruDisconnect ||
        !g_PassThruReadMsgs || !g_PassThruWriteMsgs || !g_PassThruReadVersion || !g_PassThruGetLastError) {
        printf("ERROR: Failed to get function addresses from DLL\n");
        printf("  g_PassThruOpen: %s\n", g_PassThruOpen ? "OK" : "MISSING");
        printf("  g_PassThruClose: %s\n", g_PassThruClose ? "OK" : "MISSING");
        printf("  g_PassThruConnect: %s\n", g_PassThruConnect ? "OK" : "MISSING");
        printf("  g_PassThruDisconnect: %s\n", g_PassThruDisconnect ? "OK" : "MISSING");
        printf("  g_PassThruReadMsgs: %s\n", g_PassThruReadMsgs ? "OK" : "MISSING");
        printf("  g_PassThruWriteMsgs: %s\n", g_PassThruWriteMsgs ? "OK" : "MISSING");
        printf("  g_PassThruReadVersion: %s\n", g_PassThruReadVersion ? "OK" : "MISSING");
        printf("  g_PassThruGetLastError: %s\n", g_PassThruGetLastError ? "OK" : "MISSING");
        FreeLibrary(g_hDll);
        g_hDll = NULL;
        return 0;
    }
    
    printf("Driver loaded successfully: %s\n", dll_path);
    return 1;
}

void unload_driver(void)
{
    if (g_hDll) {
        FreeLibrary(g_hDll);
        g_hDll = NULL;
    }
}

void print_error(void)
{
    if (g_PassThruGetLastError) {
        char error_msg[80];
        g_PassThruGetLastError(error_msg);
        printf("  Last Error: %s\n", error_msg);
    }
}

int main(int argc, char *argv[])
{
    unsigned long device_id = 0;
    unsigned long channel_id = 0;
    unsigned long filter_id = 0;
    long ret;
    const char *dll_path = "wican_j2534_32.dll";
    const char *device_ip = NULL;
    
    printf("==============================================\n");
    printf("WiCAN J2534 Driver Test\n");
    printf("==============================================\n\n");
    
    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dll") == 0 && i + 1 < argc) {
            dll_path = argv[++i];
        } else if (strcmp(argv[i], "-ip") == 0 && i + 1 < argc) {
            device_ip = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [-dll <path>] [-ip <address>]\n", argv[0]);
            printf("  -dll <path>    Path to J2534 DLL (default: wican_j2534_32.dll)\n");
            printf("  -ip <address>  WiCAN device IP address (default: 192.168.80.1)\n");
            return 0;
        }
    }
    
    /* Load driver */
    if (!load_driver(dll_path)) {
        return 1;
    }
    
    /* Test 1: Open device */
    printf("\n[Test 1] PassThruOpen...\n");
    ret = g_PassThruOpen((void*)device_ip, &device_id);
    if (ret != STATUS_NOERROR) {
        printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
        print_error();
        unload_driver();
        return 1;
    }
    printf("  SUCCESS: Device ID = %lu\n", device_id);
    
    /* Test 2: Read version */
    printf("\n[Test 2] PassThruReadVersion...\n");
    {
        char fw_ver[80], dll_ver[80], api_ver[80];
        ret = g_PassThruReadVersion(device_id, fw_ver, dll_ver, api_ver);
        if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
            print_error();
        } else {
            printf("  SUCCESS:\n");
            printf("    Firmware: %s\n", fw_ver);
            printf("    DLL:      %s\n", dll_ver);
            printf("    API:      %s\n", api_ver);
        }
    }
    
    /* Test 3: Connect CAN */
    printf("\n[Test 3] PassThruConnect (CAN @ 500kbps)...\n");
    ret = g_PassThruConnect(device_id, CAN, 0, 500000, &channel_id);
    if (ret != STATUS_NOERROR) {
        printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
        print_error();
        goto cleanup;
    }
    printf("  SUCCESS: Channel ID = %lu\n", channel_id);
    
    /* Test 4: Start filter (pass all) */
    printf("\n[Test 4] PassThruStartMsgFilter...\n");
    if (g_PassThruStartMsgFilter) {
        PASSTHRU_MSG mask_msg = {0};
        PASSTHRU_MSG pattern_msg = {0};
        
        mask_msg.ProtocolID = CAN;
        mask_msg.DataSize = 4;
        
        pattern_msg.ProtocolID = CAN;
        pattern_msg.DataSize = 4;
        
        ret = g_PassThruStartMsgFilter(channel_id, PASS_FILTER, &mask_msg, &pattern_msg, NULL, &filter_id);
        if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
            print_error();
        } else {
            printf("  SUCCESS: Filter ID = %lu\n", filter_id);
        }
    }
    
    /* Test 5: Read messages */
    printf("\n[Test 5] PassThruReadMsgs (1 second timeout)...\n");
    {
        PASSTHRU_MSG msgs[10];
        unsigned long num_msgs = 10;
        
        ret = g_PassThruReadMsgs(channel_id, msgs, &num_msgs, 1000);
        if (ret == ERR_BUFFER_EMPTY) {
            printf("  No messages received (buffer empty)\n");
        } else if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
            print_error();
        } else {
            printf("  SUCCESS: Received %lu messages\n", num_msgs);
            for (unsigned long i = 0; i < num_msgs && i < 5; i++) {
                printf("    [%lu] ID=0x%02X%02X%02X%02X DLC=%lu\n", i,
                    msgs[i].Data[0], msgs[i].Data[1], msgs[i].Data[2], msgs[i].Data[3],
                    msgs[i].DataSize - 4);
            }
        }
    }
    
    /* Test 6: Write message */
    printf("\n[Test 6] PassThruWriteMsgs...\n");
    {
        PASSTHRU_MSG msg = {0};
        unsigned long num_msgs = 1;
        
        msg.ProtocolID = CAN;
        msg.DataSize = 12;
        msg.Data[0] = 0x00;
        msg.Data[1] = 0x00;
        msg.Data[2] = 0x07;
        msg.Data[3] = 0xDF;
        msg.Data[4] = 0x02;
        msg.Data[5] = 0x01;
        msg.Data[6] = 0x00;
        msg.Data[7] = 0x00;
        msg.Data[8] = 0x00;
        msg.Data[9] = 0x00;
        msg.Data[10] = 0x00;
        msg.Data[11] = 0x00;
        
        ret = g_PassThruWriteMsgs(channel_id, &msg, &num_msgs, 1000);
        if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
            print_error();
        } else {
            printf("  SUCCESS: Sent %lu messages\n", num_msgs);
        }
    }
    
    /* Test 7: Read response */
    printf("\n[Test 7] PassThruReadMsgs (wait for response)...\n");
    {
        PASSTHRU_MSG msgs[10];
        unsigned long num_msgs = 10;
        
        ret = g_PassThruReadMsgs(channel_id, msgs, &num_msgs, 2000);
        if (ret == ERR_BUFFER_EMPTY) {
            printf("  No response received (is vehicle connected?)\n");
        } else if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
            print_error();
        } else {
            printf("  SUCCESS: Received %lu messages\n", num_msgs);
            for (unsigned long i = 0; i < num_msgs; i++) {
                unsigned long can_id = ((unsigned long)msgs[i].Data[0] << 24) |
                                       ((unsigned long)msgs[i].Data[1] << 16) |
                                       ((unsigned long)msgs[i].Data[2] << 8) |
                                       msgs[i].Data[3];
                printf("    [%lu] ID=0x%03lX Data=", i, can_id);
                for (unsigned long j = 4; j < msgs[i].DataSize; j++) {
                    printf("%02X ", msgs[i].Data[j]);
                }
                printf("\n");
            }
        }
    }
    
    /* Test 8: Stop filter */
    if (filter_id) {
        printf("\n[Test 8] PassThruStopMsgFilter...\n");
        ret = g_PassThruStopMsgFilter(channel_id, filter_id);
        if (ret != STATUS_NOERROR) {
            printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
        } else {
            printf("  SUCCESS\n");
        }
    }
    
    /* Test 9: Disconnect */
    printf("\n[Test 9] PassThruDisconnect...\n");
    ret = g_PassThruDisconnect(channel_id);
    if (ret != STATUS_NOERROR) {
        printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
        print_error();
    } else {
        printf("  SUCCESS\n");
    }
    channel_id = 0;
    
cleanup:
    /* Test 10: Close device */
    printf("\n[Test 10] PassThruClose...\n");
    ret = g_PassThruClose(device_id);
    if (ret != STATUS_NOERROR) {
        printf("  FAILED: %s (%ld)\n", get_error_string(ret), ret);
        print_error();
    } else {
        printf("  SUCCESS\n");
    }
    
    unload_driver();
    
    printf("\n==============================================\n");
    printf("Test Complete!\n");
    printf("==============================================\n");
    
    return 0;
}
