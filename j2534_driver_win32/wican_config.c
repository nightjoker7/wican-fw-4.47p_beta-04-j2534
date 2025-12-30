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
 * @file wican_config.c
 * @brief WiCAN J2534 Configuration Utility
 */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <commctrl.h>
#include <setupapi.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "setupapi.lib")
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

/* COM port class GUID */
static const GUID GUID_DEVINTERFACE_COMPORT = 
    {0x86e0d1e0, 0x8089, 0x11d0, {0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73}};

#define WICAN_REG_PATH      "SOFTWARE\\WiCAN"
#define WICAN_REG_PATH_WOW  "SOFTWARE\\WOW6432Node\\WiCAN"

#define IDC_IP_EDIT         101
#define IDC_PORT_EDIT       102
#define IDC_SAVE_BTN        103
#define IDC_TEST_BTN        104
#define IDC_STATUS_TEXT     105
#define IDC_DEFAULT_BTN     106
#define IDC_TRANSPORT_COMBO 107
#define IDC_COM_COMBO       108
#define IDC_REFRESH_BTN     109

#define DEFAULT_IP          "192.168.80.1"
#define DEFAULT_PORT        3333

#define WICAN_TRANSPORT_TCP 0
#define WICAN_TRANSPORT_USB 1

HWND g_hWnd = NULL;
HWND g_hIpEdit = NULL;
HWND g_hPortEdit = NULL;
HWND g_hStatusText = NULL;
HWND g_hTransportCombo = NULL;
HWND g_hComCombo = NULL;
HWND g_hIpLabel = NULL;
HWND g_hPortLabel = NULL;
HWND g_hComLabel = NULL;
HWND g_hRefreshBtn = NULL;
int g_transport = WICAN_TRANSPORT_TCP;

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
void LoadSettings(void);
void SaveSettings(void);
void TestConnection(void);
void SetDefaults(void);
void SetStatus(const char *status);
void UpdateTransportUI(void);
void RefreshComPorts(void);
void TestUSBConnection(const char *comPort);

bool ReadRegString(const char *path, const char *name, char *value, DWORD size)
{
    HKEY hKey;
    DWORD type = REG_SZ;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, path, 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        if (RegQueryValueExA(hKey, name, NULL, &type, (LPBYTE)value, &size) == ERROR_SUCCESS) {
            RegCloseKey(hKey);
            return true;
        }
        RegCloseKey(hKey);
    }
    return false;
}

bool WriteRegString(const char *path, const char *name, const char *value)
{
    HKEY hKey;
    DWORD disposition;
    if (RegCreateKeyExA(HKEY_LOCAL_MACHINE, path, 0, NULL, 0, KEY_WRITE, NULL, &hKey, &disposition) == ERROR_SUCCESS) {
        if (RegSetValueExA(hKey, name, 0, REG_SZ, (const BYTE*)value, (DWORD)strlen(value) + 1) == ERROR_SUCCESS) {
            RegCloseKey(hKey);
            return true;
        }
        RegCloseKey(hKey);
    }
    return false;
}

bool WriteRegDword(const char *path, const char *name, DWORD value)
{
    HKEY hKey;
    DWORD disposition;
    if (RegCreateKeyExA(HKEY_LOCAL_MACHINE, path, 0, NULL, 0, KEY_WRITE, NULL, &hKey, &disposition) == ERROR_SUCCESS) {
        if (RegSetValueExA(hKey, name, 0, REG_DWORD, (const BYTE*)&value, sizeof(DWORD)) == ERROR_SUCCESS) {
            RegCloseKey(hKey);
            return true;
        }
        RegCloseKey(hKey);
    }
    return false;
}

bool ReadRegDword(const char *path, const char *name, DWORD *value)
{
    HKEY hKey;
    DWORD type = REG_DWORD;
    DWORD size = sizeof(DWORD);
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, path, 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        if (RegQueryValueExA(hKey, name, NULL, &type, (LPBYTE)value, &size) == ERROR_SUCCESS) {
            RegCloseKey(hKey);
            return true;
        }
        RegCloseKey(hKey);
    }
    return false;
}

void LoadSettings(void)
{
    char ip[64] = DEFAULT_IP;
    char comPort[16] = "";
    DWORD port = DEFAULT_PORT;
    DWORD transport = WICAN_TRANSPORT_TCP;
    char portStr[16];
    
    ReadRegString(WICAN_REG_PATH, "IPAddress", ip, sizeof(ip));
    ReadRegDword(WICAN_REG_PATH, "Port", &port);
    ReadRegDword(WICAN_REG_PATH, "Transport", &transport);
    ReadRegString(WICAN_REG_PATH, "COMPort", comPort, sizeof(comPort));
    
    SetWindowTextA(g_hIpEdit, ip);
    sprintf(portStr, "%lu", port);
    SetWindowTextA(g_hPortEdit, portStr);
    
    g_transport = transport;
    SendMessage(g_hTransportCombo, CB_SETCURSEL, transport, 0);
    
    /* Select COM port in combo if it exists */
    if (comPort[0]) {
        int idx = (int)SendMessageA(g_hComCombo, CB_FINDSTRINGEXACT, -1, (LPARAM)comPort);
        if (idx != CB_ERR) {
            SendMessage(g_hComCombo, CB_SETCURSEL, idx, 0);
        }
    }
    
    UpdateTransportUI();
    SetStatus("Settings loaded");
}

void SaveSettings(void)
{
    char ip[64];
    char portStr[16];
    char comPort[16] = "";
    DWORD port;
    DWORD transport;
    int a, b, c, d;
    bool success;
    
    GetWindowTextA(g_hIpEdit, ip, sizeof(ip));
    GetWindowTextA(g_hPortEdit, portStr, sizeof(portStr));
    port = atoi(portStr);
    transport = (DWORD)SendMessage(g_hTransportCombo, CB_GETCURSEL, 0, 0);
    
    /* Get selected COM port */
    int comIdx = (int)SendMessage(g_hComCombo, CB_GETCURSEL, 0, 0);
    if (comIdx != CB_ERR) {
        SendMessageA(g_hComCombo, CB_GETLBTEXT, comIdx, (LPARAM)comPort);
    }
    
    /* Validate TCP settings */
    if (transport == WICAN_TRANSPORT_TCP) {
        if (sscanf(ip, "%d.%d.%d.%d", &a, &b, &c, &d) != 4 ||
            a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255 || d < 0 || d > 255) {
            SetStatus("Invalid IP address!");
            MessageBoxA(g_hWnd, "Please enter a valid IP address (e.g., 192.168.80.1)", "Invalid IP", MB_ICONERROR);
            return;
        }
        
        if (port < 1 || port > 65535) {
            SetStatus("Invalid port number!");
            MessageBoxA(g_hWnd, "Please enter a valid port (1-65535)", "Invalid Port", MB_ICONERROR);
            return;
        }
    } else {
        /* Validate USB settings */
        if (comPort[0] == '\0') {
            SetStatus("Select a COM port!");
            MessageBoxA(g_hWnd, "Please select a COM port for USB connection.", "No COM Port", MB_ICONERROR);
            return;
        }
    }
    
    success = true;
    success &= WriteRegString(WICAN_REG_PATH, "IPAddress", ip);
    success &= WriteRegDword(WICAN_REG_PATH, "Port", port);
    success &= WriteRegDword(WICAN_REG_PATH, "Transport", transport);
    success &= WriteRegString(WICAN_REG_PATH, "COMPort", comPort);
    success &= WriteRegString(WICAN_REG_PATH_WOW, "IPAddress", ip);
    success &= WriteRegDword(WICAN_REG_PATH_WOW, "Port", port);
    success &= WriteRegDword(WICAN_REG_PATH_WOW, "Transport", transport);
    success &= WriteRegString(WICAN_REG_PATH_WOW, "COMPort", comPort);
    
    if (success) {
        SetStatus("Settings saved!");
        if (transport == WICAN_TRANSPORT_TCP) {
            MessageBoxA(g_hWnd, "Settings saved!\n\nMake sure your PC is connected to the WiCAN WiFi network.", "Success", MB_ICONINFORMATION);
        } else {
            MessageBoxA(g_hWnd, "Settings saved!\n\nUSB mode selected. Connect WiCAN via USB cable.", "Success", MB_ICONINFORMATION);
        }
    } else {
        SetStatus("Failed - run as admin");
        MessageBoxA(g_hWnd, "Failed to save settings.\n\nPlease run this utility as Administrator.", "Error", MB_ICONERROR);
    }
}

void TestConnection(void)
{
    DWORD transport = (DWORD)SendMessage(g_hTransportCombo, CB_GETCURSEL, 0, 0);
    
    if (transport == WICAN_TRANSPORT_USB) {
        char comPort[16] = "";
        int comIdx = (int)SendMessage(g_hComCombo, CB_GETCURSEL, 0, 0);
        if (comIdx != CB_ERR) {
            SendMessageA(g_hComCombo, CB_GETLBTEXT, comIdx, (LPARAM)comPort);
        }
        if (comPort[0] == '\0') {
            SetStatus("Select a COM port first!");
            return;
        }
        TestUSBConnection(comPort);
    } else {
        /* TCP connection test */
        char ip[64];
        char portStr[16];
        DWORD port;
        WSADATA wsaData;
        SOCKET sock;
        struct sockaddr_in server;
        DWORD timeout;
        
        GetWindowTextA(g_hIpEdit, ip, sizeof(ip));
        GetWindowTextA(g_hPortEdit, portStr, sizeof(portStr));
        port = atoi(portStr);
        
        SetStatus("Testing TCP connection...");
        UpdateWindow(g_hStatusText);
        
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            SetStatus("Winsock init failed");
            return;
        }
        
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            SetStatus("Socket creation failed");
            WSACleanup();
            return;
        }
        
        timeout = 3000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
        
        memset(&server, 0, sizeof(server));
        server.sin_family = AF_INET;
        server.sin_port = htons((u_short)port);
        server.sin_addr.s_addr = inet_addr(ip);
        
        if (connect(sock, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
            closesocket(sock);
            WSACleanup();
            SetStatus("TCP Connection FAILED!");
            MessageBoxA(g_hWnd, 
                "Could not connect to WiCAN via WiFi!\n\n"
                "Please check:\n"
                "1. WiCAN is powered on\n"
                "2. PC is connected to WiCAN WiFi\n"
                "3. IP and port are correct",
                "Connection Failed", MB_ICONWARNING);
            return;
        }
        
        closesocket(sock);
        WSACleanup();
        
        SetStatus("TCP Connection SUCCESS!");
        MessageBoxA(g_hWnd, "Successfully connected to WiCAN via WiFi!", "Success", MB_ICONINFORMATION);
    }
}

void TestUSBConnection(const char *comPort)
{
    char portPath[32];
    HANDLE hSerial;
    DCB dcb;
    COMMTIMEOUTS timeouts;
    
    SetStatus("Testing USB connection...");
    UpdateWindow(g_hStatusText);
    
    sprintf(portPath, "\\\\.\\%s", comPort);
    hSerial = CreateFileA(portPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        DWORD err = GetLastError();
        char msg[256];
        if (err == ERROR_ACCESS_DENIED) {
            sprintf(msg, "COM port %s is in use by another application.", comPort);
        } else {
            sprintf(msg, "Could not open %s (Error %lu)", comPort, err);
        }
        SetStatus("USB Connection FAILED!");
        MessageBoxA(g_hWnd, msg, "Connection Failed", MB_ICONWARNING);
        return;
    }
    
    /* Configure serial port - get current state first, then modify only what's needed */
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial);
        SetStatus("USB GetCommState FAILED!");
        MessageBoxA(g_hWnd, "Could not get serial port state.", "Configuration Failed", MB_ICONWARNING);
        return;
    }
    
    /* CH342 driver doesn't support SetCommState properly - just verify current settings */
    if (dcb.BaudRate != 2000000) {
        char msg[256];
        sprintf(msg, "COM port is at %lu baud, expected 2000000.\nThe CH342 may need to be reconfigured.", dcb.BaudRate);
        CloseHandle(hSerial);
        SetStatus("USB Baud mismatch!");
        MessageBoxA(g_hWnd, msg, "Configuration Warning", MB_ICONWARNING);
        return;
    }
    
    /* Port is already configured correctly - skip SetCommState due to CH342 driver quirk */
    CloseHandle(hSerial);
    
    SetStatus("USB Connection SUCCESS!");
    MessageBoxA(g_hWnd, "COM port opened successfully at 2Mbaud!\n\nReady for J2534 communication.", 
               "Success", MB_ICONINFORMATION);
}

void SetDefaults(void)
{
    char portStr[16];
    SetWindowTextA(g_hIpEdit, DEFAULT_IP);
    sprintf(portStr, "%d", DEFAULT_PORT);
    SetWindowTextA(g_hPortEdit, portStr);
    SendMessage(g_hTransportCombo, CB_SETCURSEL, WICAN_TRANSPORT_TCP, 0);
    g_transport = WICAN_TRANSPORT_TCP;
    UpdateTransportUI();
    SetStatus("Defaults restored");
}

void SetStatus(const char *status)
{
    SetWindowTextA(g_hStatusText, status);
}

void UpdateTransportUI(void)
{
    DWORD transport = (DWORD)SendMessage(g_hTransportCombo, CB_GETCURSEL, 0, 0);
    g_transport = transport;
    
    if (transport == WICAN_TRANSPORT_USB) {
        /* USB mode - show COM port, hide IP/Port */
        ShowWindow(g_hIpEdit, SW_HIDE);
        ShowWindow(g_hPortEdit, SW_HIDE);
        ShowWindow(g_hIpLabel, SW_HIDE);
        ShowWindow(g_hPortLabel, SW_HIDE);
        ShowWindow(g_hComCombo, SW_SHOW);
        ShowWindow(g_hComLabel, SW_SHOW);
        ShowWindow(g_hRefreshBtn, SW_SHOW);
    } else {
        /* TCP/WiFi mode - show IP/Port, hide COM port */
        ShowWindow(g_hIpEdit, SW_SHOW);
        ShowWindow(g_hPortEdit, SW_SHOW);
        ShowWindow(g_hIpLabel, SW_SHOW);
        ShowWindow(g_hPortLabel, SW_SHOW);
        ShowWindow(g_hComCombo, SW_HIDE);
        ShowWindow(g_hComLabel, SW_HIDE);
        ShowWindow(g_hRefreshBtn, SW_HIDE);
    }
}

void RefreshComPorts(void)
{
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA devInfoData;
    DWORD i = 0;
    
    /* Clear existing items */
    SendMessage(g_hComCombo, CB_RESETCONTENT, 0, 0);
    
    SetStatus("Scanning COM ports...");
    UpdateWindow(g_hStatusText);
    
    hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, NULL, NULL, 
                                   DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        SetStatus("Failed to enumerate devices");
        return;
    }
    
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    
    while (SetupDiEnumDeviceInfo(hDevInfo, i++, &devInfoData)) {
        char friendlyName[256] = "";
        char comPort[16] = "";
        HKEY hKey;
        DWORD size;
        bool isWiCAN = false;
        
        /* Get friendly name */
        size = sizeof(friendlyName);
        SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                                          NULL, (PBYTE)friendlyName, size, NULL);
        
        /* Check if this is a CH342/CH34x device (WiCAN uses CH342) */
        if (strstr(friendlyName, "CH342") || strstr(friendlyName, "CH34") ||
            strstr(friendlyName, "WCH")) {
            isWiCAN = true;
        }
        
        /* Get COM port from registry */
        hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hKey != INVALID_HANDLE_VALUE) {
            size = sizeof(comPort);
            RegQueryValueExA(hKey, "PortName", NULL, NULL, (LPBYTE)comPort, &size);
            RegCloseKey(hKey);
        }
        
        if (comPort[0] && isWiCAN) {
            char displayText[128];
            sprintf(displayText, "%s - %s", comPort, friendlyName);
            
            /* Add to combo box - store just COM port name as item data */
            int idx = (int)SendMessageA(g_hComCombo, CB_ADDSTRING, 0, (LPARAM)comPort);
            (void)idx;
        }
    }
    
    SetupDiDestroyDeviceInfoList(hDevInfo);
    
    int count = (int)SendMessage(g_hComCombo, CB_GETCOUNT, 0, 0);
    if (count == 0) {
        SendMessageA(g_hComCombo, CB_ADDSTRING, 0, (LPARAM)"(No WiCAN found)");
        SetStatus("No WiCAN USB devices found");
    } else {
        SendMessage(g_hComCombo, CB_SETCURSEL, 0, 0);
        char status[64];
        sprintf(status, "Found %d WiCAN port(s)", count);
        SetStatus(status);
    }
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    HFONT hFont;
    HFONT hFontBold;
    
    switch (msg) {
        case WM_CREATE:
            hFont = CreateFontA(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, "Segoe UI");
            
            hFontBold = CreateFontA(16, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, "Segoe UI");
            
            /* Title */
            {
                HWND hTitle = CreateWindowA("STATIC", "WiCAN J2534 Configuration",
                    WS_VISIBLE | WS_CHILD | SS_CENTER,
                    10, 10, 360, 25, hWnd, NULL, NULL, NULL);
                SendMessage(hTitle, WM_SETFONT, (WPARAM)hFontBold, TRUE);
            }
            
            /* Transport selection */
            {
                HWND hTransportLabel = CreateWindowA("STATIC", "Connection Type:",
                    WS_VISIBLE | WS_CHILD, 20, 45, 120, 20, hWnd, NULL, NULL, NULL);
                SendMessage(hTransportLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            g_hTransportCombo = CreateWindowA("COMBOBOX", "",
                WS_VISIBLE | WS_CHILD | WS_TABSTOP | CBS_DROPDOWNLIST,
                140, 42, 170, 100, hWnd, (HMENU)IDC_TRANSPORT_COMBO, NULL, NULL);
            SendMessage(g_hTransportCombo, WM_SETFONT, (WPARAM)hFont, TRUE);
            SendMessageA(g_hTransportCombo, CB_ADDSTRING, 0, (LPARAM)"WiFi / TCP");
            SendMessageA(g_hTransportCombo, CB_ADDSTRING, 0, (LPARAM)"USB Serial");
            SendMessage(g_hTransportCombo, CB_SETCURSEL, 0, 0);
            
            /* TCP settings - IP Address */
            g_hIpLabel = CreateWindowA("STATIC", "WiCAN IP Address:",
                WS_VISIBLE | WS_CHILD, 20, 75, 150, 20, hWnd, NULL, NULL, NULL);
            SendMessage(g_hIpLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            g_hIpEdit = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", DEFAULT_IP,
                WS_VISIBLE | WS_CHILD | WS_TABSTOP | ES_AUTOHSCROLL,
                20, 97, 200, 25, hWnd, (HMENU)IDC_IP_EDIT, NULL, NULL);
            SendMessage(g_hIpEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            /* TCP settings - Port */
            g_hPortLabel = CreateWindowA("STATIC", "Port:",
                WS_VISIBLE | WS_CHILD, 240, 75, 50, 20, hWnd, NULL, NULL, NULL);
            SendMessage(g_hPortLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            g_hPortEdit = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "3333",
                WS_VISIBLE | WS_CHILD | WS_TABSTOP | ES_NUMBER,
                240, 97, 70, 25, hWnd, (HMENU)IDC_PORT_EDIT, NULL, NULL);
            SendMessage(g_hPortEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            /* USB settings - COM port (initially hidden) */
            g_hComLabel = CreateWindowA("STATIC", "USB COM Port:",
                WS_CHILD, 20, 75, 150, 20, hWnd, NULL, NULL, NULL);  /* Not visible initially */
            SendMessage(g_hComLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            g_hComCombo = CreateWindowA("COMBOBOX", "",
                WS_CHILD | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
                20, 97, 200, 150, hWnd, (HMENU)IDC_COM_COMBO, NULL, NULL);
            SendMessage(g_hComCombo, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            g_hRefreshBtn = CreateWindowA("BUTTON", "Refresh",
                WS_CHILD | WS_TABSTOP | BS_PUSHBUTTON,
                230, 97, 80, 25, hWnd, (HMENU)IDC_REFRESH_BTN, NULL, NULL);
            SendMessage(g_hRefreshBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            /* Info text */
            {
                HWND hInfo = CreateWindowA("STATIC", 
                    "WiFi: Connect to WiCAN WiFi network (192.168.80.1:3333)\n"
                    "USB: Connect WiCAN via USB cable (faster, more reliable)",
                    WS_VISIBLE | WS_CHILD, 20, 130, 340, 35, hWnd, NULL, NULL, NULL);
                SendMessage(hInfo, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            /* Buttons */
            {
                HWND hTestBtn = CreateWindowA("BUTTON", "Test Connection",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_PUSHBUTTON,
                    20, 175, 130, 30, hWnd, (HMENU)IDC_TEST_BTN, NULL, NULL);
                SendMessage(hTestBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            {
                HWND hDefaultBtn = CreateWindowA("BUTTON", "Defaults",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_PUSHBUTTON,
                    160, 175, 80, 30, hWnd, (HMENU)IDC_DEFAULT_BTN, NULL, NULL);
                SendMessage(hDefaultBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            {
                HWND hSaveBtn = CreateWindowA("BUTTON", "Save Settings",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_DEFPUSHBUTTON,
                    250, 175, 110, 30, hWnd, (HMENU)IDC_SAVE_BTN, NULL, NULL);
                SendMessage(hSaveBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            g_hStatusText = CreateWindowA("STATIC", "Ready",
                WS_VISIBLE | WS_CHILD | SS_CENTER,
                20, 215, 340, 20, hWnd, (HMENU)IDC_STATUS_TEXT, NULL, NULL);
            SendMessage(g_hStatusText, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            /* Initialize COM port list */
            RefreshComPorts();
            
            LoadSettings();
            break;
        
        case WM_COMMAND:
            switch (LOWORD(wParam)) {
                case IDC_SAVE_BTN:
                    SaveSettings();
                    break;
                case IDC_TEST_BTN:
                    TestConnection();
                    break;
                case IDC_DEFAULT_BTN:
                    SetDefaults();
                    break;
                case IDC_REFRESH_BTN:
                    RefreshComPorts();
                    break;
                case IDC_TRANSPORT_COMBO:
                    if (HIWORD(wParam) == CBN_SELCHANGE) {
                        UpdateTransportUI();
                    }
                    break;
            }
            break;
            
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;
            
        case WM_DESTROY:
            PostQuitMessage(0);
            break;
            
        default:
            return DefWindowProcA(hWnd, msg, wParam, lParam);
    }
    return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    WNDCLASSEXA wc = {0};
    MSG msg;
    INITCOMMONCONTROLSEX icex;
    
    icex.dwSize = sizeof(icex);
    icex.dwICC = ICC_STANDARD_CLASSES;
    InitCommonControlsEx(&icex);
    
    wc.cbSize = sizeof(WNDCLASSEXA);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW);
    wc.lpszClassName = "WiCANConfig";
    wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
    
    if (!RegisterClassExA(&wc)) {
        MessageBoxA(NULL, "Window Registration Failed!", "Error", MB_ICONERROR);
        return 0;
    }
    
    g_hWnd = CreateWindowExA(0, "WiCANConfig", "WiCAN J2534 Configuration",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT, 395, 290,
        NULL, NULL, hInstance, NULL);
    
    if (!g_hWnd) {
        MessageBoxA(NULL, "Window Creation Failed!", "Error", MB_ICONERROR);
        return 0;
    }
    
    ShowWindow(g_hWnd, nCmdShow);
    UpdateWindow(g_hWnd);
    
    while (GetMessage(&msg, NULL, 0, 0) > 0) {
        if (!IsDialogMessage(g_hWnd, &msg)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }
    
    return (int)msg.wParam;
}
