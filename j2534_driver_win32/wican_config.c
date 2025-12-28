/**
 * @file wican_config.c
 * @brief WiCAN J2534 Configuration Utility
 */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <commctrl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

#define WICAN_REG_PATH      "SOFTWARE\\WiCAN"
#define WICAN_REG_PATH_WOW  "SOFTWARE\\WOW6432Node\\WiCAN"

#define IDC_IP_EDIT         101
#define IDC_PORT_EDIT       102
#define IDC_SAVE_BTN        103
#define IDC_TEST_BTN        104
#define IDC_STATUS_TEXT     105
#define IDC_DEFAULT_BTN     106

#define DEFAULT_IP          "192.168.80.1"
#define DEFAULT_PORT        3333

HWND g_hWnd = NULL;
HWND g_hIpEdit = NULL;
HWND g_hPortEdit = NULL;
HWND g_hStatusText = NULL;

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
void LoadSettings(void);
void SaveSettings(void);
void TestConnection(void);
void SetDefaults(void);
void SetStatus(const char *status);

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
    DWORD port = DEFAULT_PORT;
    char portStr[16];
    
    ReadRegString(WICAN_REG_PATH, "IPAddress", ip, sizeof(ip));
    ReadRegDword(WICAN_REG_PATH, "Port", &port);
    
    SetWindowTextA(g_hIpEdit, ip);
    sprintf(portStr, "%lu", port);
    SetWindowTextA(g_hPortEdit, portStr);
    SetStatus("Settings loaded");
}

void SaveSettings(void)
{
    char ip[64];
    char portStr[16];
    DWORD port;
    int a, b, c, d;
    bool success;
    
    GetWindowTextA(g_hIpEdit, ip, sizeof(ip));
    GetWindowTextA(g_hPortEdit, portStr, sizeof(portStr));
    port = atoi(portStr);
    
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
    
    success = true;
    success &= WriteRegString(WICAN_REG_PATH, "IPAddress", ip);
    success &= WriteRegDword(WICAN_REG_PATH, "Port", port);
    success &= WriteRegString(WICAN_REG_PATH_WOW, "IPAddress", ip);
    success &= WriteRegDword(WICAN_REG_PATH_WOW, "Port", port);
    
    if (success) {
        SetStatus("Settings saved!");
        MessageBoxA(g_hWnd, "Settings saved!\n\nMake sure your PC is connected to the WiCAN WiFi network.", "Success", MB_ICONINFORMATION);
    } else {
        SetStatus("Failed - run as admin");
        MessageBoxA(g_hWnd, "Failed to save settings.\n\nPlease run this utility as Administrator.", "Error", MB_ICONERROR);
    }
}

void TestConnection(void)
{
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
    
    SetStatus("Testing connection...");
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
        SetStatus("Connection FAILED!");
        MessageBoxA(g_hWnd, 
            "Could not connect to WiCAN!\n\n"
            "Please check:\n"
            "1. WiCAN is powered on\n"
            "2. PC is connected to WiCAN WiFi\n"
            "3. IP and port are correct",
            "Connection Failed", MB_ICONWARNING);
        return;
    }
    
    closesocket(sock);
    WSACleanup();
    
    SetStatus("Connection SUCCESS!");
    MessageBoxA(g_hWnd, "Successfully connected to WiCAN!", "Success", MB_ICONINFORMATION);
}

void SetDefaults(void)
{
    char portStr[16];
    SetWindowTextA(g_hIpEdit, DEFAULT_IP);
    sprintf(portStr, "%d", DEFAULT_PORT);
    SetWindowTextA(g_hPortEdit, portStr);
    SetStatus("Defaults restored");
}

void SetStatus(const char *status)
{
    SetWindowTextA(g_hStatusText, status);
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
            
            {
                HWND hTitle = CreateWindowA("STATIC", "WiCAN J2534 Configuration",
                    WS_VISIBLE | WS_CHILD | SS_CENTER,
                    10, 10, 360, 25, hWnd, NULL, NULL, NULL);
                SendMessage(hTitle, WM_SETFONT, (WPARAM)hFontBold, TRUE);
            }
            
            {
                HWND hIpLabel = CreateWindowA("STATIC", "WiCAN IP Address:",
                    WS_VISIBLE | WS_CHILD, 20, 50, 150, 20, hWnd, NULL, NULL, NULL);
                SendMessage(hIpLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            g_hIpEdit = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", DEFAULT_IP,
                WS_VISIBLE | WS_CHILD | WS_TABSTOP | ES_AUTOHSCROLL,
                20, 72, 200, 25, hWnd, (HMENU)IDC_IP_EDIT, NULL, NULL);
            SendMessage(g_hIpEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            {
                HWND hPortLabel = CreateWindowA("STATIC", "Port:",
                    WS_VISIBLE | WS_CHILD, 240, 50, 50, 20, hWnd, NULL, NULL, NULL);
                SendMessage(hPortLabel, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            g_hPortEdit = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "3333",
                WS_VISIBLE | WS_CHILD | WS_TABSTOP | ES_NUMBER,
                240, 72, 70, 25, hWnd, (HMENU)IDC_PORT_EDIT, NULL, NULL);
            SendMessage(g_hPortEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
            
            {
                HWND hInfo = CreateWindowA("STATIC", 
                    "Default: 192.168.80.1:3333 (WiCAN AP mode)\n"
                    "Connect your PC to WiCAN WiFi before testing.",
                    WS_VISIBLE | WS_CHILD, 20, 105, 340, 35, hWnd, NULL, NULL, NULL);
                SendMessage(hInfo, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            {
                HWND hTestBtn = CreateWindowA("BUTTON", "Test Connection",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_PUSHBUTTON,
                    20, 150, 130, 30, hWnd, (HMENU)IDC_TEST_BTN, NULL, NULL);
                SendMessage(hTestBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            {
                HWND hDefaultBtn = CreateWindowA("BUTTON", "Defaults",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_PUSHBUTTON,
                    160, 150, 80, 30, hWnd, (HMENU)IDC_DEFAULT_BTN, NULL, NULL);
                SendMessage(hDefaultBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            {
                HWND hSaveBtn = CreateWindowA("BUTTON", "Save Settings",
                    WS_VISIBLE | WS_CHILD | WS_TABSTOP | BS_DEFPUSHBUTTON,
                    250, 150, 110, 30, hWnd, (HMENU)IDC_SAVE_BTN, NULL, NULL);
                SendMessage(hSaveBtn, WM_SETFONT, (WPARAM)hFont, TRUE);
            }
            
            g_hStatusText = CreateWindowA("STATIC", "Ready",
                WS_VISIBLE | WS_CHILD | SS_CENTER,
                20, 195, 340, 20, hWnd, (HMENU)IDC_STATUS_TEXT, NULL, NULL);
            SendMessage(g_hStatusText, WM_SETFONT, (WPARAM)hFont, TRUE);
            
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
        CW_USEDEFAULT, CW_USEDEFAULT, 395, 265,
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
