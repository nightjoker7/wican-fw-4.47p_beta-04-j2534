/**
 * @file j2534_debug_test.c
 * @brief Debug test for WiCAN J2534 communication
 */

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#pragma comment(lib, "ws2_32.lib")

#define WICAN_SYNC_BYTE1    0x55
#define WICAN_SYNC_BYTE2    0xAA
#define WICAN_CMD_OPEN      0x0D
#define WICAN_CMD_CLOSE     0x0E
#define WICAN_CMD_CONNECT   0x01

static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void send_command(SOCKET sock, uint8_t cmd, const uint8_t *data, uint16_t data_len, const char *name)
{
    uint8_t packet[256];
    packet[0] = WICAN_SYNC_BYTE1;
    packet[1] = WICAN_SYNC_BYTE2;
    packet[2] = cmd;
    packet[3] = (data_len >> 8) & 0xFF;
    packet[4] = data_len & 0xFF;
    if (data && data_len > 0) {
        memcpy(&packet[5], data, data_len);
    }
    packet[5 + data_len] = calc_checksum(packet, 5 + data_len);
    
    printf("\nSending %s command:\n", name);
    for (int i = 0; i < 6 + data_len; i++) {
        printf("%02X ", packet[i]);
    }
    printf("\n");
    
    int sent = send(sock, (const char*)packet, 6 + data_len, 0);
    printf("Sent %d bytes\n", sent);
    
    // Wait and receive response
    Sleep(100);
    
    uint8_t resp[256];
    int recv_result = recv(sock, (char*)resp, sizeof(resp), 0);
    
    if (recv_result > 0) {
        printf("Received %d bytes:\n", recv_result);
        for (int i = 0; i < recv_result; i++) {
            printf("%02X ", resp[i]);
        }
        printf("\n");
        
        if (recv_result >= 6) {
            printf("  CMD: 0x%02X, STATUS: 0x%02X", resp[2], resp[3]);
            if (resp[3] == 0x00) {
                printf(" (SUCCESS)\n");
            } else {
                printf(" (ERROR: 0x%02X)\n", resp[3]);
            }
            
            uint16_t resp_len = (resp[4] << 8) | resp[5];
            if (resp_len > 0 && recv_result >= 6 + resp_len) {
                printf("  Data: ");
                for (int i = 0; i < resp_len; i++) {
                    printf("%02X ", resp[6 + i]);
                }
                printf("\n");
            }
        }
    } else {
        printf("No response or error: %d\n", WSAGetLastError());
    }
}

int main(int argc, char *argv[])
{
    WSADATA wsa;
    SOCKET sock;
    struct sockaddr_in server;
    char *ip = "192.168.0.24";
    int port = 3333;
    
    if (argc > 1) ip = argv[1];
    if (argc > 2) port = atoi(argv[2]);
    
    printf("WiCAN J2534 Debug Test\n");
    printf("Connecting to %s:%d\n", ip, port);
    
    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        printf("WSAStartup failed: %d\n", WSAGetLastError());
        return 1;
    }
    
    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        printf("Socket creation failed: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
    
    // Set timeout
    DWORD timeout = 5000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    
    // Connect
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, ip, &server.sin_addr);
    
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        printf("Connect failed: %d\n", WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }
    
    printf("Connected!\n");
    
    // First send CLOSE to ensure clean state (uses device_id = 0x820F021C from previous)
    uint8_t close_data[4] = {0x82, 0x0F, 0x02, 0x1C};  // device_id
    send_command(sock, WICAN_CMD_CLOSE, close_data, 4, "CLOSE (reset)");
    
    // Now send OPEN command
    send_command(sock, WICAN_CMD_OPEN, NULL, 0, "OPEN");
    
    // Try CONNECT to CAN bus (protocol=5=CAN, flags=0, baud=500000)
    uint8_t connect_data[16];
    uint32_t device_id = 0x820F021C;  // From OPEN response
    uint32_t protocol = 5;  // CAN
    uint32_t flags = 0;
    uint32_t baudrate = 500000;
    
    // Pack as big-endian
    connect_data[0] = (device_id >> 24) & 0xFF;
    connect_data[1] = (device_id >> 16) & 0xFF;
    connect_data[2] = (device_id >> 8) & 0xFF;
    connect_data[3] = device_id & 0xFF;
    connect_data[4] = (protocol >> 24) & 0xFF;
    connect_data[5] = (protocol >> 16) & 0xFF;
    connect_data[6] = (protocol >> 8) & 0xFF;
    connect_data[7] = protocol & 0xFF;
    connect_data[8] = (flags >> 24) & 0xFF;
    connect_data[9] = (flags >> 16) & 0xFF;
    connect_data[10] = (flags >> 8) & 0xFF;
    connect_data[11] = flags & 0xFF;
    connect_data[12] = (baudrate >> 24) & 0xFF;
    connect_data[13] = (baudrate >> 16) & 0xFF;
    connect_data[14] = (baudrate >> 8) & 0xFF;
    connect_data[15] = baudrate & 0xFF;
    
    send_command(sock, WICAN_CMD_CONNECT, connect_data, 16, "CONNECT");
    
    closesocket(sock);
    WSACleanup();
    
    printf("\nPress Enter to exit...\n");
    getchar();
    
    return 0;
}
