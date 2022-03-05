#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include <WiFiUdp.h>

#define HAS_WIFI_JOYSTICK 1

#define SEND_TIMEOUT_MS 20000
#define MAX_MESSAGES_BETWEEN_PING 1000
#define JOYSTICK_PORT 11000

class WifiJoystick
{
public:
    WifiJoystick(int port);
    void StartJoystickService();
    void StopJoystickService();
    void Update();

private:
    bool IsValidRequest(char *buffer);
    void SendMessage();

    bool running;
    uint16_t messagesUntilTimeout;
    uint32_t lastMessageSent;
    WiFiUDP udp;
    IPAddress remoteIP;
    int remotePort;
    int listPort;
};

#endif
