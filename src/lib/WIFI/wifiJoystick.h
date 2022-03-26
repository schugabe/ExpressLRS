#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include <WiFiUdp.h>

#define HAS_WIFI_JOYSTICK 1
#define JOYSTICK_PORT 11000
#define JOYSTICK_DEFAULT_UPDATE_INTERVAL 10000

class WifiJoystick
{
public:
    static void StartJoystickService();
    static void StopJoystickService();
    static void UpdateValues();
    static void StartSending(IPAddress ip, uint32_t updateInterval);
    static bool CheckForConnection();
private:
    static bool running;
    static bool startedEvent;
    static WiFiUDP *udp;
    static IPAddress remoteIP;
};

#endif
