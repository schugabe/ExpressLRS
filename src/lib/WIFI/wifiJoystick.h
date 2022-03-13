#pragma once

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include <WiFiUdp.h>

#define HAS_WIFI_JOYSTICK 1
#define JOYSTICK_PORT 11000

class WifiJoystick
{
public:
    static void StartJoystickService();
    static void StopJoystickService();
    static bool CheckForConnection();
    static void UpdateValues();
private:
    static bool IsValidRequest(char *buffer);

    static bool running;
    static bool changedSettings;
    static WiFiUDP udp;
    static IPAddress remoteIP;
    static int remotePort;
};

#endif
