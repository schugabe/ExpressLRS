#include "device.h"
#include "wifiJoystick.h"

#if defined(HAS_WIFI_JOYSTICK)
#include <WiFiUdp.h>
#include "CRSF.h"

WifiJoystick::WifiJoystick(int port)
{
    listPort = port;
    running = false;
}

void WifiJoystick::StartJoystickService()
{
    running = udp.begin(listPort) > 0;
}

void WifiJoystick::StopJoystickService()
{
    udp.stop();
    running = false;
}

void WifiJoystick::Update()
{
    int packetSize;
    char packetBuffer[255];
    uint32_t now = micros();

    if (!running)
    {
        return;
    }

    packetSize = udp.parsePacket();
    if (packetSize)
    {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len-1] = 0;

        if (IsValidRequest(packetBuffer))
        {
            remoteIP = udp.remoteIP();
            remotePort = udp.remotePort();
            messagesUntilTimeout = MAX_MESSAGES_BETWEEN_PING;
        }
    }

    if (now - lastMessageSent > SEND_TIMEOUT_MS && messagesUntilTimeout > 0)
    {
        lastMessageSent = now;
        messagesUntilTimeout--;
        SendMessage();
    }
}

void WifiJoystick::SendMessage()
{
    // TODO send as crsf message
    udp.beginPacket(remoteIP, remotePort);
    for (uint8_t i = 0; i < 16; i++)
    {
        udp.write((uint8_t*)&CRSF::ChannelDataIn[i], 2);
    }
    udp.endPacket();
}

bool WifiJoystick::IsValidRequest(char* request)
{
    // TODO check if message is a valid crsf ping
    return true;
}

#endif
