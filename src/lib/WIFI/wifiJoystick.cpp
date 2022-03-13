#include "device.h"
#include "wifiJoystick.h"

#if defined(HAS_WIFI_JOYSTICK)
#include <WiFiUdp.h>
#include "CRSF.h"
#include "POWERMGNT.h"
#include "hwTimer.h"
#include "logging.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_IN_866) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
extern SX127xDriver Radio;
#elif defined(Regulatory_Domain_ISM_2400)
extern SX1280Driver Radio;
#endif

bool WifiJoystick::running = false;
bool WifiJoystick::changedSettings = false;
WiFiUDP WifiJoystick::udp;
IPAddress WifiJoystick::remoteIP;
int WifiJoystick::remotePort;

void WifiJoystick::StartJoystickService()
{
    running = udp.begin(JOYSTICK_PORT) > 0;
}

void WifiJoystick::StopJoystickService()
{
    udp.stop();
    running = false;
}

bool WifiJoystick::CheckForConnection()
{
    int packetSize;
    char packetBuffer[CRSF_MAX_PACKET_LEN];

    if (!running)
    {
        return false;
    }

    packetSize = udp.parsePacket();
    if (packetSize)
    {
        int len = udp.read(packetBuffer, sizeof(packetBuffer));
        if (len > 0) packetBuffer[len-1] = 0;

        if (IsValidRequest(packetBuffer))
        {
            remoteIP = udp.remoteIP();
            remotePort = udp.remotePort();

            if (!changedSettings)
            {
                changedSettings = true;

                hwTimer::updateInterval(10000);
                CRSF::setSyncParams(10000);
                CRSF::disableOpentxSync();
                POWERMGNT::setPower(MinPower);
                Radio.End();
                CRSF::RCdataCallback = UpdateValues;

                return true;
            }
        }
    }

    return false;
}

void WifiJoystick::UpdateValues()
{
    static uint32_t counter = 0;
    static uint32_t lastMessage = 0;

    uint32_t now = micros();
    uint32_t diff = now - lastMessage;

    if (!running)
    {
        return;
    }

    lastMessage = now;

    // TODO send as crsf message
    udp.beginPacket(remoteIP, remotePort);
    for (uint8_t i = 0; i < 8; i++)
    {
        udp.write((uint8_t*)&CRSF::ChannelDataIn[i], 2);
    }
    udp.write((uint8_t*)&diff, 4);
    udp.write((uint8_t*)&counter, 4);
    udp.endPacket();
    counter++;
}

bool WifiJoystick::IsValidRequest(char* request)
{
    // TODO check if message is a valid crsf ping
    return true;
}

#endif
