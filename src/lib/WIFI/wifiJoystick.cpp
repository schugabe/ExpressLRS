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
bool WifiJoystick::startedEvent = false;
WiFiUDP *WifiJoystick::udp = NULL;
IPAddress WifiJoystick::remoteIP;

void WifiJoystick::StartJoystickService()
{
    if (running)
    {
        return;
    }
    if (!udp)
    {
        udp = new WiFiUDP();
    }

    running = udp->begin(JOYSTICK_PORT) > 0;
}

void WifiJoystick::StopJoystickService()
{
    udp->stop();
    running = false;
}

void WifiJoystick::StartSending(IPAddress ip, uint32_t updateInterval)
{
    if (!running)
    {
        return;
    }

    remoteIP = ip;

    uint8_t ReplyBuffer[] = "acknowledged";
    udp->beginPacket(remoteIP, JOYSTICK_PORT);
    udp->write(ReplyBuffer, sizeof(ReplyBuffer));
    udp->endPacket();

    hwTimer::updateInterval(updateInterval);
    CRSF::setSyncParams(updateInterval);
    CRSF::disableOpentxSync();
    POWERMGNT::setPower(MinPower);
    Radio.End();
    CRSF::RCdataCallback = UpdateValues;

    startedEvent = true;
}

bool WifiJoystick::CheckForConnection()
{
    if (running && startedEvent)
    {
        startedEvent = false;
        return true;
    }

    return false;
}

void WifiJoystick::UpdateValues()
{
    if (!running)
    {
        return;
    }

    // TODO send as crsf message
    udp->beginPacket(remoteIP, JOYSTICK_PORT);
    for (uint8_t i = 0; i < 16; i++)
    {
        udp->write((uint8_t*)&CRSF::ChannelDataIn[i], 2);
    }
    udp->endPacket();
}

#endif
