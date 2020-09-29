#include "Telemetry.h"
#include <CRSF.h>
#include "../../src/targets.h"

extern CRSF crsf;

uint8_t UARTinPacketPtr;
uint8_t UARTinPacketLen;
uint32_t UARTinLastDataTime;
uint32_t UARTinLastPacketTime;
bool UARTframeActive = false;

uint8_t UARTinBuffer[256];

void ProcessTelemetryPacket()
{
    if (UARTinBuffer[2] == CRSF_FRAMETYPE_COMMAND)
    {
        Serial.println("Got CMD Packet");
        #ifdef PLATFORM_STM32
            if (UARTinBuffer[3] == 0x62 && UARTinBuffer[4] == 0x6c)
            {
                delay(100);
                Serial.println("Jumping to Bootloader...");
                delay(100);
                HAL_NVIC_SystemReset();
            }
        #endif
    }

    if (UARTinBuffer[2] == CRSF_FRAMETYPE_BATTERY_SENSOR)
    {
        crsf.TLMbattSensor.voltage = (UARTinBuffer[3] << 8) + UARTinBuffer[4];
        crsf.TLMbattSensor.current = (UARTinBuffer[5] << 8) + UARTinBuffer[6];
        crsf.TLMbattSensor.capacity = (UARTinBuffer[7] << 16) + (UARTinBuffer[8] << 8) + UARTinBuffer[9];
        crsf.TLMbattSensor.remaining = UARTinBuffer[9];
    }
}

void RX_Telemetry()
{
    while (Serial.available())
    {
        UARTinLastDataTime = millis();
        char inChar = Serial.read();

        if (UARTframeActive == false)
        {
            // stage 1 wait for sync byte //
            if (inChar == CRSF_ADDRESS_CRSF_TRANSMITTER || inChar == CRSF_SYNC_BYTE) // we got sync, reset write pointer
            {
                UARTinPacketPtr = 0;
                UARTinPacketLen = 0;
                UARTframeActive = true;
                UARTinBuffer[UARTinPacketPtr] = inChar;
                UARTinPacketPtr++;
            }
        }
        else // frame is active so we do the processing
        {
            // first if things have gone wrong //
            if (UARTinPacketPtr > CRSF_MAX_PACKET_LEN - 1) // we reached the maximum allowable packet length, so start again because shit fucked up hey.
            {
                UARTinPacketPtr = 0;
                UARTinPacketLen = 0;
                UARTframeActive = false;
                return;
            }
            // special case where we save the expected pkt len to buffer //
            if (UARTinPacketPtr == 1)
            {
                if (inChar <= CRSF_MAX_PACKET_LEN)
                {
                    UARTinPacketLen = inChar;
                }
                else
                {
                    UARTinPacketPtr = 0;
                    UARTinPacketLen = 0;
                    UARTframeActive = false;
                    return;
                }
            }

            UARTinBuffer[UARTinPacketPtr] = inChar;
            UARTinPacketPtr++;

            if (UARTinPacketPtr == UARTinPacketLen + 2) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
            {
                char CalculatedCRC = CalcCRC((uint8_t *)UARTinBuffer + 2, UARTinPacketPtr - 3);

                if (CalculatedCRC == inChar)
                {
                    UARTinLastPacketTime = millis();
                    ProcessTelemetryPacket();
                    UARTinPacketPtr = 0;
                    UARTinPacketLen = 0;
                    UARTframeActive = false;
                }
                else
                {
                    UARTinPacketPtr = 0;
                    UARTinPacketLen = 0;
                    UARTframeActive = false;
                    Serial.println("UART in CRC failure");
                    while (Serial.available())
                    {
                        Serial.read(); // empty the read buffer
                    }
                }
            }
        }
    }
}