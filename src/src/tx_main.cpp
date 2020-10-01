#include <Arduino.h>
#include "FIFO.h"
#include "utils.h"
#include "common.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
SX127xDriver Radio;
#elif Regulatory_Domain_ISM_2400
#include "SX1280Driver.h"
SX1280Driver Radio;
#endif

#include "CRSF.h"
#include "FHSS.h"
#include "LED.h"
// #include "debug.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "msp.h"
#include "msptypes.h"
#include <OTA.h>
//#include "elrs_eeprom.h"
#include "hwTimer.h"
#include "LQCALC.h"
#include "LowPassFilter.h"

#ifdef PLATFORM_ESP8266
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#ifdef PLATFORM_ESP32
#include "ESP32_WebUpdate.h"
#endif

#ifdef TARGET_R9M_TX
#include "DAC.h"
#include "button.h"
button button;
R9DAC R9DAC;
#endif

#if defined(TARGET_R9M_LITE_TX) || (TARGET_R9M_LITE_PRO_TX)
#include "STM32_hwTimer.h"
#endif
const uint8_t thisCommit[6] = {LATEST_COMMIT};

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT 3000 // After 1500ms of no TLM response consider that slave has lost connection
#define MSP_PACKET_SEND_INTERVAL 200

/// define some libs to use ///
hwTimer hwTimer;
CRSF crsf;
POWERMGNT POWERMGNT;
MSP msp;

void ICACHE_RAM_ATTR TimerCallbackISR();
volatile uint8_t NonceTX;

bool webUpdateMode = false;

//// MSP Data Handling ///////
uint32_t MSPPacketLastSent = 0;  // time in ms when the last switch data packet was sent
uint32_t MSPPacketSendCount = 0; // number of times to send MSP packet
mspPacket_t MSPPacket;

////////////SYNC PACKET/////////
uint32_t SyncPacketLastSent = 0;

uint32_t LastTLMpacketRecvMillis = 0;
LQCALC LQCALC;
LPF LPD_DownlinkLQ(1);

volatile bool UpdateParamReq = false;
uint8_t luaCommitPacket[] = {(uint8_t)0xF0, thisCommit[0], thisCommit[1], thisCommit[2]};
uint8_t luaCommitOtherHalfPacket[] = {(uint8_t)0xF1, thisCommit[3], thisCommit[4], thisCommit[5]};

uint32_t PacketLastSentMicros = 0;

bool Channels5to8Changed = false;

bool WaitRXresponse = false;

// MSP packet handling function defs
void ProcessMSPPacket(mspPacket_t *packet);
void OnRFModePacket(mspPacket_t *packet);
void OnTxPowerPacket(mspPacket_t *packet);
void OnTLMRatePacket(mspPacket_t *packet);

uint8_t baseMac[6];

void ICACHE_RAM_ATTR ProcessTLMpacket()
{
  uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
  uint8_t inCRC = Radio.RXdataBuffer[7];
  uint8_t type = Radio.RXdataBuffer[0] & TLM_PACKET;
  uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;
  uint8_t TLMheader = Radio.RXdataBuffer[1];
  //Serial.println("TLMpacket0");

  if (packetAddr != DeviceAddr)
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM device address error");
#endif
    return;
  }

  if ((inCRC != calculatedCRC))
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM crc error");
#endif
    return;
  }

  if (type != TLM_PACKET)
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM type error");
    Serial.println(type);
#endif
    return;
  }

  if (connectionState != connected)
  {
    connectionState = connected;
    LPD_DownlinkLQ.init(100);
    Serial.println("got downlink conn");
  }

  LastTLMpacketRecvMillis = millis();
  LQCALC.add();

  if (TLMheader == CRSF_FRAMETYPE_LINK_STATISTICS)
  {
    crsf.LinkStatistics.uplink_RSSI_1 = Radio.RXdataBuffer[2];
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.uplink_SNR = Radio.RXdataBuffer[4];
    crsf.LinkStatistics.uplink_Link_quality = Radio.RXdataBuffer[5];

    crsf.LinkStatistics.downlink_SNR = int(Radio.LastPacketSNR * 10);
    crsf.LinkStatistics.downlink_RSSI = 120 + Radio.LastPacketRSSI;
    crsf.LinkStatistics.downlink_Link_quality = LPD_DownlinkLQ.update(LQCALC.getLQ()) + 1; // +1 fixes rounding issues with filter and makes it consistent with RX LQ Calculation
    //crsf.LinkStatistics.downlink_Link_quality = Radio.currPWR;
    crsf.LinkStatistics.rf_Mode = 4 - ExpressLRS_currAirRate_Modparams->index;

    crsf.TLMbattSensor.voltage = (Radio.RXdataBuffer[3] << 8) + Radio.RXdataBuffer[6];

    crsf.sendLinkStatisticsToTX();
    crsf.sendLinkBattSensorToTX();
  }
}

void ICACHE_RAM_ATTR CheckChannels5to8Change()
{ //check if channels 5 to 8 have new data (switch channels)
  for (int i = 4; i < 8; i++)
  {
    if (crsf.ChannelDataInPrev[i] != crsf.ChannelDataIn[i])
    {
      Channels5to8Changed = true;
    }
  }
}

void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
  uint8_t PacketHeaderAddr;
  uint8_t index = (ExpressLRS_currAirRate_Modparams->index & 0b11);
  uint8_t TLmrate = (ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111);
  PacketHeaderAddr = (DeviceAddr << 2) + SYNC_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
  Radio.TXdataBuffer[2] = NonceTX;
  Radio.TXdataBuffer[3] = (index << 6) + (TLmrate << 3);
  Radio.TXdataBuffer[4] = UID[3];
  Radio.TXdataBuffer[5] = UID[4];
  Radio.TXdataBuffer[6] = UID[5];
}

void ICACHE_RAM_ATTR Generate4ChannelData_10bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[2] = ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[3] = ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[4] = ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[5] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b0000000011) << 6) +
                          ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b0000000011) << 4) +
                          ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b0000000011) << 2) +
                          ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b0000000011) << 0);
}

void ICACHE_RAM_ATTR Generate4ChannelData_11bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((crsf.ChannelDataIn[0]) >> 3);
  Radio.TXdataBuffer[2] = ((crsf.ChannelDataIn[1]) >> 3);
  Radio.TXdataBuffer[3] = ((crsf.ChannelDataIn[2]) >> 3);
  Radio.TXdataBuffer[4] = ((crsf.ChannelDataIn[3]) >> 3);
  Radio.TXdataBuffer[5] = ((crsf.ChannelDataIn[0] & 0b00000111) << 5) +
                          ((crsf.ChannelDataIn[1] & 0b111) << 2) +
                          ((crsf.ChannelDataIn[2] & 0b110) >> 1);
  Radio.TXdataBuffer[6] = ((crsf.ChannelDataIn[2] & 0b001) << 7) +
                          ((crsf.ChannelDataIn[3] & 0b111) << 4); // 4 bits left over for something else?
#ifdef One_Bit_Switches
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[4]) << 3;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[5]) << 2;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[6]) << 1;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[7]) << 0;
#endif
}

void ICACHE_RAM_ATTR GenerateMSPData()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + MSP_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = MSPPacket.function;
  Radio.TXdataBuffer[2] = MSPPacket.payloadSize;
  Radio.TXdataBuffer[3] = 0;
  Radio.TXdataBuffer[4] = 0;
  Radio.TXdataBuffer[5] = 0;
  Radio.TXdataBuffer[6] = 0;
  if (MSPPacket.payloadSize <= 4)
  {
    MSPPacket.payloadReadIterator = 0;
    for (int i = 0; i < MSPPacket.payloadSize; i++)
    {
      Radio.TXdataBuffer[3 + i] = MSPPacket.readByte();
    }
  }
  else
  {
    Serial.println("Unable to send MSP command. Packet too long.");
  }
}

void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index) // Set speed of RF link (hz)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen);
  hwTimer.updateInterval(ModParams->interval);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;

  crsf.setSyncParams(ModParams->interval);
  connectionState = connected;

#ifdef PLATFORM_ESP32
  updateLEDs(connectionState, ExpressLRS_currAirRate_Modparams->TLMinterval);
#endif
}

uint8_t ICACHE_RAM_ATTR decTLMrate()
{
  Serial.println("dec TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval < (uint8_t)TLM_RATIO_1_2)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval + 1);
    Serial.println(currTLMinterval);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

uint8_t ICACHE_RAM_ATTR incTLMrate()
{
  Serial.println("inc TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval > (uint8_t)TLM_RATIO_NO_TLM)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval - 1);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

void ICACHE_RAM_ATTR decRFLinkRate()
{
  Serial.println("dec RFrate");
  SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index + 1);
}

void ICACHE_RAM_ATTR incRFLinkRate()
{
  Serial.println("inc RFrate");
  SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index - 1);
}

void ICACHE_RAM_ATTR HandleFHSS()
{
  uint8_t modresult = (NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  if (modresult == 0) // if it time to hop, do so.
  {
    Radio.SetFrequency(FHSSgetNextFreq());
  }
}

void ICACHE_RAM_ATTR HandleTLM()
{
  if (ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
  {
    uint8_t modresult = (NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    if (modresult != 0) // wait for tlm response because it's time
    {
      return;
    }
    Radio.RXnb();
    WaitRXresponse = true;
  }
}

void ICACHE_RAM_ATTR SendRCdataToRF()
{
#ifdef FEATURE_OPENTX_SYNC
  crsf.JustSentRFpacket(); // tells the crsf that we want to send data now - this allows opentx packet syncing
#endif

  /////// This Part Handles the Telemetry Response ///////
  if ((uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
  {
    uint8_t modresult = (NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    if (modresult == 0)
    { // wait for tlm response
      if (WaitRXresponse == true)
      {
        WaitRXresponse = false;
        LQCALC.inc();
        return;
      }
      else
      {
        NonceTX++;
      }
    }
  }

  uint32_t SyncInterval;

#if defined(NO_SYNC_ON_ARM) && defined(ARM_CHANNEL)
  SyncInterval = 250;
  bool skipSync = (bool)CRSF_to_BIT(crsf.ChannelDataIn[ARM_CHANNEL - 1]);
#else
  SyncInterval = (connectionState == connected) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  bool skipSync = false;
#endif

  if ((!skipSync) && ((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq()) && ((NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0))) // sync just after we changed freqs (helps with hwTimer.init() being in sync from the get go)
  {

    GenerateSyncPacketData();
    SyncPacketLastSent = millis();
    //Serial.println("sync");
    //Serial.println(Radio.currFreq);
  }
  else
  {
    if ((millis() > (MSP_PACKET_SEND_INTERVAL + MSPPacketLastSent)) && MSPPacketSendCount)
    {
      GenerateMSPData();
      MSPPacketLastSent = millis();
      MSPPacketSendCount--;
    }
    else
    {
      #if defined HYBRID_SWITCHES_8
      GenerateChannelDataHybridSwitch8(Radio.TXdataBuffer, &crsf, DeviceAddr);
      #elif defined SEQ_SWITCHES
      GenerateChannelDataSeqSwitch(Radio.TXdataBuffer, &crsf, DeviceAddr);
      #else
      Generate4ChannelData_11bit();
#endif
    }
  }

  ///// Next, Calculate the CRC and put it into the buffer /////
  uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
  Radio.TXdataBuffer[7] = crc;
  Radio.TXnb(Radio.TXdataBuffer, 8);
}

void ICACHE_RAM_ATTR RadioUARTconnected()
{
  hwTimer.resume();
  //inital state variables, maybe move elsewhere?
  for (int i = 0; i < 3; i++) // sometimes OpenTX ignores our packets (not sure why yet...)
  {
    uint8_t LUAdisableWebupdate[4] = {0xFE, 0x00, 0x00, 0x00};
    crsf.sendLUAresponse(LUAdisableWebupdate, 4);
    uint8_t LUAbindDone[4] = {0xFF, 0x00, 0x00, 0x00};
    crsf.sendLUAresponse(LUAbindDone, 4);
  }
}

void ICACHE_RAM_ATTR ParamUpdateReq()
{
  UpdateParamReq = true;

  if (crsf.ParameterUpdateData[0] == 1)
  {
    hwTimer.stop();
  }
}

void HandleUpdateParameter()
{
  if (UpdateParamReq == false)
  {
    return;
  }
  
  switch (crsf.ParameterUpdateData[0])
  {
  case 0: // send all params
    Serial.println("send all lua params");
    crsf.sendLUAresponse(luaCommitPacket, 4);
    crsf.sendLUAresponse(luaCommitOtherHalfPacket, 4);
    break;

  case 1:
    Serial.println("Change Link rate");
    if ((micros() + PacketLastSentMicros) > ExpressLRS_currAirRate_Modparams->interval) // special case, if we haven't waited long enough to ensure that the last packet hasn't been sent we exit. 
    {
      if (crsf.ParameterUpdateData[1] == 0)
      {
        decRFLinkRate();
      }
      else if (crsf.ParameterUpdateData[1] == 1)
      {
        incRFLinkRate();
      }
      Serial.println(ExpressLRS_currAirRate_Modparams->enum_rate);
      hwTimer.resume();
    }
    else
    {
      return;
    }
    break;

  case 2:

    if (crsf.ParameterUpdateData[1] == 0)
    {
      decTLMrate();
    }
    else if (crsf.ParameterUpdateData[1] == 1)
    {
      incTLMrate();
    }

    break;

  case 3:

    if (crsf.ParameterUpdateData[1] == 0)
    {
      Serial.println("Decrease RF power");
      POWERMGNT.decPower();
    }
    else if (crsf.ParameterUpdateData[1] == 1)
    {
      Serial.println("Increase RF power");
      POWERMGNT.incPower();
    }

    break;

  case 4:

    break;

  case 0xFE:
    if (crsf.ParameterUpdateData[1] == 1)
    {
#ifdef PLATFORM_ESP32
      uint8_t LUAdisableWebupdate[4] = {0xFE, 0x01, 0x00, 0x00};
      crsf.sendLUAresponse(LUAdisableWebupdate, 4); // send this to confirm
      delay(500);
      Serial.println("Wifi Update Mode Requested!");
      webUpdateMode = true;
      BeginWebUpdate();
      return; // no need to do anything else
#else
      uint8_t LUAdisableWebupdate[4] = {0xFE, 0x00, 0x00, 0x00};
      crsf.sendLUAresponse(LUAdisableWebupdate, 4); // send this to confirm
      Serial.println("Wifi Update Mode Requested but not supported on this platform!");
#endif
      break;
    }

  case 0xFF:
    if (crsf.ParameterUpdateData[1] == 1)
    {
      Serial.println("Binding Requested!");
      uint8_t luaBindingRequestedPacket[] = {(uint8_t)0xFF, (uint8_t)0x01, (uint8_t)0x00, (uint8_t)0x00};
      crsf.sendLUAresponse(luaBindingRequestedPacket, 4);

      //crsf.sendLUAresponse((uint8_t)0xFF, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00); // send this to confirm binding is done
    }
    break;

  default:
    break;
  }

  UpdateParamReq = false;
  uint8_t luaCurrParams[] = {(uint8_t)(ExpressLRS_currAirRate_Modparams->enum_rate + 3), (uint8_t)(ExpressLRS_currAirRate_Modparams->TLMinterval + 1), (uint8_t)(POWERMGNT.currPower() + 2), Regulatory_Domain_Index};
  crsf.sendLUAresponse(luaCurrParams, 4);
}

void ICACHE_RAM_ATTR RXdoneISR()
{
  ProcessTLMpacket();
}

void ICACHE_RAM_ATTR TXdoneISR()
{
  NonceTX++; // must be done before callback
  HandleFHSS();
  HandleTLM();
}

void setup()
{
#ifdef PLATFORM_ESP32
  Serial.begin(115200);
  #ifdef USE_UART2
    #ifndef TARGET_TTGO_LORA_V2_AS_TX
    Serial2.begin(400000);
    #else
    Serial.println("USE_UART2 was enable but is not supported on TTGOv2");
    #endif
  #endif
#endif

#if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX) || defined(TARGET_R9M_LITE_PRO_TX)

    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
    pinMode(GPIO_PIN_LED_RED, OUTPUT);
    digitalWrite(GPIO_PIN_LED_GREEN, HIGH);

#ifdef USE_ESP8266_BACKPACK
    HardwareSerial(USART1);
#else
    HardwareSerial(USART2);
    Serial.setTx(PA2);
    Serial.setRx(PA3);
#endif
    Serial.begin(400000);

#if defined(TARGET_R9M_TX)
    // Annoying startup beeps
#ifndef JUST_BEEP_ONCE
  pinMode(GPIO_PIN_BUZZER, OUTPUT);
  const int beepFreq[] = {659, 659, 523, 659, 783, 392};
  const int beepDurations[] = {300, 300, 100, 300, 550, 575};

  for (int i = 0; i < 6; i++)
  {
    tone(GPIO_PIN_BUZZER, beepFreq[i], beepDurations[i]);
    delay(beepDurations[i]);
    noTone(GPIO_PIN_BUZZER);
  }
  #else
  tone(GPIO_PIN_BUZZER, 400, 200);
  delay(200);
  tone(GPIO_PIN_BUZZER, 480, 200);
  #endif
  button.init(GPIO_PIN_BUTTON, true); // r9 tx appears to be active high
  R9DAC.init();
#endif

#endif

#ifdef PLATFORM_ESP32
#ifdef GPIO_PIN_LED
  strip.Begin();
#endif
  // Get base mac address
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Print base mac address
  // This should be copied to common.h and is used to generate a unique hop sequence, DeviceAddr, and CRC.
  // UID[0..2] are OUI (organisationally unique identifier) and are not ESP32 unique.  Do not use!
#endif

  FHSSrandomiseFHSSsequence();

  Radio.RXdoneCallback = &RXdoneISR;
  Radio.TXdoneCallback = &TXdoneISR;

#ifndef One_Bit_Switches
  crsf.RCdataCallback1 = &CheckChannels5to8Change;
#endif
  crsf.connected = &RadioUARTconnected; // it will auto init when it detects UART connection
  crsf.disconnected = &hwTimer.stop;
  crsf.RecvParameterUpdate = &ParamUpdateReq;
  hwTimer.callbackTock = &TimerCallbackISR;

  Serial.println("ExpressLRS TX Module Booted...");

  POWERMGNT.init();
  Radio.currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
  #if !(defined(TARGET_TX_ESP32_E28_SX1280_V1) || defined(TARGET_TX_ESP32_SX1280_V1) || defined(TARGET_RX_ESP8266_SX1280_V1) || defined(Regulatory_Domain_ISM_2400))
  Radio.currSyncWord = UID[3];
  #endif
  bool init_success = Radio.Begin();
  while (!init_success)
  {
    #if defined(TARGET_R9M_TX)
    digitalWrite(GPIO_PIN_LED_GREEN, LOW);
    tone(GPIO_PIN_BUZZER, 480, 200);
    digitalWrite(GPIO_PIN_LED_RED, LOW);
    delay(200);
    tone(GPIO_PIN_BUZZER, 400, 200);
    digitalWrite(GPIO_PIN_LED_RED, HIGH);
    delay(1000);
    #endif
  }
  POWERMGNT.setDefaultPower();
  SetRFLinkRate(RATE_DEFAULT); // fastest rate by default
  crsf.Begin();
  hwTimer.init();
  hwTimer.stop(); //comment to automatically start the RX timer and leave it running
  LQCALC.init(10);
}

void loop()
{

#if defined(PLATFORM_ESP32)
  if (webUpdateMode)
  {
    HandleWebUpdate();
    return;
  }
#endif

  HandleUpdateParameter();

#ifdef FEATURE_OPENTX_SYNC
// Serial.println(crsf.OpenTXsyncOffset);
#endif

  if (millis() > (RX_CONNECTION_LOST_TIMEOUT + LastTLMpacketRecvMillis))
  {
    connectionState = disconnected;
    #if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX) || defined(TARGET_R9M_LITE_PRO_TX)
    digitalWrite(GPIO_PIN_LED_RED, LOW);
    #endif
  }
  else
  {
    connectionState = connected;
    #if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX) || defined(TARGET_R9M_LITE_PRO_TX)
    digitalWrite(GPIO_PIN_LED_RED, HIGH);
    #endif
  }

#if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX) || defined(TARGET_R9M_LITE_PRO_TX)
  crsf.STM32handleUARTin();
  #ifdef FEATURE_OPENTX_SYNC
  crsf.sendSyncPacketToTX();
  #endif
  crsf.UARTwdt();
  #ifdef TARGET_R9M_TX
  button.handle();
  #endif
#endif

#ifdef PLATFORM_ESP32
  if (Serial2.available())
  {
    uint8_t c = Serial2.read();
#else
  if (Serial.available())
  {
    uint8_t c = Serial.read();
#endif

    if (msp.processReceivedByte(c))
    {
      // Finished processing a complete packet
      ProcessMSPPacket(msp.getReceivedPacket());
      msp.markPacketReceived();
    }
  }
}

void ICACHE_RAM_ATTR TimerCallbackISR()
{
  PacketLastSentMicros = micros();
  SendRCdataToRF();
}

void OnRFModePacket(mspPacket_t *packet)
{
  // Parse the RF mode
  uint8_t rfMode = packet->readByte();
  CHECK_PACKET_PARSING();

  switch (rfMode)
  {
  case RATE_200HZ:
    SetRFLinkRate(RATE_200HZ);
    break;
  case RATE_100HZ:
    SetRFLinkRate(RATE_100HZ);
    break;
  case RATE_50HZ:
    SetRFLinkRate(RATE_50HZ);
    break;
  default:
    // Unsupported rate requested
    break;
  }
}

void OnTxPowerPacket(mspPacket_t *packet)
{
  // Parse the TX power
  uint8_t txPower = packet->readByte();
  CHECK_PACKET_PARSING();
  Serial.println("TX setpower");

  switch (txPower)
  {
  case PWR_10mW:
    POWERMGNT.setPower(PWR_10mW);
    break;
  case PWR_25mW:
    POWERMGNT.setPower(PWR_25mW);
    break;
  case PWR_50mW:
    POWERMGNT.setPower(PWR_50mW);
    break;
  case PWR_100mW:
    POWERMGNT.setPower(PWR_100mW);
    break;
  case PWR_250mW:
    POWERMGNT.setPower(PWR_250mW);
    break;
  case PWR_500mW:
    POWERMGNT.setPower(PWR_500mW);
    break;
  case PWR_1000mW:
    POWERMGNT.setPower(PWR_1000mW);
    break;
  case PWR_2000mW:
    POWERMGNT.setPower(PWR_2000mW);
    break;
  default:
    // Unsupported power requested
    break;
  }
}

void OnTLMRatePacket(mspPacket_t *packet)
{
  // Parse the TLM rate
  //uint8_t tlmRate = packet->readByte();
  CHECK_PACKET_PARSING();

  // TODO: Implement dynamic TLM rates
  // switch (tlmRate) {
  // case TLM_RATIO_NO_TLM:
  //   break;
  // case TLM_RATIO_1_128:
  //   break;
  // default:
  //   // Unsupported rate requested
  //   break;
  // }
}

void ProcessMSPPacket(mspPacket_t *packet)
{
  // Inspect packet for ELRS specific opcodes
  if (packet->function == MSP_ELRS_FUNC)
  {
    uint8_t opcode = packet->readByte();

    CHECK_PACKET_PARSING();

    switch (opcode)
    {
    case MSP_ELRS_RF_MODE:
      OnRFModePacket(packet);
      break;
    case MSP_ELRS_TX_PWR:
      OnTxPowerPacket(packet);
      break;
    case MSP_ELRS_TLM_RATE:
      OnTLMRatePacket(packet);
      break;
    default:
      break;
    }
  }
  else if (packet->function == MSP_SET_VTX_CONFIG)
  {
    MSPPacket = *packet;
    MSPPacketSendCount = 6;
  }
}
