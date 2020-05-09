#pragma once

#include <Arduino.h>
#include "../../src/targets.h"

#ifdef PLATFORM_ESP32
#include "FreeRTOS.h"
#include "esp32-hal-timer.h"
#endif

#ifdef PLATFORM_8266
#include <cstdint>
#endif

#define SX1280_XTAL_FREQ 52000000
#define SX1280_FREQ_STEP ((double)(XTAL_FREQ / pow(2.0, 18.0)))

typedef enum
{
    SX1280_RF_IDLE = 0x00, //!< The radio is idle
    SX1280_RF_RX_RUNNING,  //!< The radio is in reception state
    SX1280_RF_TX_RUNNING,  //!< The radio is in transmission state
    SX1280_RF_CAD,         //!< The radio is doing channel activity detection
} SX1280_RadioStates_t;

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX1280_MODE_SLEEP = 0x00, //! The radio is in sleep mode
    SX1280_MODE_CALIBRATION,  //! The radio is in calibration mode
    SX1280_MODE_STDBY_RC,     //! The radio is in standby mode with RC oscillator
    SX1280_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    SX1280_MODE_FS,           //! The radio is in frequency synthesis mode
    SX1280_MODE_RX,           //! The radio is in receive mode
    SX1280_MODE_TX,           //! The radio is in transmit mode
    SX1280_MODE_CAD           //! The radio is in channel activity detection mode
} SX1280_RadioOperatingModes_t;

#define SX1280_RX_TX_CONTINUOUS ( TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0xFFFF }
#define SX1280_RX_TX_SINGLE     ( TickTime_t ){ RADIO_TICK_SIZE_0015_US, 0 }

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    SX1280_STDBY_RC = 0x00,
    SX1280_STDBY_XOSC = 0x01,
} SX1280_RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    SX1280_USE_LDO = 0x00,  //! Use LDO (default value)
    SX1280_USE_DCDC = 0x01, //! Use DCDC
} SX1280_RadioRegulatorModes_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    SX1280_PACKET_TYPE_GFSK = 0x00,
    SX1280_PACKET_TYPE_LORA,
    SX1280_PACKET_TYPE_RANGING,
    SX1280_PACKET_TYPE_FLRC,
    SX1280_PACKET_TYPE_BLE,
    SX1280_PACKET_TYPE_NONE = 0x0F,
} RadioPacketTypes_t;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    SX1280_RADIO_RAMP_02_US = 0x00,
    SX1280_RADIO_RAMP_04_US = 0x20,
    SX1280_RADIO_RAMP_06_US = 0x40,
    SX1280_RADIO_RAMP_08_US = 0x60,
    SX1280_RADIO_RAMP_10_US = 0x80,
    SX1280_RADIO_RAMP_12_US = 0xA0,
    SX1280_RADIO_RAMP_16_US = 0xC0,
    SX1280_RADIO_RAMP_20_US = 0xE0,
} SX1280_RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    SX1280_LORA_CAD_01_SYMBOL = 0x00,
    SX1280_LORA_CAD_02_SYMBOLS = 0x20,
    SX1280_LORA_CAD_04_SYMBOLS = 0x40,
    SX1280_LORA_CAD_08_SYMBOLS = 0x60,
    SX1280_LORA_CAD_16_SYMBOLS = 0x80,
} SX1280_RadioLoRaCadSymbols_t;

/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    SX1280_LORA_SF5 = 0x50,
    SX1280_LORA_SF6 = 0x60,
    SX1280_LORA_SF7 = 0x70,
    SX1280_LORA_SF8 = 0x80,
    SX1280_LORA_SF9 = 0x90,
    SX1280_LORA_SF10 = 0xA0,
    SX1280_LORA_SF11 = 0xB0,
    SX1280_LORA_SF12 = 0xC0,
} RadioLoRaSpreadingFactors_t;

/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    SX1280_LORA_BW_0200 = 0x34,
    SX1280_LORA_BW_0400 = 0x26,
    SX1280_LORA_BW_0800 = 0x18,
    SX1280_LORA_BW_1600 = 0x0A,
} RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    SX1280_LORA_CR_4_5 = 0x01,
    SX1280_LORA_CR_4_6 = 0x02,
    SX1280_LORA_CR_4_7 = 0x03,
    SX1280_LORA_CR_4_8 = 0x04,
    SX1280_LORA_CR_LI_4_5 = 0x05,
    SX1280_LORA_CR_LI_4_6 = 0x06,
    SX1280_LORA_CR_LI_4_7 = 0x07,
} RadioLoRaCodingRates_t;


typedef enum
{
    SX1280_LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    SX1280_LORA_PACKET_FIXED_LENGTH                = 0x80,         //!< The packet is known on both sides, no header included in the packet
    SX1280_LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
    SX1280_LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
}SX1280_RadioLoRaPacketLengthsModes_t;

typedef enum
{
    SX1280_LORA_CRC_ON                             = 0x20,         //!< CRC activated
    SX1280_LORA_CRC_OFF                            = 0x00,         //!< CRC not used
}SX1280_RadioLoRaCrcModes_t;

typedef enum
{
    SX1280_LORA_IQ_NORMAL                          = 0x40,
    SX1280_LORA_IQ_INVERTED                        = 0x00,
}SX1280_RadioLoRaIQModes_t;


typedef enum
{
    SX1280_IRQ_RADIO_NONE                          = 0x0000,
    SX1280_IRQ_TX_DONE                             = 0x0001,
    SX1280_IRQ_RX_DONE                             = 0x0002,
    SX1280_IRQ_SYNCWORD_VALID                      = 0x0004,
    SX1280_IRQ_SYNCWORD_ERROR                      = 0x0008,
    SX1280_IRQ_HEADER_VALID                        = 0x0010,
    SX1280_IRQ_HEADER_ERROR                        = 0x0020,
    SX1280_IRQ_CRC_ERROR                           = 0x0040,
    SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE         = 0x0080,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED     = 0x0100,
    SX1280_IRQ_RANGING_MASTER_RESULT_VALID         = 0x0200,
    SX1280_IRQ_RANGING_MASTER_TIMEOUT              = 0x0400,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID         = 0x0800,
    SX1280_IRQ_CAD_DONE                            = 0x1000,
    SX1280_IRQ_CAD_DETECTED                        = 0x2000,
    SX1280_IRQ_RX_TX_TIMEOUT                       = 0x4000,
    SX1280_IRQ_PREAMBLE_DETECTED                   = 0x8000,
    SX1280_IRQ_RADIO_ALL                           = 0xFFFF,
}RadioIrqMasks_t;


typedef enum
{
    SX1280_RADIO_DIO1                              = 0x02,
    SX1280_RADIO_DIO2                              = 0x04,
    SX1280_RADIO_DIO3                              = 0x08,
}SX1280_RadioDios_t;


typedef enum
{
    SX1280_RADIO_TICK_SIZE_0015_US                 = 0x00,
    SX1280_RADIO_TICK_SIZE_0062_US                 = 0x01,
    SX1280_RADIO_TICK_SIZE_1000_US                 = 0x02,
    SX1280_RADIO_TICK_SIZE_4000_US                 = 0x03,
}SX1280_RadioTickSizes_t;

typedef enum
{
    SX1280_OPMODE_FSK_OOK = SX127X_FSK_OOK,
    SX1280_OPMODE_LORA = SX127X_LORA, //removed CURR_OPMODE_ACCESS_SHARED_REG_OFF and CURR_OPMODE_ACCESS_SHARED_REG_ON for now
    SX1280_OPMODE_SLEEP = SX127X_SLEEP,
    SX1280_OPMODE_STANDBY = SX127X_STANDBY,
    OPMODE_FSTX = SX127X_FSTX,
    OPMODE_TX = SX127X_TX,
    OPMODE_FSRX = SX127X_FSRX,
    OPMODE_RXCONTINUOUS = SX127X_RXCONTINUOUS,
    OPMODE_RXSINGLE = SX127X_RXSINGLE,
    OPMODE_CAD = SX127X_CAD,
    OPMODE_UNDEF = 0b11111111
} RadioOPmodes;

typedef enum
{
    CH_SX1272,
    CH_SX1273,
    CH_SX1276,
    CH_SX1277,
    CH_SX1278,
    CH_SX1279
} Chip;
typedef enum
{
    BW_7_80_KHZ = 0,
    BW_10_40_KHZ = 1,
    BW_15_60_KHZ = 2,
    BW_20_80_KHZ = 3,
    BW_31_25_KHZ = 4,
    BW_41_70_KHZ = 5,
    BW_62_50_KHZ = 6,
    BW_125_00_KHZ = 7,
    BW_250_00_KHZ = 8,
    BW_500_00_KHZ = 9
} Bandwidth;
typedef enum
{
    SF_6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12
} SpreadingFactor;
typedef enum
{
    CR_4_5,
    CR_4_6,
    CR_4_7,
    CR_4_8
} CodingRate;
typedef enum
{
    RFMOD_SX1278,
    RFMOD_SX1276
} RFmodule_;

class SX1280Driver
{

public:
    ///////Callback Function Pointers/////
    static void (*RXdoneCallback1)(); //function pointer for callback
    static void (*RXdoneCallback2)(); //function pointer for callback
    static void (*TXdoneCallback1)(); //function pointer for callback
    static void (*TXdoneCallback2)(); //function pointer for callback
    static void (*TXdoneCallback3)(); //function pointer for callback
    static void (*TXdoneCallback4)(); //function pointer for callback

    static void (*TXtimeout)(); //function pointer for callback
    static void (*RXtimeout)(); //function pointer for callback

    static void (*TimerDoneCallback)(); //function pointer for callback

#ifdef PLATFORM_ESP32
    static TaskHandle_t Timertask_handle; //Task Handle for ContTX mode
#endif

    //static void (*TXcallback)();

    ////////Hardware/////////////
    static uint8_t _RXenablePin;
    static uint8_t _TXenablePin;

    static uint8_t SX1280_nss;
    static uint8_t SX1280_dio0;
    static uint8_t SX1280_dio1;

    static uint8_t SX1280_MOSI;
    static uint8_t SX1280_MISO;
    static uint8_t SX1280_SCK;
    static uint8_t SX1280_RST;
    /////////////////////////////

    ///////////Radio Variables////////
    static uint8_t TXdataBuffer[256];
    static uint8_t RXdataBuffer[256];

    static volatile uint8_t TXbuffLen;
    static volatile uint8_t RXbuffLen;

    //static volatile bool headerExplMode;

    static volatile uint32_t TimerInterval; //20ms default for now.

    static uint32_t currFreq;
    static uint8_t _syncWord;

    static RFmodule_ RFmodule;
    static Bandwidth currBW;
    static SpreadingFactor currSF;
    static CodingRate currCR;
    static uint8_t currPWR;
    static uint8_t maxPWR;
    static RadioOPmodes currOpmode;
    ///////////////////////////////////

    /////////////Packet Stats//////////
    static int8_t LastPacketRSSI;
    static int8_t LastPacketSNR;
    static float PacketLossRate;
    static volatile uint8_t NonceTX;
    static volatile uint8_t NonceRX;
    static uint32_t TotalTime;
    static uint32_t TimeOnAir;
    static uint32_t TXstartMicros;
    static uint32_t TXspiTime;
    static uint32_t HeadRoom;
    static uint32_t TXdoneMicros;
    /////////////////////////////////

    //// Local Variables //// Copy of values for SPI speed optimisation
    static uint8_t CURR_REG_PAYLOAD_LENGTH;
    static uint8_t CURR_REG_DIO_MAPPING_1;
    static uint8_t CURR_REG_FIFO_ADDR_PTR;

    ////////////////Configuration Functions/////////////
    static uint8_t Begin();
    static uint8_t Config(Bandwidth bw, SpreadingFactor sf, CodingRate cr, uint32_t freq, uint8_t syncWord);
    static uint8_t SX127xConfig(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t syncWord);
    static void ConfigLoraDefaults();

    static uint8_t SetBandwidth(Bandwidth bw);
    static uint32_t getCurrBandwidth();
    static uint32_t getCurrBandwidthNormalisedShifted();
    static uint8_t SetSyncWord(uint8_t syncWord);
    static uint8_t SetOutputPower(uint8_t Power);
    static uint8_t SetPreambleLength(uint8_t PreambleLen);
    static uint8_t SetSpreadingFactor(SpreadingFactor sf);
    static uint8_t SetCodingRate(CodingRate cr);
    static uint8_t SetFrequency(uint32_t freq);
    static int32_t GetFrequencyError();
    static void setPPMoffsetReg(int32_t offset);

    static uint8_t SX127xBegin();
    static uint8_t SetMode(RadioOPmodes mode);
    static uint8_t TXsingle(uint8_t *data, uint8_t length);
    ////////////////////////////////////////////////////

    /////////////////Utility Funcitons//////////////////
    static void ClearIRQFlags();

    //////////////RX related Functions/////////////////

    static uint8_t RunCAD();

    static uint8_t ICACHE_RAM_ATTR UnsignedGetLastPacketRSSI();

    static int8_t ICACHE_RAM_ATTR GetLastPacketRSSI();
    static int8_t ICACHE_RAM_ATTR GetLastPacketSNR();
    static int8_t ICACHE_RAM_ATTR GetCurrRSSI();

    ////////////Non-blocking TX related Functions/////////////////
    static void nullCallback(void);

    static void ICACHE_RAM_ATTR StartTimerTask(); //Start Cont TX mode, sends data continuiously
    static void ICACHE_RAM_ATTR StopTimerTask();
    static void ICACHE_RAM_ATTR UpdateTimerInterval();
    static uint8_t ICACHE_RAM_ATTR TXnb(const volatile uint8_t *data, uint8_t length);

    static void ICACHE_RAM_ATTR TXnbISR(); //ISR for non-blocking TX routine
    static void ICACHE_RAM_ATTR TimerTask_ISRhandler();
    static void ICACHE_RAM_ATTR TimerTask(void *param);

    /////////////Non-blocking RX related Functions///////////////
    static void ICACHE_RAM_ATTR StopContRX();
    static void ICACHE_RAM_ATTR RXnb();

    static void ICACHE_RAM_ATTR RXnbISR(); //ISR for non-blocking RC routine

    static uint8_t ICACHE_RAM_ATTR RXsingle(uint8_t *data, uint8_t length);
    static uint8_t ICACHE_RAM_ATTR RXsingle(uint8_t *data, uint8_t length, uint32_t timeout);

private:
};