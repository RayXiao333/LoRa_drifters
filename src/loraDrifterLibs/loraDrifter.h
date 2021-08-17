#ifndef LORADRIFTER.H
#define LORADRIFTER.H

// A. WiFi & Web Server
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// B. SPI & LoRa
#include <SPI.h>
#include <LoRa.h>

// B1. SPIFFS
#include "SPIFFS.h"

// C. GPS libraries
#include <Wire.h>           // for reseting NMEA output from some T-Beam units
#include <TinyGPS++.h>      // decoding GPS 

// D. Power management on the TTGo T-beam
#include "axp20x.h"         // I need it for the new ones
AXP20X_Class PMU;

// E. Defines for TTGO T Beam V1.1 with LoRa
#define GPS_RX_PIN                  34
#define GPS_TX_PIN                  12
#define BUTTON_PIN                  38
#define BUTTON_PIN_MASK GPIO_SEL_38
#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DI0_PIN               26
#define RADIO_RST_PIN               23
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              32

#define GPS_BAND_RATE               9600
#define LoRa_frequency              915E6
#define UNUSE_PIN                   (0)
#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

String processor(const String& var);

bool initPMU() {
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
    if(PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    /*
    * The charging indicator can be turned on or off
    **/
    PMU.setChgLEDMode(AXP20X_LED_OFF);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC1Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    *
    **/

    /*
    *  Turn off unused power sources to save power
    **/

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
        * Set the power of LoRa and GPS module to 3.3V
        **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD
    PMU.setDCDC1Voltage(3300);  //3.3V Pin next to 21 and 22 is controlled by DCDC1

    PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] {
        // pmu_irq = true;
    }, FALLING);

    PMU.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                    AXP202_VBUS_CUR_ADC1 |
                    AXP202_BATT_CUR_ADC1 |
                    AXP202_BATT_VOL_ADC1,
                    AXP202_ON);

    PMU.enableIRQ(AXP202_VBUS_REMOVED_IRQ |
                    AXP202_VBUS_CONNECT_IRQ |
                    AXP202_BATT_REMOVED_IRQ |
                    AXP202_BATT_CONNECT_IRQ,
                    AXP202_ON);
    PMU.clearIRQ();

    return true;
}

void initBoard() {
    Serial.begin(115200);
    Serial.println("initBoard");
    initPMU();
    delay(50);
    
    #ifdef BOARD_LED
      /*
      * T-BeamV1.0, V1.1 LED defaults to low level as turn on,
      * so it needs to be forced to pull up
      * * * * */
      #if LED_ON == LOW
          gpio_hold_dis(GPIO_NUM_4);
      #endif
      pinMode(BOARD_LED, OUTPUT);
      digitalWrite(BOARD_LED, LED_ON);
    #endif
    delay(50);
 
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    delay(50);
}

// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + String(".") +
    String(ipAddress[1]) + String(".") +
    String(ipAddress[2]) + String(".") +
    String(ipAddress[3]);
}

// 3 + 4 + 2 + (1 * 5) + (2 * 8) + 4 + 4 = 38 bytes
#pragma pack(1) // Fixes padding issues
struct Packet {
  char name[3];             // D01
  int drifterTimeSlotSec;   // 15
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double lng;
  double lat;
  uint32_t age;
  int nSamples;
};

#endif //LORADRIFTER.H
