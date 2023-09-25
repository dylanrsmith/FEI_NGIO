// https://github.com/Xinyuan-LilyGO/T-FPGA
/* ESP32S3 Dev Module
  USB CDC On Boot  Enabled
  CPU Frequency 240MHz (WiFi)
  Core Debug Level  None
  USB DFU On Boot Enabled
  Events Run On Core 1
  Flash Mode  QIO 80MHz
  Flash Size  16MB (128Mb)
  JTAG Adapter  Integrated USB JTAG
  Arduino Runs On Core 1
  USB Firmware MSC On Boot  Disabled
  Partition Scheme  Huge APP (3MB No OTA/1MB SPIFFS)
  PSRAM OPI PSRAM
  USB Mode  Hardware CDC and JTAG
*/


#include <SPI.h>
#include "Wire.h"
#include "XPowersLib.h"//https://github.com/lewisxhe/XPowersLib
XPowersAXP2101 PMU;

void writeSlotBoard(byte slotNumber, byte redCol, byte greenCol, byte blueCol);
uint16_t checksumCalculator(uint8_t * data, uint16_t length);

//#define VSPI_MISO   4
//#define VSPI_MOSI   3
//#define VSPI_SCLK   5
//#define VSPI_SS     6

// FPGA PINS
#define PIN_BTN      0
#define PIN_IIC_SDA  38
#define PIN_IIC_SCL  39
#define PIN_PMU_IRQ  40
//vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
#define PIN_LED      46
const int ESP_D0 = 1;
const int ESP_D1 = 3;
const int ESP_D2 = 5;
const int ESP_D3 = 6;
const int ESP_D4 = 4;
const int ESP_D5 = 2;
/*
  CONN/!DAT
  D0i D1i D2i D3i D4i D5o
  1   0   0   0   0   x  SMART IO CONNECT OE=0
  1   0   0   1   0   x   SMART IO CONNECT OE=1
  1   1   0   0   0   x   SIMPLE IO CONNECT OE=0
  1   1   0   1   0   x   SIMPLE IO CONNECT OE=1
  1   0   1   0   0   x   RELAY IO CONNECT

  D0i D1i D2i D3i   D4i     D5o
  0   SCK MO  SYNC  LATCH   MISO    SMART IO CONNECT DATA MODE
  0   SCK MO  PL    LATCH   MISO  SIMPLE IO CONNECT DATA MODE
  0   SCK MO  OE    LATCH   x RELAY IO CONNECT DATA MODE
*/
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif
static const uint32_t spiClk = 100000;
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

bool passFailArray[100];

byte testData = 0;
byte redColor = 255;
byte greenColor = 255;
byte blueColor = 255;
float factor1, factor2;
int ind = 0;
void rainbowAnimation() {
  int stepSize = 1023;
  int dividerVar = 341;
  int subVar = 682;
  switch ((int)((ind % stepSize) / dividerVar)) {
    case 0:
      factor1 = 1.0 - ((float)(ind % stepSize - 0 * dividerVar) / dividerVar);
      factor2 = (float)((int)(ind - 0) % stepSize) / dividerVar;
      //strip_0.strip.setPixelColor(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
      redColor = int(255 * factor1 + 0 * factor2);
      greenColor = int( 0 * factor1 + 255 * factor2);
      blueColor = int(0 * factor1 + 0 * factor2);
      break;
    case 1:
      factor1 = 1.0 - ((float)(ind % stepSize - 1 * dividerVar) / dividerVar);
      factor2 = (float)((int)(ind - dividerVar) % stepSize) / dividerVar;
      //strip_0.strip.setPixelColor(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
      redColor = int(0 * factor1 + 0 * factor2);
      greenColor = int( 255 * factor1 + 0 * factor2);
      blueColor = int(0 * factor1 + 255 * factor2);
      break;
    case 2:
      factor1 = 1.0 - ((float)(ind % stepSize - 2 * dividerVar) / dividerVar);
      factor2 = (float)((int)(ind - subVar) % stepSize) / dividerVar;
      //strip_0.strip.setPixelColor(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
      redColor = int(0 * factor1 + 255 * factor2);
      greenColor = int( 0 * factor1 + 0 * factor2);
      blueColor = int(255 * factor1 + 0 * factor2);
      break;
  }
  if (ind >= stepSize) {
    ind = 0;
  } else ind++;
}

void setup() {
  Serial.begin(230400);
  delay(3000);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_IIC_SDA, PIN_IIC_SCL);
  if (result == false) {
    //Serial.println("PMU is not online...");
    while (1)
      delay(50);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }


  PMU.setDC4Voltage(1200);   // Here is the FPGA core voltage. Careful review of the manual is required before modification.
  PMU.setALDO1Voltage(3300); // BANK0 area voltage
  PMU.setALDO2Voltage(3300); // BANK1 area voltage
  PMU.setALDO3Voltage(3300); // BANK2 area voltage
  PMU.setALDO4Voltage(3300); // BANK3 area voltage

  PMU.enableALDO1();
  PMU.enableALDO2();
  PMU.enableALDO3();
  PMU.enableALDO4();
  PMU.disableTSPinMeasure();
  //delay(1000);
  //vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  pinMode(ESP_D0, OUTPUT);
  pinMode(ESP_D1, OUTPUT);//sck
  pinMode(ESP_D2, OUTPUT);//mosi
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT);//ss
  pinMode(ESP_D5, INPUT);//miso

  //  digitalWrite(ESP_D1, HIGH);
  //  digitalWrite(ESP_D2, HIGH);
  //  digitalWrite(ESP_D3, HIGH);
  //  digitalWrite(ESP_D4, HIGH);
  //  digitalWrite(ESP_D0, HIGH);
  digitalWrite(ESP_D0, LOW);



  vspi = new SPIClass(VSPI);
  //  vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  //  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  //Serial.println("Starting...");

  pinMode(ESP_D1, OUTPUT);
  pinMode(ESP_D2, OUTPUT);
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT);
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW); //ALL LOW is SMART with OE Enabled

  digitalWrite(ESP_D0, HIGH);// all is applied on rising edge
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW);
  digitalWrite(ESP_D0, LOW);

  vspi->begin(ESP_D1, ESP_D5, ESP_D2);
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
}

void loop() {

  //  unsigned long animationStart = millis();
  int numberOfSlots = 32;

  for (int i = 1; i <= numberOfSlots; i++) {
    writeSlotBoard(i, 200, 0, 0);
    delay(20);
  }
  delay(1000);
  /*
    while (millis() - animationStart < 5000) {  //rainbow wave
     rainbowAnimation();
     for (int i = 1; i <= numberOfSlots; i++) {
       writeSlotBoard(i, redColor, greenColor, blueColor);
       digitalWrite(PIN_LED, !digitalRead(PIN_LED));
       unsigned long startRainbowWaitTime = millis();
       while (millis() - startRainbowWaitTime < 10) {
       }
     }
    }

     animationStart = millis(); //random rainbow
     for (int i = 1; i <= numberOfSlots; i++) {
       writeSlotBoard(i, 200, 0, 0);
       delay(20);
     }
     for (int i = 1; i <= numberOfSlots; i++) {
       writeSlotBoard(i, 0, 200, 0);
       delay(20);
     }
     for (int i = 1; i <= numberOfSlots; i++) {
       writeSlotBoard(i, 0, 0, 200);
       delay(20);
     }
     for (int i = 1; i <= numberOfSlots; i++) {
       writeSlotBoard(i, 0, 0, 0);
       delay(20);
     }
     while (millis() - animationStart < 10000) {
       int randomLED = random(1, numberOfSlots+1);
       writeSlotBoard(randomLED, redColor, greenColor, blueColor);
       unsigned long startRainbowWaitTime = millis();
       while (millis() - startRainbowWaitTime < 20) {
         rainbowAnimation();
       }
     }

  */


  //  unsigned long startWaitTime = millis();
  //  while (millis() - startWaitTime < 10) {
  //
  //  }
}
