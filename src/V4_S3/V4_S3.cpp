#include <SPI.h>
#include "Wire.h"
#include "XPowersLib.h" //https://github.com/lewisxhe/XPowersLib
#include "EasyCAT.h"
#include <CRC.h>
XPowersAXP2101 PMU;

void writeSlotBoard(byte slotNumber, byte redCol, byte greenCol, byte blueCol);

// #define VSPI_MISO   4
// #define VSPI_MOSI   3
// #define VSPI_SCLK   5
// #define VSPI_SS     6

// FPGA PINS
#define PIN_BTN 0
#define PIN_IIC_SDA 38
#define PIN_IIC_SCL 39
#define PIN_PMU_IRQ 40
// vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
#define PIN_LED 46
#define VSPI HSPI // previously fspi, change to hspi
const int ESP_D0 = 1;
const int ESP_D1 = 3;
const int ESP_D2 = 5;
const int ESP_D3 = 6;
const int ESP_D4 = 4;
const int ESP_D5 = 2;
const int SpiCS_Pin = 10;
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
// #if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
// #define VSPI FSPI
// #endif
static const uint32_t spiClk = 100000;
SPIClass *vspi = NULL;
// SPIClass * hspi = NULL;

CRC crc;

bool passFailArray[100];
unsigned long previous;
byte command = 1;
byte slot_type = 3;

byte testData = 0;
byte redColor = 255;
byte greenColor = 255;
byte blueColor = 255;
float factor1, factor2;
int ind = 0;

void writeSlotBoard(byte slotNumber, byte command, byte slot_type)
{
  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);

  byte slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; // select the slot

  uint8_t misoData[100];
  for (int i = 0; i < 4; i++)
  {
    vspi->transfer(slotSelect[i]);
  }
  // delay(1);
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  byte colorData[] = {command, 0xFF, 0xFF, 0xFF, 0xFF, slot_type, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t checkSum = crc.checksumCalculator(colorData, 6);
  colorData[6] = checkSum;
  colorData[7] = checkSum >> 8;

  Serial.print("TX: ");
  for (int i = 0; i < 12; i++)
  {                                             // 8 instead of 11? both work
    misoData[i] = vspi->transfer(colorData[i]); // why do this?
    // vspi->transfer(colorData[i]);
    Serial.printf("%d ", colorData[i]);
  }
  Serial.println();
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  for (int i = 0; i < 4; i++)
  {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  digitalWrite(ESP_D4, LOW);

  // byte ackData[] = {'%', '\n', 0xFF, 0xFF, 0xFF, 0xFF,0xFF,0xFF,};
  byte ackData[] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, slot_type, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};

  Serial.print("RX: ");
  for (int i = 0; i < 12; i++)
  {
    misoData[i] = vspi->transfer(ackData[i]); // ack or color data? both seem to do the same
    Serial.printf("%d ", misoData[i]);
  }
  Serial.println();
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);


  uint16_t checkSumOfC3 = crc.checksumCalculator(misoData, 6);
  Serial.print(" [C3 0x");
  uint16_t checksumFromC3 = misoData[6] + (misoData[7] << 8);
  Serial.print(checksumFromC3, HEX);
  Serial.print(" ] [S3 0x");
  Serial.print(checkSum, HEX);
  for (int i = 0; i < 99; i++)
  {
    passFailArray[i] = passFailArray[i + 1];
  }
  if (checkSumOfC3 == checksumFromC3)
  {
    Serial.print("]  PASS  ");
    passFailArray[99] = true;
    EasyCAT_BufferIn.Cust.checksum_good++;
    EasyCAT_BufferIn.Cust.Data1 = misoData[1] + (misoData[2] << 8);
  }
  else
  {
    Serial.print("]  FAIL  ");
    passFailArray[99] = false;
    EasyCAT_BufferIn.Cust.checksum_bad++;
  }

  int passCount = 0;
  for (int i = 0; i < 100; i++)
  {
    if (passFailArray[i])
      passCount++;
  }
  Serial.print(passCount);
  Serial.println("/100");

  // delay(1);
  //   vspi->endTransaction();
}

void rainbowAnimation()
{
  int stepSize = 1023;
  int dividerVar = 341;
  int subVar = 682;
  switch ((int)((ind % stepSize) / dividerVar))
  {
  case 0:
    factor1 = 1.0 - ((float)(ind % stepSize - 0 * dividerVar) / dividerVar);
    factor2 = (float)((int)(ind - 0) % stepSize) / dividerVar;
    // strip_0.strip.setPixelColor(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
    redColor = int(255 * factor1 + 0 * factor2);
    greenColor = int(0 * factor1 + 255 * factor2);
    blueColor = int(0 * factor1 + 0 * factor2);
    break;
  case 1:
    factor1 = 1.0 - ((float)(ind % stepSize - 1 * dividerVar) / dividerVar);
    factor2 = (float)((int)(ind - dividerVar) % stepSize) / dividerVar;
    // strip_0.strip.setPixelColor(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
    redColor = int(0 * factor1 + 0 * factor2);
    greenColor = int(255 * factor1 + 0 * factor2);
    blueColor = int(0 * factor1 + 255 * factor2);
    break;
  case 2:
    factor1 = 1.0 - ((float)(ind % stepSize - 2 * dividerVar) / dividerVar);
    factor2 = (float)((int)(ind - subVar) % stepSize) / dividerVar;
    // strip_0.strip.setPixelColor(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
    redColor = int(0 * factor1 + 255 * factor2);
    greenColor = int(0 * factor1 + 0 * factor2);
    blueColor = int(255 * factor1 + 0 * factor2);
    break;
  }
  if (ind >= stepSize)
  {
    ind = 0;
  }
  else
    ind++;
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_IIC_SDA, PIN_IIC_SCL);
  if (result == false)
  {
    // Serial.println("PMU is not online...");
    while (1)
      delay(50);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  // Serial.println("ETHERCAT SETUP");
  pinMode(40, OUTPUT); // EC RST pin
  digitalWrite(40, HIGH);
  delay(100);
  EasyCAT_Init(SpiCS_Pin, ASYNC);
  delay(1000);

  Serial.println("PMU SETUP");
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
  // delay(1000);
  // vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  Serial.println("ESP D Pin Setup");
  pinMode(ESP_D0, OUTPUT);
  pinMode(ESP_D1, OUTPUT); // sck
  pinMode(ESP_D2, OUTPUT); // mosi
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT); // ss
  pinMode(ESP_D5, INPUT);  // miso

  //  digitalWrite(ESP_D1, HIGH);
  //  digitalWrite(ESP_D2, HIGH);
  //  digitalWrite(ESP_D3, HIGH);
  //  digitalWrite(ESP_D4, HIGH);
  //  digitalWrite(ESP_D0, HIGH);
  digitalWrite(ESP_D0, LOW);

  Serial.println("vspi declaration");
  vspi = new SPIClass(VSPI);
  //  vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  //  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  // Serial.println("Starting...");

  pinMode(ESP_D1, OUTPUT);
  pinMode(ESP_D2, OUTPUT);
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT);
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW); // ALL LOW is SMART with OE Enabled

  digitalWrite(ESP_D0, HIGH); // all is applied on rising edge
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW);
  digitalWrite(ESP_D0, LOW);

  Serial.println("vspi begin");
  vspi->begin(ESP_D1, ESP_D5, ESP_D2);
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  Serial.println("end of setup");
}

void loop()
{

  EasyCAT_MainTask();

  //  unsigned long animationStart = millis();
  int numberOfSlots = 1;

  if(EasyCAT_BufferOut.Cust.slot_type != 0){
    slot_type = EasyCAT_BufferOut.Cust.slot_type;
  }
  if(EasyCAT_BufferOut.Cust.command != 0){
    command = EasyCAT_BufferOut.Cust.command;
  }

  unsigned long current = millis();
  if (current - previous >= 10)
  {
    previous = current;
    // slot_type++;
    // if (slot_type > 8)
    // {
    //   slot_type = 1;
    // }

    for (int i = 1; i <= numberOfSlots; i++)
    {
      writeSlotBoard(i, command, slot_type);
      delay(20);
    }
  }
}
