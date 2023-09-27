#include <ESP32SPISlave.h>
#include <Arduino.h>
#include <atomic>
#include <Adafruit_NeoPixel.h>
#include <CRC.h>
#include <Pin_config_C3.h>
#include <Reset_Reason_C3.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <PWM.hpp>
#include <ble_ota.h>
#include <WiFi.h>

const char VERSION[] = "V4_C3";
const char *filename = "/config.txt";
ESP32SPISlave slave;
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
byte newLEDdata[3];
boolean newDataAvail = false;
bool ble_enabled = false;
static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// Global atomic variables SLOT TYPE and DATA OUT
std::atomic<int> data_out_atom(0);
std::atomic<int> slot_type_atom(0);
std::atomic<int> ble_state_atom(0);

Adafruit_MCP4725 dac;
CRC crc;
PWM my_pwm(SLOT_IO0pin);

const int pwmResolution = 10;
int freq_value = 0;
int freq_value_last = 0;
int pwm_value = 0;
int pwm_value_last = 0;
bool low_speed_freq = false;
int pwmState = LOW;

int slot_number;
int slot_type;
int ble_state;

struct Config{
  int slot_number_json;
  int slot_type_json;
};

Config config;

uint32_t primaryColors[10] = {
    // SWAP RED AND GREEN
    RGBled.Color(0, 0, 0),       // off
    RGBled.Color(0, 255, 0),     // Red
    RGBled.Color(255, 255, 255), // White
    RGBled.Color(0, 128, 128),   // Purple
    RGBled.Color(165, 255, 0),   // Orange
    RGBled.Color(0, 0, 255),     // Blue
    RGBled.Color(255, 0, 0),     // Green
    RGBled.Color(255, 255, 0),   // Yellow
    RGBled.Color(169, 169, 169), // Grey
    RGBled.Color(69, 139, 19)    // Brown
};


void loadConfiguration(const char *filename, Config &config) {
  Serial.println(F("Checking Config File"));
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open(filename, "r");
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println(F("Failed to read file..."));
    }
    config.slot_number_json = doc["slot_number_json"];
    config.slot_type_json = doc["slot_type_json"];
  } else {
    Serial.println(F("SPIFFS FAULT"));
  }
}

void saveConfiguration(const char *filename, const Config &config) {
  Serial.println(F("saving config file..."));
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open(filename, "w");
    if (!file) {
      Serial.println(F("Failed to save config file..."));
      return;
    }
    StaticJsonDocument<2000> doc;
    doc["slot_number_json"] = config.slot_number_json;
    doc["slot_type_json"] = config.slot_type_json;
    if (serializeJson(doc, file) == 0) {
      Serial.println(F("Failed to write to file"));
    }
    file.close();
  }
}

void set_buffer()
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    spi_slave_tx_buf[i] = 0xFF;
  }
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

void task_wait_spi(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // block until the transaction comes from master
    slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    xTaskNotifyGive(task_handle_process_buffer);
  }
}

void task_process_buffer(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // show received data
    // Serial.print("RX: ");
    // for (size_t i = 0; i < BUFFER_SIZE; ++i)
    // {
    //   Serial.printf("%d ", spi_slave_rx_buf[i]);
    // }
    // Serial.println();
    uint16_t checkSumFromS3 = spi_slave_rx_buf[6] + (spi_slave_rx_buf[7] << 8);
    uint16_t checkSumOfS3 = crc.checksumCalculator(spi_slave_rx_buf, 6);
    uint16_t data = data_out_atom.load();
    int command = 0;

    if (checkSumOfS3 == checkSumFromS3 && checkSumFromS3 != 0)
    {
      command = spi_slave_rx_buf[0];    
      spi_slave_tx_buf[0] = spi_slave_rx_buf[0];
      spi_slave_tx_buf[1] = data;
      spi_slave_tx_buf[2] = data >> 8;
      spi_slave_tx_buf[3] = spi_slave_rx_buf[3];  // data2
      spi_slave_tx_buf[4] = spi_slave_rx_buf[4];  // data2 >> 8
      spi_slave_tx_buf[5] = spi_slave_rx_buf[5];  // data3


      uint16_t checkSumOfC3 = crc.checksumCalculator(spi_slave_tx_buf, 6);
      spi_slave_tx_buf[6] = checkSumOfC3;
      spi_slave_tx_buf[7] = checkSumOfC3 >> 8;
    }
      
    if(command == 5)
    {
      slot_type_atom.store(spi_slave_rx_buf[5]);
    }
    else if(command == 6)
    {
      ble_state_atom.store(spi_slave_rx_buf[5]);
    }
    // Serial.print("TX: ");
    // for (size_t i = 0; i < BUFFER_SIZE; ++i)
    // {
    //   Serial.printf("%d ", spi_slave_tx_buf[i]);
    // }
    // Serial.printf("\n");

    slave.pop();

    xTaskNotifyGive(task_handle_wait_spi);
  }
}

void setup()
{
  Serial.begin(115200);
  slave.setDataMode(SPI_MODE0);

  loadConfiguration(filename,config);

  slot_type = config.slot_type_json;
  slot_number = config.slot_number_json;
  
  if(0 < slot_type < 9)
  {
    RGBled.setPixelColor(0, primaryColors[slot_type]);
    RGBled.setBrightness(128);
    RGBled.show();
  }

  switch (slot_type)
  {
  case 1:       // Digital Output
    pinMode(DIGI_OUTpin,OUTPUT);
    break;
  case 2:       // Digital Input
    pinMode(SLOT_IO0pin,INPUT);
    break;
  case 3:       // Analog Input
    pinMode(SLOT_IO0pin,INPUT);
    break;
  case 4:       // Analog Output
    //dac
    Wire.begin(SCL,SDA);
    dac.begin(0x62);
    break;
  case 5:       // PWM (input)
    // call my_pwm.begin(true) in loop to get duty cycle
    break;
  case 6:       // Freq (output)
    pinMode(DIGI_OUTpin,OUTPUT);
    break;
  case 7:
    break;
  case 8:
    break;
  default:
    // while(1){      // blink red error
    //   RGBled.setPixelColor(0, primaryColors[0]);
    //   RGBled.show();
    //   delay(200);
    //   RGBled.setPixelColor(0, primaryColors[1]);
    //   RGBled.show();
    //   delay(200);
    // }
    config.slot_type_json = 1;
    config.slot_number_json = 0;
    saveConfiguration(filename,config);
    delay(1000);
    ESP.restart();
    break;
  }
  
  pinMode(SLOT_TP1pin,OUTPUT);

  gpio_set_drive_capability((gpio_num_t)ESP_D5, GPIO_DRIVE_CAP_1);
  slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
  set_buffer();

  xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
  xTaskNotifyGive(task_handle_wait_spi);
  xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);
}

void loop()
{
  digitalWrite(SLOT_TP1pin,HIGH);
  slot_type = slot_type_atom.load();
  ble_state = ble_state_atom.load();
  if(!ble_enabled)
  {
    RGBled.setPixelColor(0, primaryColors[slot_type]);
    RGBled.setBrightness(128);
    RGBled.show();
  }
  uint16_t data;

  switch (slot_type)
  {
  case 1:
    data = 1000;
    digitalWrite(SLOT_IO0pin,HIGH);   
    break;
  case 2:
    data = digitalRead(SLOT_IO0pin);
    break;
  case 3:
    data = analogRead(SLOT_IO0pin);
    break;
  case 4:
    data = 4000;
    // dac.setVoltage(512,false);
    break;
  case 5:
    data = my_pwm.getValue();
    break;
  case 6:
    data = 6000;
    // Freq output
    // ledcWrite()
    break;
  case 7:
    data = 7000;
    break;
  case 8:
    data = 8000;
    break;
  default:
    data = 0;
    break;
  }
  data_out_atom.store(data);

  // BLE OTA check
  if(ble_state == 1 && ble_enabled == false){
    // BLE Configuration
    String slot = "SLOT_";
    String mac = WiFi.macAddress();
    ota_dfu_ble.begin(slot+mac);
    ble_enabled = true;
    RGBled.setPixelColor(0, primaryColors[5]);
    RGBled.setBrightness(128);
    delay(500);
  }
  else if((ble_state == 0) && (ble_enabled == true)){
    RGBled.setPixelColor(0, primaryColors[6]);
    RGBled.setBrightness(128);
    ESP.restart();    // if ble is on and received message to turn off, reboot ESP
  }
  else if(ble_state == 2){
    ESP.restart();    // if ble is on and received message to turn off, reboot ESP
  }

  delayMicroseconds(100);
  digitalWrite(SLOT_TP1pin,LOW);
}
