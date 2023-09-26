#include <ESP32SPISlave.h>
#include <Arduino.h>
#include <atomic>
#include <Adafruit_NeoPixel.h>
#include <CRC.h>
#include <Pin_config_C3.h>
#include <Reset_Reason_C3.h>


ESP32SPISlave slave;
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
byte newLEDdata[3];
boolean newDataAvail = false;
static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// Global atomic variables SLOT TYPE and DATA OUT
std::atomic<int> data_out(0);
std::atomic<int> slot_type(0);

CRC crc;

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

void set_buffer()
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    spi_slave_tx_buf[i] = (0xFF - i) & 0xFF;
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

    if (spi_slave_rx_buf[0] == '$' && spi_slave_rx_buf[4] == '\n') {
      uint16_t checkSum = crc.checksumCalculator(spi_slave_rx_buf, 5);
      // spi_slave_tx_buf[0] = checkSum;
      // spi_slave_tx_buf[1] = checkSum >> 8;
      spi_slave_tx_buf[0] = 0x01;
      spi_slave_tx_buf[1] = 0x02;
      spi_slave_tx_buf[2] = 0x03;
      spi_slave_tx_buf[3] = 0x04;
      spi_slave_tx_buf[4] = 0x05;
      spi_slave_tx_buf[5] = 0x06;
      spi_slave_tx_buf[6] = 0x07;
      spi_slave_tx_buf[7] = 0x08;
      // byte b1 = 0x30;

      // spi_slave_tx_buf[0] = b1;//spi_slave_rx_buf[0];
      // spi_slave_tx_buf[1] = spi_slave_rx_buf[1];
      // spi_slave_tx_buf[2] = spi_slave_rx_buf[2];
      // spi_slave_tx_buf[3] = 5;//spi_slave_rx_buf[3];
      // spi_slave_tx_buf[4] = spi_slave_rx_buf[4];
      // // spi_slave_tx_buf[5] = spi_slave_rx_buf[5];
      // // spi_slave_tx_buf[6] = spi_slave_rx_buf[6];
      // spi_slave_tx_buf[7] = spi_slave_rx_buf[7];
      // spi_slave_tx_buf[5] = checkSum;
      // spi_slave_tx_buf[6] = checkSum >> 8;

      slot_type.store(spi_slave_rx_buf[2]);

      // Serial.print("TX: ");
      // for (size_t i = 0; i < BUFFER_SIZE; ++i)
      // {
      //   Serial.printf("%d ", spi_slave_tx_buf[i]);
      // }
      // Serial.printf("\n");

      // newDataAvail = true;
      // newLEDdata[0] = spi_slave_rx_buf[1];
      // newLEDdata[1] = spi_slave_rx_buf[2];
      // newLEDdata[2] = spi_slave_rx_buf[3];
    }

    slave.pop();

    xTaskNotifyGive(task_handle_wait_spi);
  }
}

void setup()
{
  Serial.begin(115200);
  slave.setDataMode(SPI_MODE0);

  gpio_set_drive_capability((gpio_num_t)ESP_D5, GPIO_DRIVE_CAP_1);
  slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
  set_buffer();

  xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
  xTaskNotifyGive(task_handle_wait_spi);
  xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);
}

void loop()
{

  uint8_t type = slot_type.load();
  RGBled.setPixelColor(0, primaryColors[type]);
  RGBled.show();
  delayMicroseconds(100);

  // if (newDataAvail) {
  //   newDataAvail = false;
  //   RGBled.setPixelColor(0, RGBled.Color(newLEDdata[0], newLEDdata[1], newLEDdata[2])); // Moderately bright green color.
  //   RGBled.show();
  // }
}
