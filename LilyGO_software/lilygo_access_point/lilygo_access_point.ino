#include <WiFi.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCLK 14
#define PIN_CS   26

const char* ssid     = "CageNet"; // To replace with GenNet for the other rooms
const char* password = "oui123456";

WiFiServer server(80); // WiFi access point port
uint8_t rcv_buf[128]; // SPI receiving buffer

void setup_wifi() {  
  Serial.print("[CAGENET] Setting up WiFI Access Point…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("[CAGENET] AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void setup_spi_slave() {
  Serial.print("[CAGENET] initiliazing SPI slave ...");
  spi_bus_config_t buss_cofiguration = {
    .mosi_io_num = PIN_MOSI,
    .miso_io_num = PIN_MISO,
    .sclk_io_num = PIN_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 128
  };

  spi_slave_interface_config_t slave_configuration = {
    .mode = 0,
    .spics_io_num = PIN_CS,
    .queue_size = 3,
    .flags = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = NULL
  };

  esp_err_t ret;
  ret = spi_slave_initialize(HSPI_HOST, &buss_cofiguration, &slave_configuration, SPI_DMA_DISABLED);
  assert(ret == ESP_OK);
  Serial.print("[CAGENET] SPI slave ready");
}

void setup(){
  Serial.begin(115200);
  setup_wifi_ap();
  setup_spi_slave();
}

void loop() {
  spi_slave_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8 * sizeof(recv_buf);  // True size of transmission in bits
  t.rx_buffer = recv_buf; // Sets the receiving buffer

  esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY); // Wait for data coming from spi

  if (ret == ESP_OK) {
    Serial.print("Received via SPI: ");
    for (int i = 0; i < t.trans_len / 8; i++) {
      Serial.print((char)recv_buf[i]);
    }
    Serial.println();
  }

  delay(100);
}