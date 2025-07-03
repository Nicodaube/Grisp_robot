#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BAND 433E6
#define RX1_PIN 2
#define TX1_PIN 15

#define LORA_MOSI 27
#define LORA_MISO 19
#define LORA_CLK 5
#define LORA_DIO0 26
#define LORA_RST 23
#define LORA_SS 18

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     4
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

  Serial.println("[LORA_TRANSMITTER] LoRa setting up ...");
  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if(!LoRa.begin(BAND)) {
    Serial.println("[LORA_TRANSMITTER] Starting LoRa Failed !");
    while(1);
  }
  Serial.println("[LORA_TRANSMITTER] LoRa Ready !");

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(32, 32);
  display.printf("TRANSMITTER");
  display.display();
}

void loop() {
  if (Serial1.available()) {
    uint8_t buf[256];
    int len = 0;
    while (Serial1.available() && len < sizeof(buf)) {
      buf[len++] = Serial1.read();
    }
    LoRa.beginPacket();
    LoRa.write(buf, len);
    LoRa.endPacket();
  }

}
