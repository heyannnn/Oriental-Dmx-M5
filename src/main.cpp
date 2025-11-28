// M5 + DMX[DONE]
#include <M5Unified.h>  
#include <SPI.h>   
#include <esp_dmx.h> 

//#include "M5_Ethernet.h" 
//#include <ModbusTCP.h>  

// =============== DMX CONFIG ===============
#define DMX_TX_PIN GPIO_NUM_7
#define DMX_RX_PIN GPIO_NUM_10
#define DMX_EN_PIN GPIO_NUM_6

dmx_port_t dmxPort = DMX_NUM_1;  // Use UART1
uint8_t data[DMX_PACKET_SIZE];   // DMX data buffer (513 bytes)
// =============== DMX CONFIG ===============


void setup() {
  M5.begin();
  M5.Display.setTextSize(2);
  
  Serial.begin(115200);
  delay(1000);
  
  // =============== DMX CONFIG ===============
  dmx_config_t config = DMX_CONFIG_DEFAULT;   // 1. Configure DMX with default settings
  dmx_driver_install(dmxPort, &config, NULL, 0);   // 3. Install driver
  dmx_set_pin(dmxPort, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);   // 2. Set GPIO pins (TX, RX, EN)
  memset(data, 0, DMX_PACKET_SIZE);

  delay(1000);
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(GREEN);
  M5.Display.setCursor(10, 10);
  M5.Display.println("DMX FADING...");
  M5.Display.setCursor(10, 40);
  M5.Display.printf("Bright:", data[1]);
  M5.Display.setCursor(10, 70);
  M5.Display.printf("Color:", data[2]);
  // =============== DMX CONFIG ===============

  
}

void loop() {
  M5.update();

  // =============== DMX CONFIG ===============
  static uint8_t brightness = 0; //range: 0-255
  static uint8_t color = 0; //2700k(yellow)-6500k(white)

  static int8_t brightness_dir = 1;  // 1 = getting brighter, -1 = getting dimmer
  static int8_t color_dir = -1; // 1 = getting whiter, -1 = getting warmer

  static int8_t brightness_speed = 2;  // loop speed
  static int8_t color_speed = 1; // loop speed

  brightness += brightness_dir * brightness_speed; 
  color += color_dir * color_speed;

  if (brightness >= 50) brightness_dir = -1;  // Reverse direction at limits
  if (brightness <= 0) brightness_dir = 1;

  if (color >= 255) color_dir = -1;   // Reverse direction at limits
  if (color <= 0) color_dir = 1;

  data[1] = brightness;  // Channel 1 
  data[2] = color;  // Channel 2 

  dmx_write(dmxPort, data, DMX_PACKET_SIZE); // write
  dmx_send(dmxPort); // send
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  static uint32_t lastUpdate = 0; // print the value
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    // Clear and update brightness number
    M5.Display.fillRect(100, 40, 60, 16, BLACK);
    M5.Display.setCursor(100, 40);
    M5.Display.print(data[1]);

    // Clear and update color number
    M5.Display.fillRect(100, 70, 60, 16, BLACK);
    M5.Display.setCursor(100, 70);
    M5.Display.print(data[2]);
  }

  delay(30);  // 30ms = ~33 updates/sec for DMX
  
}