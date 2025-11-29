// M5 + DMX
#include <M5Unified.h>  
#include <SPI.h>   
#include <esp_dmx.h> 

#include <M5Module_LAN.h> // M5 IP ADDRESS

//#include <ModbusTCP.h>  // ORIENTAL MOTOR
//#include <ModbusIP_ESP8266.h> 
#include "motor_registers.h"
#include <esp_task_wdt.h> 

// =============== DMX CONFIG ===============
#define DMX_TX_PIN GPIO_NUM_7 /// GPIO_NUM_7
#define DMX_RX_PIN GPIO_NUM_10 // GPIO_NUM_10
#define DMX_EN_PIN GPIO_NUM_6 // GPIO_NUM_6



dmx_port_t dmxPort = DMX_NUM_1;  // Use UART1
uint8_t data[DMX_PACKET_SIZE];   // DMX data buffer (513 bytes)
// =============== DMX CONFIG ===============



// =============== M5 IP CONFIG ===============

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress m5_ip(192, 168, 100, 30); 
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 100, 1);   // My PC/router
IPAddress dns(192, 168, 100, 1);       // Usually same

M5Module_LAN LAN;
EthernetClient ethClient;
EthernetClient client;
//ModbusTCP mb;
bool motorConnected = false;
// =============== M5 IP CONFIG ===============



void setupDMX();
void updateDMX();
void setupLAN();


void setupModbus();
void readMotorPosition();
void moveMotor(int32_t position);
void testMotorTCP();


void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  M5.Display.setTextSize(1.5);
  
  // disable watchdog timer
  esp_task_wdt_delete(NULL); 

  //setupDMX();
  setupLAN();
  delay(10000);
  //setupModbus();
  testMotorTCP();

}

void testMotorTCP(){
  Serial.println("Testing TCP connection to motor...");
  M5.Display.setCursor(10,150);
  M5.Display.print("TCP test...");

  EthernetClient client;
  IPAddress motor_ip = MOTOR_IP_ADDRESS;
  uint16_t motor_port = MOTOR_PORT;

  Serial.printf("Connecting to %s:%d\n", motor_ip.toString().c_str(), motor_port);
  if (client.connect(motor_ip, motor_port)) {
    Serial.println("TCP Connection SUCCESS!");
    M5.Display.fillRect(0, 150, 320, 20, BLACK);
    M5.Display.setCursor(10, 150);
    M5.Display.setTextColor(GREEN);
    M5.Display.println("TCP: Connected!");
    M5.Display.setTextColor(WHITE);

    motorConnected = true;
    client.stop();
  } else {
    Serial.println("TCP Connection FAILED!");
    M5.Display.fillRect(0, 150, 320, 20, BLACK);
    M5.Display.setCursor(10, 150);
    M5.Display.setTextColor(RED);
    M5.Display.println("TCP: Failed!");
    M5.Display.setTextColor(WHITE);
  }

}
void setupDMX(){
  dmx_config_t config = DMX_CONFIG_DEFAULT;   // 1. Configure DMX with default settings
  dmx_driver_install(dmxPort, &config, NULL, 0);   // 3. Install driver
  dmx_set_pin(dmxPort, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);   // 2. Set GPIO pins (TX, RX, EN)
  memset(data, 0, DMX_PACKET_SIZE);

  delay(1000);
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(GREEN);
  M5.Display.setCursor(0, 10);
  M5.Display.println("DMX ING...");
  M5.Display.setCursor(0, 40);
  M5.Display.printf("Bright:", data[1]);
  M5.Display.setCursor(0, 70);
  M5.Display.printf("Color:", data[2]);

  }

void updateDMX() {
  static uint8_t brightness = 0; //range: 0-255
  static uint8_t color = 0; //2700k(yellow)-6500k(white)

  static int8_t brightness_dir = 1;  // 1 = getting brighter, -1 = getting dimmer
  static int8_t color_dir = 1; // 1 = getting whiter, -1 = getting warmer

  static int8_t brightness_speed = 1;  // loop speed
  static int8_t color_speed = 1; // loop speed

  brightness += brightness_dir * brightness_speed; 
  color += color_dir * color_speed;

  if (brightness >= 50) brightness_dir = -1;  // Reverse direction at limits
  if (brightness <= 0) brightness_dir = 1;

  if (color >= 255) color_dir = -1;   // Reverse direction at limits
  if (color <= 0) color_dir = 1;

  data[1] = brightness;
  data[2] = color;
  dmx_write(dmxPort, data, DMX_PACKET_SIZE); // write
  dmx_send(dmxPort); // send
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  static uint32_t lastUpdate = 0; // print the value
  if (millis() - lastUpdate > 500) {
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
}

void setupLAN() {
  // Serial.println("Init SPI...");
  // SPI.begin(SCK, MISO, MOSI, -1);

  Serial.println("Release I2C...");
  M5.Ex_I2C.release();
  LAN.setResetPin(GPIO_NUM_13);
  LAN.reset();
  LAN.init(1);  // CS pin = 1 for CoreS3

  Serial.println("Begin LAN...");
  LAN.begin(mac, m5_ip, dns, gateway, subnet);  // Forstatic IP
  
  delay(500);

  uint8_t hwStatus = LAN.hardwareStatus();
  M5.Display.printf("HW Status: %d\n", hwStatus);

  // Wait for link to be established
  Serial.println("Waiting for Ethernet...");
  uint32_t startTime = millis();
  while (LAN.linkStatus() != LinkON && millis() - startTime < 10000) {
    delay(100);
    M5.Display.print(".");
  }

  int linkStat = LAN.linkStatus();
  Serial.printf("Link status: %d\n", linkStat);
  // LinkOFF = 0, LinkON = 1, Unknown = 2

  if (LAN.linkStatus() == 1) {
    M5.Display.println();
    M5.Display.println("Link UP");
  } else {
    M5.Display.println();
    M5.Display.println("Link FAILED!");
  }

  delay(2000);

  M5.Display.println();
  M5.Display.println();
  M5.Display.print("IP = ");
  M5.Display.println(LAN.localIP());
}




// void setupModbus() {
//   mb.client();  // ModbusTCPクライアントとして設定
// }

// void connectToMotor(){
//   static uint32_t lastAttempt = 0;
//   static bool attemptedOnce = false;

//   EthernetClient client;
//   if(motorConnected) return;
//   uint32_t interval= attemptedOnce ? 10000 : 5000;

//   if (millis() - lastAttempt > interval){
//     lastAttempt = millis();
//     attemptedOnce = true;

//     Serial.println("Attempting to connect to motor...");
//     M5.Display.fillRect(0, 150, 320, 20, BLACK);
//     M5.Display.setCursor(10, 150);
//     M5.Display.print("Motor: Connecting...");

//     Serial.println("Connecting to Oriental Motor...");
//     IPAddress oriental_motor_ip = MOTOR_IP_ADDRESS;

//     if (mb.connect(oriental_motor_ip)) {
//       motorConnected = true;
//       M5.Display.setTextColor(GREEN);
//       M5.Display.println("Connected");
      
//     } else {
//       M5.Display.setTextColor(RED);
//       M5.Display.println("Failed");
//     }

//   }
// }
// void readMotorPosition() {
//   // mb.readHoldingRegisters(REG_CURRENT_POSITION, ...);
// }

// void moveMotor(int32_t position) {
//   // mb.writeRegister(REG_POSITION_COMMAND, position);
// }

void loop() {
  M5.update();
  //updateDMX();
  // connectToMotor();

  // if(motorConnected){
  //   mb.task();
  // }

//yield(); // let system handble background task
delay(30);
}

