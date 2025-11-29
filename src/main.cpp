// M5 + DMX
#include <M5Unified.h>  
#include <SPI.h>   
#include <esp_dmx.h> 

#include <M5Module_LAN.h> // M5 IP ADDRESS

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
EthernetClient modbusClient;
bool motorConnected = false;
uint16_t transactionID = 0;
// =============== M5 IP CONFIG ===============


void setupDMX();
void updateDMX();
void setupLAN();

bool connectToMotor();
void readMotorPosition();
void startContinuousSpeedControl(int32_t speed, float startAcc, float stopAcc);




void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  M5.Display.setTextSize(1.5);
  
  esp_task_wdt_delete(NULL);  // disable watchdog timer

  setupDMX();
  setupLAN();
  delay(2000);

  connectToMotor();

  // ADD THIS: Start continuous rotation after connection
  if (motorConnected) {
    delay(1000);  // Wait 1 second after connection

    Serial.println("\n🎪 INSTALLATION MODE");
    Serial.println("Starting continuous rotation...\n");

    // Speed and acceleration settings (adjust these!)
    int32_t speed = 1000;      // Rotation speed (try 500-2000)
    float startAcc = 1.0;      // Very slow start (lower = slower)
    float stopAcc = 1.0;       // Very slow stop (lower = slower)

    startContinuousSpeedControl(speed, startAcc, stopAcc);
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
  static uint8_t brightness = 0.0; //range: 0-255
  static uint8_t color = 0.0; //2700k(yellow)-6500k(white)

  static int8_t brightness_dir = 1;  // 1 = getting brighter, -1 = getting dimmer
  static int8_t color_dir = 1; // 1 = getting whiter, -1 = getting warmer

  static uint8_t brightness_speed = 1;  // loop speed
  static uint8_t color_speed = 1; // loop speed

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
}

void setupLAN() {
  // Serial.println("Init SPI...");
  // SPI.begin(SCK, MISO, MOSI, -1);

  M5.Ex_I2C.release();
  LAN.setResetPin(GPIO_NUM_13);
  LAN.reset();
  LAN.init(1);  // CS pin = 1 for CoreS3
  LAN.begin(mac, m5_ip, dns, gateway, subnet);  // Forstatic IP
  delay(500);

  uint8_t hwStatus = LAN.hardwareStatus();
  Serial.printf("HW Status: %d\n", hwStatus);
  
  Serial.println("Waiting for Ethernet..."); // Wait for link to be established
  uint32_t startTime = millis();
  while (LAN.linkStatus() != LinkON && millis() - startTime < 10000) {
    delay(100);
    Serial.print(".");
  }
  int linkStat = LAN.linkStatus();
  Serial.printf("Link status: %d\n", linkStat); // LinkOFF = 0, LinkON = 1, Unknown = 2
  

  if (LAN.linkStatus() == 1) {
    M5.Display.println();
    M5.Display.println();
    M5.Display.println("Link UP");
  } else {
    M5.Display.println();
    M5.Display.println();
    M5.Display.println("Link FAILED!");
  }

  delay(2000);

  M5.Display.println();
  M5.Display.println();
  M5.Display.print("IP = ");
  M5.Display.println(LAN.localIP());
}

bool connectToMotor(){
  if (motorConnected) return true;

  Serial.println("Connecting to motor...");
  IPAddress motor_ip = MOTOR_IP_ADDRESS;
  uint16_t motor_port = MOTOR_PORT;

  if (modbusClient.connect(motor_ip, motor_port)) {
      motorConnected = true;
      Serial.println("Motor Connected");
      M5.Display.fillRect(0, 150, 320, 20, BLACK);
      M5.Display.setCursor(0, 150);
      M5.Display.setTextColor(GREEN);
      M5.Display.println("Motor: Connected!");
      M5.Display.setTextColor(WHITE);
      return true;
    }
  Serial.println("Motor Connection FAILED!");
  M5.Display.fillRect(0, 150, 320, 20, BLACK);
  M5.Display.setCursor(0, 150);
  M5.Display.setTextColor(RED);
  M5.Display.println("Motor: Failed!");
  M5.Display.setTextColor(WHITE);
  return false;
}

bool modbusReadHoldingRegisters(uint16_t startAddress, uint16_t numRegisters, uint16_t* data){
  if(!motorConnected){
    Serial.println("Not connected to Motor!");
    return false;
  }

  uint8_t request[12];
  transactionID++;

  //Modbus Apllication Protocal
  request[0] = (transactionID >> 8) & 0xFF; // Transaction ID high byte
  request[1] = transactionID & 0xFF;        // Transaction ID low byte
  request[2] = 0x00;                        // Protocal ID high, o for Modbus
  request[3] = 0x00;                        // Protocal ID low
  request[4] = 0x00;                        // Length high byte
  request[5] = 0x06;                        // Length low byte (6 bytes following)
  request[6] = MOTOR_SLAVE_ID;                    // unit ID slave, why it is fxxking named slave.. 

  //Protocal Data Unit, actual modbus command
  request[7] = 0x03;                        // read holding registers
  request[8] = (startAddress >> 8) & 0xFF;  //Starting register address high
  request[9] = startAddress & 0xFF;         //Starting register address low
  request[10] = (numRegisters >> 8) & 0xFF; //Number of registers high
  request[11] = numRegisters & 0xFF;        //Number of registers low

  //Send request to motor
  Serial.printf("Reading %d registers from 0x%04x\n", numRegisters, startAddress);
  modbusClient.write(request, 12);
  modbusClient.flush();

  // Wait for response (timeout after 1000ms)
  uint32_t startTime = millis();
  while (modbusClient.available() < 9 && millis() - startTime < 1000) {
    delay(1);
  }
  
  if (modbusClient.available() < 9) {
    Serial.println("✗ Modbus timeout - no response!");
    return false;
  }

  // Read the response header (9 bytes)
  uint8_t response[9];
  modbusClient.readBytes(response, 9);

  // Check if motor returned an error
  if (response[7] & 0x80) {  // Error bit set
    Serial.printf("✗ Modbus error code: 0x%02X\n", response[8]);
    return false;
  }

  // Check function code matches
  if (response[7] != 0x03) {
    Serial.printf("✗ Wrong function code: 0x%02X\n", response[7]);
    return false;
  }

  uint8_t byteCount = response[8];
  Serial.printf("← Received %d bytes\n", byteCount);

  // Read the actual register values
  for (int i = 0; i < numRegisters; i++) {
    while (modbusClient.available() < 2 && millis() - startTime < 1000) {
      delay(1);
    }

    if (modbusClient.available() < 2) {
      Serial.println("✗ Timeout reading data!");
      return false;
    }

    uint8_t high = modbusClient.read();
    uint8_t low = modbusClient.read();
    data[i] = (high << 8) | low;  // Combine into 16-bit value
    Serial.printf("  Register %d: 0x%04X (%d)\n", i, data[i], data[i]);
  }

  return true;

}

bool modbusWriteMultipleRegisters(uint16_t startAddress, uint16_t numRegisters, uint16_t* data){
  if(!motorConnected){
    Serial.println("Not connected to motor!");
    return false;
  }

// Calculate PDU length: 1 (func) + 2 (addr) + 2 (count) + 1 (byte count) + data bytes
uint8_t byteCount = numRegisters * 2;  // MBAP header (7) + PDU (6 + data)
uint16_t pduLength = 7 + byteCount;

// Build request packet
uint8_t request[13 + byteCount];
transactionID++;

// MBAP Header
request[0] = (transactionID >> 8) & 0xFF;
request[1] = transactionID & 0xFF;
request[2] = 0x00;                         // Protocol ID high
request[3] = 0x00;                         // Protocol ID low
request[4] = (pduLength >> 8) & 0xFF;      // Length high
request[5] = pduLength & 0xFF;             // Length low
request[6] = MOTOR_SLAVE_ID;                     // Unit ID

// PDU - Function Code 0x10 (Write Multiple Registers)
request[7] = 0x10;                         // Function code
request[8] = (startAddress >> 8) & 0xFF;   // Start address high
request[9] = startAddress & 0xFF;          // Start address low
request[10] = (numRegisters >> 8) & 0xFF;  // Number of registers high
request[11] = numRegisters & 0xFF;         // Number of registers low
request[12] = byteCount;                   // Byte count

// Data
for (int i = 0; i < numRegisters; i++) {
  request[13 + i * 2] = (data[i] >> 8) & 0xFF;  // High byte
  request[13 + i * 2 + 1] = data[i] & 0xFF;     // Low byte
}

// Send request
Serial.printf("→ Writing %d registers to 0x%04X\n", numRegisters, startAddress);
modbusClient.write(request, 13 + byteCount);
modbusClient.flush();

// Wait for response (12 bytes for write multiple)
uint32_t startTime = millis();
while (modbusClient.available() < 12 && millis() - startTime < 1000) {
  delay(1);
}

if (modbusClient.available() < 12) {
  Serial.println("✗ Modbus timeout!");
  return false;
}

// Read response
uint8_t response[12];
modbusClient.readBytes(response, 12);

// Check for errors
if (response[7] & 0x80) {
  Serial.printf("✗ Modbus error: 0x%02X\n", response[8]);
  return false;
}

if (response[7] != 0x10) {
  Serial.printf("✗ Wrong function code: 0x%02X\n", response[7]);
  return false;
}

Serial.println("✓ Write successful!");
return true;

}

void readMotorPosition() {
  static uint32_t lastRead = 0;
  if (millis() - lastRead > 100){
    lastRead = millis();
    Serial.println("\n--- Reading Motor Position ---");
    uint16_t posData[2]; // Position is 32-bit = 2 registers

    if (modbusReadHoldingRegisters(ADDR_POS, 2, posData)){
      // Combine two 16-bit registers into one 32-bit position
      int32_t position = ((int32_t)posData[0] << 16) | posData[1];
      // M5 Display
      M5.Display.fillRect(0, 180, 320, 20, BLACK);
      M5.Display.setCursor(0, 180);
      M5.Display.printf("Pos: %d", position);
    } else {
      Serial.println("✗ Failed to read position");
    }
  }
}

void startContinuousSpeedControl(int32_t speed, float startAcc, float stopAcc) {
  Serial.printf("\n🔄 Starting continuous rotation\n");
  Serial.printf("   Speed: %d\n", speed);
  Serial.printf("   Start acceleration: %.1f (slow start)\n", startAcc);
  Serial.printf("   Stop acceleration: %.1f (slow stop)\n", stopAcc);

    // Prepare Direct Operation Data (12 registers)
    uint16_t moveData[12];

    moveData[0] = 0;                         // Reserved
    moveData[1] = 16;                        // Operation mode: 1 = Absolute Position
    moveData[2] = 0;                         // Target position LOW word
    moveData[3] = 0;                         // Target position HIGH word
    moveData[4] = speed & 0xFFFF;            // Velocity LOW (1000 = slow speed)
    moveData[5] = (speed >> 16) & 0xFFFF;    // Velocity HIGH
    // Convert acceleration to motor units (multiply by 1000)
    uint16_t startRate = (uint16_t)(startAcc * 1000);
    uint16_t stopRate = (uint16_t)(stopAcc * 1000);

    moveData[6] = startRate & 0xFFFF;        // Start rate LOW (10.0)
    moveData[7] = (startRate >> 16) & 0xFFFF;// Start rate HIGH
    moveData[8] = stopRate & 0xFFFF;         // Stop rate LOW (10.0)
    moveData[9] = (stopRate >> 16) & 0xFFFF; // Stop rate HIGH
    moveData[10] = 1000;                     // Operating current (100%)
    moveData[11] = 0;                        // Current HIGH

    // Write movement data to motor
    if (modbusWriteMultipleRegisters(ADDR_TRIG_MODE, 12, moveData)) {
      delay(50);

      // Trigger the movement
      uint16_t triggerData[1] = {0x0100};  // Trigger command
      if (modbusWriteMultipleRegisters(ADDR_TRIG_MODE, 1, triggerData)) {
        Serial.println("✓ Continuous rotation started!");

        M5.Display.fillRect(0, 200, 320, 20, BLACK);
        M5.Display.setCursor(0, 200);
        M5.Display.setTextColor(GREEN);
        M5.Display.println("RUNNING");
        M5.Display.setTextColor(WHITE);
      }
    }
}

void stopMotor() {
  Serial.println("🛑 Stopping motor (deceleration)...");

  uint16_t stopCmd[1] = {1 << 5};  // Bit 5 = STOP

  if (modbusWriteMultipleRegisters(ADDR_STATIC_IO_IN, 1, stopCmd)) {
    Serial.println("✓ Stop command sent");

    M5.Display.setCursor(0, 210);
    M5.Display.setTextColor(RED);
    M5.Display.println("STOPPED!");
    M5.Display.setTextColor(WHITE);
  }
}


void loop() {
  M5.update();
  updateDMX();

  if(motorConnected){
    readMotorPosition();
    }

  

  yield(); // let system handble background task
  delay(1000/60);
}

