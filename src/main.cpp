#include <M5Unified.h>  // M5 
#include <SPI.h>   // M5 
#include <esp_dmx.h> // DMX
#include <M5Module_LAN.h> // M5 IP ADDRESS
#include "motor_registers.h" // Oriental Motor Registers
#include <esp_task_wdt.h>  // watchdog for unrebooting
#include <math.h>
#include "angle8_control.h" 

// ===============   DMX CONFIG   ===============
#define DMX_TX_PIN GPIO_NUM_7 /// GPIO_NUM_7
#define DMX_RX_PIN GPIO_NUM_10 // GPIO_NUM_10
#define DMX_EN_PIN GPIO_NUM_6 // GPIO_NUM_6

dmx_port_t dmxPort = DMX_NUM_1;  // Use UART1
uint8_t data[DMX_PACKET_SIZE];   // DMX data buffer (513 bytes)
const float GAMMA = 2.2;
float calculateVariableSpeed(float current_brightness, float max_brightness, float base_speed);
// ===============   DMX CONFIG   ===============

// ===============   8ANGLE GLOBAL CONFIG   ===============
M5_ANGLE8 angle8;
bool angle8_found = false;
AngleInputs angleInputs;
// ===============   8ANGLE GLOBAL CONFIG   ===============

// ===============  M5 IP CONFIG  ===============
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// IPAddress m5_ip(192, 168, 100, 30); 
// IPAddress subnet(255, 255, 255, 0);
// IPAddress gateway(192, 168, 100, 1);   // My PC/router
// IPAddress dns(192, 168, 100, 1);   
IPAddress m5_ip(192, 168, 1, 30); 
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 1);   // My PC/router
IPAddress dns(192, 168, 1, 1);       // Usually same

M5Module_LAN LAN;
EthernetClient modbusClient;
// ===============  M5 IP CONFIG   ===============


// ===============  ORIENTAL MOTOR ===============
bool motorConnected = false;
uint16_t transactionID = 0;
bool justReconnected = false;

int32_t oriental_motorSpeed = -500; //500-5000 (swap direction -500)
float oriental_motorStartAcc = 0.2; //0.2-5
float oriental_motorStopAcc = 0.2; //0.2-5
// ===============  ORIENTAL MOTOR ===============

// ===============  DISPLAY FLAGS  ===============
bool auto_mode_firstDraw = true;  // Global flag for auto mode display
// ===============  DISPLAY FLAGS  ===============



// =============== FUNCTION DEFINE ===============
void setupDMX();
void updateDMX();
void updateDMX_Manual();
void updateDMX_Auto();
void setupLAN();

bool connectToMotor();
bool checkMotorConnection();
void readMotorPosition();
void startContinuousSpeedControl(int32_t speed, float startAcc, float stopAcc);
void performHoming();
bool checkAlarmsStatus();
void clearAlarms();
// =============== FUNCTION DEFINE ===============


void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  M5.Display.setTextSize(1.5);

  esp_task_wdt_delete(NULL);  // disable watchdog timer

  // Initialize 8Angle unit
  Serial.println("\nInitializing 8Angle...");
  M5.Display.setCursor(0, 100);
  if (angle8_found) {
    M5.Display.setTextColor(GREEN);
    M5.Display.println("8Angle: OK");
    Serial.println("✓ 8Angle is connected!");
  } else {
    M5.Display.setTextColor(RED);
    // M5.Display.println("8Angle: FAIL");
    Serial.println("✗ 8Angle NOT connected!");
  }
  M5.Display.setTextColor(WHITE);

  delay(1000);

  setupDMX();
  setupLAN();
  init8Angle();
  delay(1000);
  
  connectToMotor();

  // ADD THIS: Start continuous rotation after connection
  if (motorConnected) {
    delay(1000);  // Wait 1 second after connection
    if(checkAlarmsStatus()){
      clearAlarms();
      performHoming();
    }
    Serial.println("\n🎪 INSTALLATION MODE");
    Serial.println("Starting continuous rotation...\n");

    startContinuousSpeedControl(oriental_motorSpeed, oriental_motorStartAcc, oriental_motorStopAcc);
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
  
float calculateVariableSpeed(float current_brightness, float max_brightness, float base_speed) {
  float min_speed = base_speed;   // Define minimum speed
  float max_speed = base_speed + 1;   // Define maximum speed

  float position = current_brightness / max_brightness;  // 0.0 to 1.0
  float speed = base_speed + 1.6 * exp(-position * 10.0);
  return speed; 
}

void updateDMX() {
  static uint8_t prev_mode = 255;

  if (angleInputs.mode_switch != prev_mode) {
    M5.Display.fillScreen(BLACK);
    auto_mode_firstDraw = true;  // Reset auto mode display flag
    prev_mode = angleInputs.mode_switch;
  }

  // Check mode switch and route to appropriate function
  if (angleInputs.mode_switch == 0) {
    // Switch 0 = Manual mode
    updateDMX_Manual();
  } else {
    // Switch 1 = Auto mode
    updateDMX_Auto();
  }
}

void updateDMX_Manual() {
  static uint8_t last_mode_state = 255;

  // Clear on entering manual mode (optional, for consistency)
  if (last_mode_state != 0) {
    last_mode_state = 0;
    // Could add firstDraw here too if manual mode needs it
  }

  static float min_brightness = 0.0;   // Minimum brightness floor
  static float max_brightness = 255.0;  // Maximum brightness

  // Map 8Angle inputs to DMX values
  float brightness = map_manual_BrightnessNonLinear(angleInputs.ch1, min_brightness, max_brightness);
  float color = map_manual_ColorLinear(angleInputs.ch2);

  // ADD THIS DEBUG OUTPUT:
  Serial.printf("Raw CH1: %d -> Brightness: %.1f\n", angleInputs.ch1, brightness);
  Serial.printf("Raw CH2: %d -> Color: %.1f\n", angleInputs.ch2, color);

  // Clamp values
  brightness = constrain(brightness, min_brightness, max_brightness);
  color = constrain(color, 0.0, 255.0);

  // Send to DMX
  data[1] = (uint8_t)brightness;
  data[2] = (uint8_t)color;

  dmx_write(dmxPort, data, DMX_PACKET_SIZE);
  dmx_send(dmxPort);
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  // Display update
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    // Show mode
    M5.Display.fillRect(0, 0, 320, 30, BLACK);
    M5.Display.setCursor(0, 10);
    M5.Display.setTextColor(CYAN);
    M5.Display.println("MANUAL MODE");
    M5.Display.setTextColor(WHITE);

    // Show brightness value
    M5.Display.fillRect(0, 40, 320, 20, BLACK);
    M5.Display.setCursor(0, 40);
    M5.Display.printf("Bright: %d", data[1]);

    // Show color value
    M5.Display.fillRect(0, 70, 320, 20, BLACK);
    M5.Display.setCursor(0, 70);
    M5.Display.printf("Color: %d", data[2]);
  }
  
}

void updateDMX_Auto() {
  // === STATE VARIABLES (keep between loop calls) ===
  static float brightness = 0.0;    // Current brightness value
  static float brightness_dir = 1;    // 1 = increasing, -1 = decreasing

  static float color = 0.0;         // Current color value
  static float color_dir = 1;         // 1 = increasing, -1 = decreasing

  // === STEP 1: Read and map knob inputs ===
  // Map CH1 (raw 0-254) to min_brightness (0-255)
  float min_brightness = map_auto_min_Brightness(angleInputs.ch1);
  float max_brightness = map_auto_max_Brightness(angleInputs.ch2);
  float base_speed = map_auto_base_Speed(angleInputs.ch3); // Base speed setting
  
  if (min_brightness > max_brightness) {
    float temp = min_brightness;
    min_brightness = max_brightness;
    max_brightness = temp;
  }

  // Prevent zero or negative ranges
  if (max_brightness < 1.0) {
    max_brightness = 1.0;  // Minimum value to prevent division by zero
  }
  if (min_brightness < 0.0) {
    min_brightness = 0.0;
  }

  float current_speed = calculateVariableSpeed(brightness, max_brightness, base_speed);

  // Update brightness position
  brightness += brightness_dir * current_speed;

  // Check boundaries and reverse direction
  if (brightness >= max_brightness) {
    brightness = max_brightness;
    brightness_dir = -1;  // Start going down
  }
  if (brightness <= min_brightness) {
    brightness = min_brightness;
    brightness_dir = 1;   // Start going up
  }

  // Safety clamp
  brightness = constrain(brightness, min_brightness, max_brightness);

  float min_color = map_auto_min_Color(angleInputs.ch4);
  float max_color = map_auto_max_Color(angleInputs.ch5);
  float color_speed = map_auto_color_Speed(angleInputs.ch6); // Base speed setting

  // Update color position
  color += color_dir * color_speed;

  // Validate color range
  if (min_color > max_color) {
    float temp = min_color;
    min_color = max_color;
    max_color = temp;
  }
  if (max_color < 1.0) max_color = 1.0;
  if (min_color < 0.0) min_color = 0.0;

  // Check boundaries and reverse direction
  if (color >= max_color) {
    color = max_color;
    color_dir = -1;
  }
  if (color <= min_color) {
    color = min_color;
    color_dir = 1;
  }

  // Safety clamp
  color = constrain(color, min_color, max_color);

  data[1] = (uint8_t)brightness;
  data[2] = (uint8_t)color;

  dmx_write(dmxPort, data, DMX_PACKET_SIZE); // write
  dmx_send(dmxPort); // send
  dmx_wait_sent(dmxPort, DMX_TIMEOUT_TICK);

  // === Display Update ===
  static uint32_t lastUpdate = 0;

  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    // Draw static labels ONLY on first draw (use global flag)
    if (auto_mode_firstDraw) {
      M5.Display.fillScreen(BLACK);  // Clear once
      M5.Display.setTextColor(CYAN);
      M5.Display.setCursor(0, 0);
      M5.Display.println("AUTO MODE");
      M5.Display.setTextColor(WHITE);

      // Draw labels that never change
      M5.Display.setCursor(0, 30);
      M5.Display.print("Min Bri:");
      M5.Display.setCursor(0, 50);
      M5.Display.print("Max Bri:");
      M5.Display.setCursor(0, 70);
      M5.Display.print("Bri Spd:");

      M5.Display.setCursor(0, 100);
      M5.Display.print("Min Col:");
      M5.Display.setCursor(0, 120);
      M5.Display.print("Max Col:");
      M5.Display.setCursor(0, 140);
      M5.Display.print("Col Spd:");

      M5.Display.setTextColor(GREEN);
      M5.Display.setCursor(0, 170);
      M5.Display.print("-> Bri:");
      M5.Display.setCursor(0, 190);
      M5.Display.print("-> Col:");
      M5.Display.setTextColor(WHITE);

      auto_mode_firstDraw = false;
    }

    // Update ONLY the numbers (clear small area, redraw)
    M5.Display.fillRect(100, 30, 80, 16, BLACK);  // Clear just the number area
    M5.Display.setCursor(100, 30);
    M5.Display.printf("%d", (int)min_brightness);

    M5.Display.fillRect(100, 50, 80, 16, BLACK);
    M5.Display.setCursor(100, 50);
    M5.Display.printf("%d", (int)max_brightness);

    M5.Display.fillRect(100, 70, 80, 16, BLACK);
    M5.Display.setCursor(100, 70);
    M5.Display.printf("%.2f", base_speed);

    M5.Display.fillRect(100, 100, 80, 16, BLACK);
    M5.Display.setCursor(100, 100);
    M5.Display.printf("%d", (int)min_color);

    M5.Display.fillRect(100, 120, 80, 16, BLACK);
    M5.Display.setCursor(100, 120);
    M5.Display.printf("%d", (int)max_color);

    M5.Display.fillRect(100, 140, 80, 16, BLACK);
    M5.Display.setCursor(100, 140);
    M5.Display.printf("%.2f", color_speed);

    M5.Display.fillRect(100, 170, 80, 16, BLACK);
    M5.Display.setCursor(100, 170);
    M5.Display.setTextColor(GREEN);
    M5.Display.printf("%d", data[1]);

    M5.Display.fillRect(100, 190, 80, 16, BLACK);
    M5.Display.setCursor(100, 190);
    M5.Display.printf("%d", data[2]);
    M5.Display.setTextColor(WHITE);
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



// MOTOR FUNCTIONS
bool connectToMotor(){
  if (motorConnected) return true;

  Serial.println("Connecting to motor...");
  IPAddress motor_ip = MOTOR_IP_ADDRESS;
  uint16_t motor_port = MOTOR_PORT;
  Serial.printf("M5 IP: %s\n", LAN.localIP().toString().c_str());  // ← Add this
  Serial.printf("Motor IP: %s\n", motor_ip.toString().c_str());     // ← Add this
  Serial.printf("Motor Port: %d\n", motor_port);  
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

bool checkMotorConnection(){
  if(motorConnected && modbusClient.connected()){
    return true;
  }
  if (motorConnected){
    Serial.println("⚠️ Motor connection lost! Reconnecting...");
  }
  motorConnected = false;

  bool reconnected = connectToMotor();
  if(reconnected){
    justReconnected = true;
  }
  return reconnected;

  return connectToMotor();
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

void performHoming(){
  uint16_t homingCmd[1] = {1}; // Just 1, not bit-shifted!
  uint16_t clearCmd[1] = {0};

  if (modbusWriteMultipleRegisters(P_PRESET_EXECUTE, 1, homingCmd)) {
    Serial.println("✓ set currect position to 0!");
  }
  delay(100);
  if (modbusWriteMultipleRegisters(P_PRESET_EXECUTE, 1, clearCmd)) {
    Serial.println("✓ set currect position to 0!");
  }
}

bool checkAlarmsStatus()
{
  uint16_t alarmStatus[1];
  if (modbusReadHoldingRegisters(ADDR_ALARM_MON, 1, alarmStatus)) {
    if (alarmStatus[0] !=0 ) {
      Serial.printf("✗ Alarm status: 0x%04X", alarmStatus[0]);
      return true;
    }
  }
  return false;
}

void clearAlarms(){
  uint16_t clearCmd[1] = {1 << 7};
  uint16_t resetCmd[1] = {0};

  if (modbusWriteMultipleRegisters(ADDR_STATIC_IO_IN, 1, clearCmd)) {
    Serial.println("✓ Alarms cleared");
    delay(100);
    modbusWriteMultipleRegisters(ADDR_STATIC_IO_IN, 1, resetCmd);
  }
}
// MOTOR FUNCTIONS




void loop() {
  M5.update();
  read8AngleInputs();
  updateDMX();


  if(checkMotorConnection()){
    if(justReconnected)
    {
      if(checkAlarmsStatus()){
        clearAlarms();
        performHoming();
      }

    startContinuousSpeedControl(oriental_motorSpeed, oriental_motorStartAcc, oriental_motorStopAcc);

    justReconnected = false;
    }
    readMotorPosition();
    }
  
  yield(); // let system handble background task
  delay(1000/60);
  
}
