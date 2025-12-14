#ifndef ANGLE8_CONTROL_H
#define ANGLE8_CONTROL_H

#include "M5_ANGLE8.h"
#include <Arduino.h>
#include <Wire.h>
#include "pins_arduino.h"

struct AngleInputs {
    bool mode_switch = 0;
    int16_t ch1 = 0;
    int16_t ch2 = 0;
    int16_t ch3 = 0;
    int16_t ch4 = 0;
    int16_t ch5 = 0;
    int16_t ch6 = 0;
    int16_t ch7 = 0;
    int16_t ch8 = 0;
};

extern M5_ANGLE8 angle8;
extern bool angle8_found;
extern AngleInputs angleInputs;

bool init8Angle();
void read8AngleInputs();
float mapBrightnesssNonLinear(int16_t angle_value, float min_brightness, float max_brightness);
float mapColorLinear(int16_t angle_value);
void update8AngleLEDs();

inline bool init8Angle() {
    Serial.println("Initializing 8Angle...");

    if(angle8.begin()) {  // Frequency defaults to 100000
      angle8_found = true;
      Serial.println("✓ 8Angle initialized!");
      return true;
    } else {
      angle8_found = false;
      Serial.println("✗ 8Angle initialization FAILED!");
      return false;
    }
  }

inline void read8AngleInputs() {
    if (!angle8_found) {
      return;
    }

    // CORRECTED: Read digital input (returns byte with switch states)
    uint8_t digital_inputs = angle8.getDigitalInput();

    // Extract switch 0 state (bit 0) - 0 = manual, 1 = auto
    angleInputs.mode_switch = digital_inputs & 0x01;

    // CORRECTED: getAnalogInput only takes one parameter (channel 0-7)
    angleInputs.ch1 = angle8.getAnalogInput(0);  // Brightness
    angleInputs.ch2 = angle8.getAnalogInput(1);  // Color
    angleInputs.ch3 = angle8.getAnalogInput(2);  // Motor position (for later)
    angleInputs.ch4 = angle8.getAnalogInput(3);  // Position scale (for later)
    angleInputs.ch5 = angle8.getAnalogInput(4);  // Motor direction (for later)
    angleInputs.ch6 = angle8.getAnalogInput(5);  // Color speed (for later)
    angleInputs.ch7 = angle8.getAnalogInput(6);  // Motor speed (for later)
    angleInputs.ch8 = angle8.getAnalogInput(7);  // Reserved (for later)

    // Debug print
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.printf("Mode: %d, CH1: %d, CH2: %d\n",
                    angleInputs.mode_switch, angleInputs.ch1, angleInputs.ch2);
    }
  }
  

inline float mapBrightnessNonLinear(int16_t angle_value, float min_brightness, float max_brightness) {
    float normalized = angle_value / 4095.0;

    if (normalized < 0.0) normalized = 0.0;
    if (normalized > 1.0) normalized = 1.0;

    float curved = pow(normalized, 2.0);
    float brightness = min_brightness + (max_brightness - min_brightness) * curved;

    return brightness;
}

inline float mapColorLinear(int16_t angle_value) {
    float color = (angle_value / 4095.0) * 255.0;

    if (color < 0) color = 0;
    if (color > 255) color = 255;

    return color;
}


#endif