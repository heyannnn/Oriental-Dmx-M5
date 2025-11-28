# Shiseido Motor Control System

M5Stack CoreS3 controller for Oriental Motor stepping motors, Mitsubishi motors, and DMX512 lighting.

## Hardware

- **M5Stack CoreS3**
- **M5Stack DMX512 Module** (LAN Module 13.2)
- **Oriental Motor**: AZM46MK-TS3.6 stepping motor
- **Mitsubishi Motors**: 2x stepping motors (clockwise + counter-clockwise)
- **DMX Light**: Nanlite FC-60B/120B bi-color LED spotlight

## Current Status

### ✅ DMX512 Control - WORKING
- Controls Nanlite FC-60B/120B light
- CCT 8-bit mode (2 channels)
- Channel mapping:
  - data[0] = unused
  - data[1] = Brightness/Dimmer (0-128)
  - data[2] = Color Temperature (0-255: 2700K-6500K)
- Fading animation implemented
- M5 display shows current values

### 🔧 Motor Control - TODO
- Oriental Motor (AZM46MK-TS3.6) - Pending
- Mitsubishi Motors (2x) - Pending

## Pin Configuration

### DMX512 Module (M5Stack CoreS3)
- TX: GPIO 7
- RX: GPIO 10
- EN: GPIO 6

## Libraries Used

- `M5Unified` - M5Stack hardware
- `M5GFX` - Display graphics
- `esp_dmx` v4.1.0 - DMX512 control

## Notes

- DMX address on light: 1
- Display updates at 100ms intervals to avoid flicker
- DMX sends at 30ms intervals (~33Hz)
