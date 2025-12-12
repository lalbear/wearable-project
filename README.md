# IoT Health Monitoring System

This project is a simple **IoT Health Monitoring System** for prototyping step detection and heart-rate estimation using an ADXL345 accelerometer and a basic analog heart-rate sensor. The firmware is written in C++ for Arduino/ESP32 platforms.

---

## Features

- Interrupt-driven step detection using the accelerometer interrupt pin.
- Fallback step detection using acceleration magnitude thresholding.
- Estimated pulse rate from an analog heart-rate sensor (linear mapping).
- Compact, low-latency loop suitable for embedded devices.

---

## Hardware

**Recommended components**
- ESP32 (recommended) or Arduino Uno/Nano/Leonardo (note: ISR attributes differ)
- ADXL345 accelerometer breakout (I2C)
- Heart-rate sensor module (e.g., analog-output pulse sensor)
- Connecting wires, breadboard, 3.3V or 5V power (depending on board and sensors)

**Pin connections (example for ESP32 / generic)**
- ADXL345:  
  - VIN -> 3.3V (or 5V depending on module)  
  - GND -> GND  
  - SDA -> SDA (ESP32: GPIO21 / Arduino UNO: A4)  
  - SCL -> SCL (ESP32: GPIO22 / Arduino UNO: A5)
- Heart-rate sensor:  
  - Signal -> `A0` (HR_SENSOR_PIN)  
  - VCC -> 3.3V/5V (match sensor requirement)  
  - GND -> GND
- ADXL345 Interrupt pin -> `D2` (or other interrupt-capable pin)  
  - Connect ADXL345 INT1 or INT2 to microcontroller digital pin defined by `ACCELEROMETER_INTERRUPT_PIN`.

> **Important:** For AVR/Uno/Nano, `IRAM_ATTR` is not required and should be removed from the ISR prototype. Use `attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);` as in the code.

---

## Software / Libraries

Install these Arduino libraries via Library Manager (or git):

- `Adafruit ADXL345` (Adafruit_ADXL345_U)
- `Adafruit Sensor` (Adafruit_Sensor)

If using PlatformIO, add to `platformio.ini`:
```ini
lib_deps =
  adafruit/Adafruit ADXL345@^1.0.0
  adafruit/Adafruit Unified Sensor@^1.0.0
