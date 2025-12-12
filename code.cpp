/*******************************************************
  Health Monitoring System
  - ADXL345 accelerometer (Adafruit_ADXL345_U)
  - Heart-rate sensor on analog pin
  - Interrupt-driven step detection (accelerometer interrupt)
  - Fallback step detection via magnitude threshold

  Author: Pranjal Upadhyay (refactored)
  Date: 2025
*******************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>
#include <math.h>

// ----- Configuration -----
#define HR_SENSOR_PIN A0                      // Analog pin for heart-rate sensor
#define ACCELEROMETER_INTERRUPT_PIN 2         // Interrupt pin connected to accel INT (use appropriate pin)
#define ACCEL_RANGE ADXL345_RANGE_2_G         // +/- 2g range

// Tuning parameters
const unsigned long STEP_DEBOUNCE_MS = 500;    // Minimum time between steps (ms)
const float STEP_MAG_THRESHOLD = 1.0;          // Threshold on magnitude change to detect a step
const unsigned long MEASURE_INTERVAL_MS = 1000; // How often to read sensors in loop

// Heart rate mapping/calibration (adjust for your sensor)
const int HR_MAP_IN_MIN = 0;
const int HR_MAP_IN_MAX = 7000;   // depends on sensor output range
const int HR_MAP_OUT_MIN = 60;    // displayable bpm lower bound
const int HR_MAP_OUT_MAX = 150;   // displayable bpm upper bound

// ----- Globals -----
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

volatile bool stepDetected = false;
volatile unsigned long lastStepISRTime = 0;

unsigned long lastStepTime = 0;
volatile unsigned long stepCount = 0;

float prevAccelX = 0.0f;
float prevAccelY = 0.0f;
float prevAccelZ = 0.0f;

unsigned long lastMeasureMillis = 0;

// ----- ISR -----
// If using ESP32 you can keep IRAM_ATTR; for AVR boards remove it.
void IRAM_ATTR handleInterrupt() {
  unsigned long currentTime = millis();
  // Debounce ISR to avoid multiple triggers for same step
  if (currentTime - lastStepISRTime >= STEP_DEBOUNCE_MS) {
    stepDetected = true;
    lastStepISRTime = currentTime;
  }
}

// ----- Helper functions -----
/**
 * Convert raw analog value from HR sensor to estimated BPM using mapping.
 * Note: This is a rough linear mapping. Replace with sensor-specific calibration if available.
 */
float estimatePulseBPM(int rawValue) {
  // constrain first
  rawValue = constrain(rawValue, HR_MAP_IN_MIN, HR_MAP_IN_MAX);
  return map(rawValue, HR_MAP_IN_MIN, HR_MAP_IN_MAX, HR_MAP_OUT_MIN, HR_MAP_OUT_MAX);
}

/**
 * Compute acceleration magnitude from vector components.
 */
float accelMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

/**
 * Read accelerometer and perform magnitude-based step detection (fallback).
 * Returns true if a step is detected by magnitude change.
 */
bool magnitudeStepDetect() {
  sensors_event_t event;
  accel.getEvent(&event);

  float curX = event.acceleration.x;
  float curY = event.acceleration.y;
  float curZ = event.acceleration.z;

  float curMag = accelMagnitude(curX, curY, curZ);
  float prevMag = accelMagnitude(prevAccelX, prevAccelY, prevAccelZ);

  // Save current for next call
  prevAccelX = curX;
  prevAccelY = curY;
  prevAccelZ = curZ;

  if (fabs(curMag - prevMag) > STEP_MAG_THRESHOLD) {
    unsigned long now = millis();
    if (now - lastStepTime >= STEP_DEBOUNCE_MS) {
      lastStepTime = now;
      return true;
    }
  }
  return false;
}

/**
 * Read sensors and print telemetry to Serial.
 */
void measureData() {
  // --- Heart rate ---
  int rawHR = analogRead(HR_SENSOR_PIN);
  float pulseRate = estimatePulseBPM(rawHR);

  // If interrupt flagged a step, increment and reset flag
  if (stepDetected) {
    stepCount++;
    stepDetected = false;
    Serial.print("[ISR] Step detected. Total steps: ");
    Serial.println(stepCount);
  }

  // Fallback magnitude-based detection
  if (magnitudeStepDetect()) {
    stepCount++;
    Serial.print("[MAG] Step detected. Total steps: ");
    Serial.println(stepCount);
  }

  // Print readings
  Serial.print("Pulse (est): ");
  Serial.print(pulseRate, 1);
  Serial.print(" BPM\tRaw HR: ");
  Serial.print(rawHR);
  Serial.print("\tSteps: ");
  Serial.println(stepCount);

  // (Optional) Add code to push data to display/SD/telemetry here
}

// ----- Setup & Loop -----
void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize I2C & accelerometer
  Wire.begin();
  if (!accel.begin()) {
    Serial.println("ERROR: ADXL345 not found. Check wiring.");
    while (1) {
      delay(1000);
      Serial.println("Waiting for ADXL345...");
    }
  }
  accel.setRange(ACCEL_RANGE);
  Serial.println("ADXL345 initialized.");

  // Initialize previous accelerometer values (one read)
  sensors_event_t initEvent;
  accel.getEvent(&initEvent);
  prevAccelX = initEvent.acceleration.x;
  prevAccelY = initEvent.acceleration.y;
  prevAccelZ = initEvent.acceleration.z;

  // Configure interrupt pin (pullup recommended for many boards)
  pinMode(ACCELEROMETER_INTERRUPT_PIN, INPUT_PULLUP);

  // Attach interrupt: RISING edge typically used for ADXL345 INT1/INT2
  attachInterrupt(digitalPinToInterrupt(ACCELEROMETER_INTERRUPT_PIN), handleInterrupt, RISING);

  Serial.println("Setup complete.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastMeasureMillis >= MEASURE_INTERVAL_MS) {
    lastMeasureMillis = now;
    measureData();
  }

  // Allow the MCU to do other tasks; keep loop responsive
  delay(5);
}
