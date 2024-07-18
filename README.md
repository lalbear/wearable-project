#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>

#define HR_SENSOR_PIN A0  // Pin connected to the heart rate sensor
#define ACCELEROMETER_INTERRUPT_PIN 2 // Interrupt pin connected to the accelerometer

Adafruit_ADXL345_Unified accel;  // Declare an instance of the Adafruit_ADXL345_Unified library

volatile bool stepDetected = false;
volatile int stepCount = 0;

unsigned long lastStepTime = 0;
unsigned long stepInterval = 500; // Minimum time between steps in milliseconds

float prevAccelX = 0.0;
float prevAccelY = 0.0;
float prevAccelZ = 0.0;

void IRAM_ATTR handleInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastStepTime >= stepInterval) {
    stepDetected = true;
    lastStepTime = currentTime;
  }
}

void measureData() {
  int sensorValue = analogRead(HR_SENSOR_PIN);
  float pulseRate = map(sensorValue, 0, 7000, 60, 150);  // Adjust the conversion based on your heart rate sensor characteristics
  
  if (stepDetected) {
    stepCount++;
    stepDetected = false;
    Serial.print("Step count: ");
    Serial.println(stepCount);
  }
  
  Serial.print("Pulse rate: ");
  Serial.println(pulseRate);

  sensors_event_t event;
  accel.getEvent(&event);

  float currentAccelX = event.acceleration.x;
  float currentAccelY = event.acceleration.y;
  float currentAccelZ = event.acceleration.z;

  // Calculate the magnitude of the acceleration vector
  float accelerationMagnitude = sqrt(pow(currentAccelX, 2) + pow(currentAccelY, 2) + pow(currentAccelZ, 2));

  // Check for significant change in acceleration to detect a step
  if (abs(accelerationMagnitude - sqrt(pow(prevAccelX, 2) + pow(prevAccelY, 2) + pow(prevAccelZ, 2))) > 1.0) {
    stepCount++;
    Serial.print("Step count: ");
    Serial.println(stepCount);
  }

  prevAccelX = currentAccelX;
  prevAccelY = currentAccelY;
  prevAccelZ = currentAccelZ;
}

void setup() {
  Serial.begin(115200);

  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }

  Wire.begin();
  accel.setRange(ADXL345_RANGE_2_G);  // Set the accelerometer range to +/- 2g

  pinMode(ACCELEROMETER_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACCELEROMETER_INTERRUPT_PIN), handleInterrupt, RISING);
}

void loop() {
  measureData();
  delay(1000);
}
