/*
  Servo Torque Tester - Arduino Firmware
  This code runs on an Arduino Nano ESP32.
*/

// --- INCLUDES ---
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <HX711_ADC.h>
#include <ArduinoJson.h>
#include <SparkFunLSM6DSO.h>

// --- PIN DEFINITIONS ---
#define PIN_SERVO 3         // PWM pin for the servo
#define PIN_HX711_DOUT 5    // Data pin for HX711
#define PIN_HX711_SCK 6     // Clock pin for HX711

// --- GLOBAL OBJECTS ---
Servo myServo;
HX711_ADC scale(PIN_HX711_DOUT, PIN_HX711_SCK);
LSM6DSO myIMU;

// --- GLOBAL VARIABLES ---
unsigned long lastSendTime = 0;
const int SENSOR_SEND_INTERVAL_MS = 50; 
int targetServoPos = 165; 
bool isServoActive = false; // Tracks if servo is attached

// --- GYRO SMOOTHING ---
float smoothedGyroZ = 0;
const float GYRO_ALPHA = 0.2; 

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Wire.begin();
  if (!myIMU.begin()) { 
    Serial.println("{\"type\":\"status\", \"msg\":\"Failed to communicate with LSM6DSO\"}");
  }
  if (!myIMU.initialize(BASIC_SETTINGS)) {
      Serial.println("{\"type\":\"status\", \"msg\":\"Failed to init IMU settings\"}");
  }

  // --- Setup Servo ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50); 
  
  // 1. Attach for Startup Sequence
  myServo.attach(PIN_SERVO, 500, 2400);
  
  // 2. Wiggle Test
  Serial.println("{\"type\":\"status\", \"msg\":\"Arduino: Wiggle Test\"}");
  myServo.write(80);
  delay(500);
  myServo.write(100);
  delay(500);
  
  // 3. Go to Resting Position
  myServo.write(165);
  targetServoPos = 165;
  delay(500); // Wait for move to finish
  
  // 4. Detach (OFF by default)
  myServo.detach();
  isServoActive = false;
  
  Serial.println("{\"type\":\"status\", \"msg\":\"Arduino Ready: Motor Detached\"}");

  // --- Setup Load Cell ---
  scale.begin();
  scale.start(1000, true); 
  scale.setCalFactor(1.0); 
}

void loop() {
  handleSerialCommands();
  sendSensorData();
}

void handleSerialCommands() {
  static String inputBuffer = "";

  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      if (inputBuffer.length() > 0) {
        Serial.print("DEBUG: RX: ");
        Serial.println(inputBuffer);

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, inputBuffer);

        if (!error) {
          const char* cmd = doc["cmd"];

          // --- COMMAND: ACTIVE ---
          if (strcmp(cmd, "active") == 0) {
            int val = doc["value"];
            if (val == 1) {
              if (!isServoActive) {
                myServo.attach(PIN_SERVO, 500, 2400);
                myServo.write(targetServoPos); // Snap to last known target
                isServoActive = true;
                Serial.println("{\"type\":\"status\", \"msg\":\"Servo Attached\"}");
              }
            } else {
              if (isServoActive) {
                myServo.detach();
                isServoActive = false;
                Serial.println("{\"type\":\"status\", \"msg\":\"Servo Detached\"}");
              }
            }
          }
          // --- COMMAND: MOVE ---
          else if (strcmp(cmd, "move") == 0) {
            int pos = doc["value"];
            if (pos < 0) pos = 0;
            if (pos > 180) pos = 180;
            
            // Always update target, so if we attach later, we know where to go
            targetServoPos = pos;

            // Only physically write if active
            if (isServoActive) {
              myServo.write(pos);
            }
          }
        }
      }
      inputBuffer = "";
    } 
    else if (c != '\r') { 
      inputBuffer += c;
    }
  }
}

void sendSensorData() {
  if (millis() - lastSendTime > SENSOR_SEND_INTERVAL_MS) {
    lastSendTime = millis();

    if (scale.update()) {
      long raw_load = scale.getData();

      float raw_gz_dps = myIMU.readFloatGyroZ(); 
      smoothedGyroZ = (GYRO_ALPHA * raw_gz_dps) + ((1.0 - GYRO_ALPHA) * smoothedGyroZ);
      float gyro_z_rads = smoothedGyroZ * (PI / 180.0); 

      JsonDocument doc;
      doc["type"] = "data";
      doc["load_raw"] = raw_load;
      doc["gyro_z"] = gyro_z_rads;
      doc["servo_pos"] = targetServoPos; // Send target even if detached

      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}