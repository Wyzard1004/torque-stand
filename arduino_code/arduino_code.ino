/*
  Servo Torque Tester - Arduino Firmware
  This code runs on an Arduino Nano ESP32.

  -- UPDATED to use the 'Arduino_LSM6DSOX' library --

  It performs three main tasks:
  1. Reads the Load Cell (HX711)
  2. Reads the IMU (LSM6DSOX) for angular velocity
  3. Controls the Servo Motor (ESP32Servo)

  It communicates with a computer GUI via Serial, sending JSON data packets
  and receiving JSON commands.
*/

// --- INCLUDES ---
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <HX711_ADC.h>
#include <ArduinoJson.h>
#include <Arduino_LSM6DSOX.h> // <-- UPDATED LIBRARY

// --- PIN DEFINITIONS (USER MUST SET) ---
#define PIN_SERVO 12        // PWM pin for the servo
#define PIN_HX711_DOUT 4    // Data pin for HX711
#define PIN_HX711_SCK 5     // Clock pin for HX711
// I2C (SDA, SCL) for IMU are usually default pins, no define needed

// --- GLOBAL OBJECTS ---
Servo myServo;
HX711_ADC scale(PIN_HX711_DOUT, PIN_HX711_SCK);
// The LSM6DSOX library uses a global singleton object 'LSM6DSOX'
// No need to declare: Adafruit_LSM6DSO lsm;

// --- GLOBAL VARIABLES ---
long loadCellOffset = 0;
float loadCellScale = 1.0;
unsigned long lastSendTime = 0;
const int SENSOR_SEND_INTERVAL_MS = 50; // Send data every 50ms (20Hz)

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Start I2C for IMU
  Wire.begin();
  // Use the global LSM6DSOX object
  if (!IMU_LSM6DSOX.begin()) { // <-- UPDATED CHECK (using IMU_LSM6DSOX)
    Serial.println("{\"type\":\"status\", \"msg\":\"Failed to find LSM6DSOX chip\"}");
    while (1);
  }
  // The line below is removed, as .begin() automatically sets the range to 2000 dps.
  // IMU_LSM6DSOX.setGyroscopeRange(2000); // <-- THIS LINE IS DELETED

  // --- Setup Servo ---
  myServo.attach(PIN_SERVO);
  myServo.write(0); // Start at 0 position

  // --- Setup Load Cell (HX711) ---
  scale.begin();
  scale.start(2000, true); // Start conversion, wait for 2s, tare
  scale.setCalFactor(loadCellScale); // Set a default scale
  Serial.println("{\"type\":\"status\", \"msg\":\"Arduino Ready\"}");
}

// Main loop: handle serial and send data
void loop() {
  handleSerialCommands();
  sendSensorData();
}

// Check for and process incoming commands from the GUI
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);

    if (error) {
      Serial.print("{\"type\":\"error\", \"msg\":\"JSON deserialize failed: ");
      Serial.print(error.c_str());
      Serial.println("\"}");
      return;
    }

    // Process the command
    const char* cmd = doc["cmd"];

    if (strcmp(cmd, "move") == 0) {
      int pos = doc["value"];
      myServo.write(pos);
    } 
    else if (strcmp(cmd, "tare") == 0) {
      // Manually read 10 samples for a stable average
      long tare_val = 0;
      for (int i = 0; i < 10; i++) {
        while(!scale.update()); // Wait for new data to be ready
        tare_val += scale.getData();
      }
      tare_val /= 10; // Get the average

      // Send this raw value back to the GUI
      JsonDocument response;
      response["type"] = "tare_result";
      response["offset"] = tare_val;
      serializeJson(response, Serial);
      Serial.println();
    }
    else if (strcmp(cmd, "calibrate") == 0) {
      // Manually read 10 samples for stable calibration
      long cal_val = 0;
      for (int i = 0; i < 10; i++) {
        while(!scale.update()); // Wait for new data to be ready
        cal_val += scale.getData();
      }
      cal_val /= 10; // Get the average
      
      // Send this raw value back to the GUI
      JsonDocument response;
      response["type"] = "calibration_result";
      response["raw_val"] = cal_val;
      serializeJson(response, Serial);
      Serial.println();
    }
    else if (strcmp(cmd, "set_offset") == 0) {
      loadCellOffset = doc["value"].as<long>();
      scale.setTareOffset(loadCellOffset);
    }
    else if (strcmp(cmd, "set_scale") == 0) {
      loadCellScale = doc["value"].as<float>();
      scale.setCalFactor(loadCellScale);
    }
  }
}

// Send sensor data to the GUI on a regular interval
void sendSensorData() {
  if (millis() - lastSendTime > SENSOR_SEND_INTERVAL_MS) {
    lastSendTime = millis();

    // 1. Read Load Cell
    // We must check if data is ready first
    if (scale.update()) {
      long raw_load = scale.getData();

      // 2. Read IMU
      float gx, gy, gz; // Variables to hold gyro data
      // Read gyro data (in degrees per second)
      IMU_LSM6DSOX.readGyroscope(gx, gy, gz); // <-- UPDATED API CALL (was readGyro)

      // --- CONVERT TO RAD/S for the Python GUI ---
      // The GUI calculation expects radians/second
      float gyro_z_rads = gz * (PI / 180.0); // <-- NEW CONVERSION

      // 3. Read Servo Position
      int servo_pos = myServo.read();

      // 4. Create JSON packet
      JsonDocument doc;
      doc["type"] = "data";
      doc["load_raw"] = raw_load;
      doc["gyro_z"] = gyro_z_rads; // <-- Send converted value
      doc["servo_pos"] = servo_pos;

      // 5. Send JSON packet
      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}