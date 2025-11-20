# Servo Motor Torque Tester

This project contains all the software required to build and operate a servo motor torque testing stand. It includes the Arduino firmware for the ESP32 and a Python-based computer GUI for control, visualization, and data logging.

The system uses a "dumb firmware, smart GUI" architecture: the Arduino streams raw sensor data at high speed, and the Python application handles all calibration, filtering, and torque calculations.

## Features

### Real-Time Control:

- **Safety Toggle**: Motor is OFF by default. Must be explicitly activated via the GUI.
- **Live Slider**: Servo moves immediately as you drag the slider (throttled for stability).

### Advanced Visualization:

- **4-Panel Live Graphing**: Load (g), Torque (Nm), Raw ADC, and Angular Acceleration.
- **Auto-Scaling Limits**: Graphs prevent zooming in on noise (e.g., <50g or <0.2Nm).
- **Serial Console**: Built-in debugging window to view raw JSON data streams.

### Calibration:

- **GUI-Based Tare & Scale**: Calibrate the load cell directly from the interface without re-flashing firmware.
- **Automated Testing**: Buttons to run pre-defined test sequences for stall and rated torque.

## Hardware Components & Wiring

- **Arduino Nano ESP32**
- **Servo Motor**: Signal Pin → D3 (Requires external 5V power!)
- **Load Cell & HX711**:
  - DOUT → D5
  - SCK → D6
- **LSM6DSO 6DoF IMU**: Connected via I2C (SDA/SCL).

## Setup & Installation

### 1. Arduino (ESP32)

**Libraries to Install:**
You must install the following libraries through the Arduino IDE Library Manager:

- HX711-ADC by Olav M.
- SparkFun Qwiic 6DoF LSM6DSO (Search for "SparkFun LSM6DSO")
- ESP32Servo
- ArduinoJson (version 7.x recommended)

**Firmware:**

1. Open `ServoTorqueTester.ino` in the Arduino IDE.
2. Verify pin definitions at the top match your wiring.
3. Select your board (Arduino Nano ESP32) and COM port.
4. **Tools > USB CDC On Boot**: Ensure this is set to **Enabled**.
5. Upload the sketch.

**Note:** On startup, the servo will perform a "wiggle test" (80°→100°) and then detach to resting position (165°).

### 2. Python GUI

**Dependencies:**
You will need Python 3 installed. Install required libraries:

```bash
pip install PyQt6 pyqtgraph pyserial
```

**Running the GUI:**

```bash
python torque_tester_gui.py
```

## How to Use the System

### 1. Connection & Safety

1. Plug in the Arduino. **Close the Arduino IDE Serial Monitor** (port conflict will occur otherwise).
2. In the GUI, select the COM port and click **Connect**.
3. Open **Show Serial Console** if you want to verify raw data is flowing.
4. **Activate Motor**: The motor starts in an "OFF" (detached) state. Click the red "Motor Status: OFF" button to attach power. It will turn green.

### 2. Calibration (Load Cell)

- **Tare**: Remove all test weights (leave the lever arm attached). Click **Tare**. Wait for the "Offset" value to update.
- **Calibrate**:
  1. Place a known weight (e.g., 100g) on the load cell/lever.
  2. Enter `100` in the "Known Weight" box.
  3. Click **Calibrate**.
  4. The "Scale" value will update, and the Live Load graph should read ~100g.

### 3. Running Tests

- **Manual**: Drag the slider to move the servo.
- **Stall Torque**:
  1. Ensure a weight is attached.
  2. Click **"Run Stall Torque Test"**.
  3. The system moves to 90°, measures the lift force, and calculates torque based on the reduction in load cell force.
- **Rated Torque**:
  1. Click **"Run Rated Torque Test"**.
  2. The system performs a high-speed sweep (0° → 120°).
  3. It measures peak Angular Acceleration via the IMU and calculates torque using $T = I \cdot \alpha$.

## Troubleshooting

- **Servo not moving?** Ensure the "Motor Status" button is Green (ON). Check external power supply.
- **No Serial Data?** Check if Arduino IDE is open (close it). Check USB cable.
- **Graph is noisy?** The system applies a low-pass filter (Alpha 0.2) on the Arduino side, and graph limits prevent zooming in on static noise.