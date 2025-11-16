# Servo Motor Torque Tester

This project contains all the software required to build and operate a servo motor torque testing stand. It includes the Arduino firmware for the ESP32 and a Python-based computer GUI for control, visualization, and data logging.

The system is designed to measure two key metrics:

- **Stall Torque**: The maximum torque a servo can produce from a standstill
- **Rated Torque**: The torque a servo can produce while in motion, calculated using angular acceleration

## Hardware Components

- Arduino Nano ESP32
- Servo Motor
- Load Cell & HX711 Amplifier
- LSM6DSO 6DoF IMU (I2C)

## Features

- **Python GUI**: A full graphical user interface to control the tester
- **Live Graphing**: Real-time plots of load cell readings and calculated torque
- **GUI-Based Calibration**: Calibrate the load cell (Tare and Scale) directly from the GUI without re-flashing the Arduino
- **Serial Control**: All servo movements and test initiations are controlled from the computer
- **Automated Tests**: Buttons to run pre-defined test sequences for stall and rated torque
- **Variable-Based Setup**: All physical dimensions (lever arm, inertia) are variables in the GUI for easy adjustment

## Setup & Installation

### 1. Arduino (ESP32)

**Libraries to Install:**
You must install the following libraries through the Arduino IDE Library Manager:

- HX711-ADC by Olav M. (or another popular HX711 library)
- Arduino_LSM6DSOX (by Arduino)
- ESP32Servo (for ESP32-specific servo control)
- ArduinoJson (version 6.x or 7.x)

**Firmware:**

1. Open `ServoTorqueTester.ino` in the Arduino IDE
2. At the top of the file, set your pin variables (`PIN_SERVO`, `PIN_HX711_DOUT`, `PIN_HX711_SCK`)
3. Select your board (Arduino Nano ESP32) and COM port
4. Upload the sketch

### 2. Python GUI

**Dependencies:**
You will need Python 3 installed. You can install the required libraries using pip:

```bash
pip install PyQt6 pyqtgraph pyserial
```

**Running the GUI:**

1. Save the `torque_tester_gui.py` file
2. Run it from your terminal:

```bash
python torque_tester_gui.py
```

## How to Use the System

1. **Connect Hardware**: Plug your Arduino into the computer via USB

2. **Run GUI**: Start the `torque_tester_gui.py` application

3. **Connect to Arduino**:
   - Select the correct COM port from the "COM Port" dropdown
   - Click "Connect". The status label should turn green and say "Connected"
   - The "Live Load Cell" graph will start showing raw data

4. **Calibrate Load Cell**:
   - **Tare**: Ensure there is no weight on the load cell. Click the "Tare" button. The system will get an offset value, and the load graph should now read near 0
   - **Calibrate**:
     - Place a known weight (e.g., 100g) on the load cell
     - Enter this weight into the "Known Weight (g)" box
     - Click the "Calibrate" button
     - The system will calculate a scale factor, and the "Live Load Cell" graph should now read the correct weight in grams
   - This calibration is saved in the GUI. You are now ready to test

5. **Enter Physical Parameters**:
   - Fill in the Lever Arm (m), Inertia (Arm) (kg·m²), and Inertia (Weight) (kg·m²) fields. These are crucial for the torque calculations

6. **Run Tests**:
   - **Manual Control**: Use the slider and "Move Servo" button to test the servo's range
   - **Stall Torque Test**:
     - Place your test weight on the load cell. The graph will show its "baseline" weight
     - Click "Run Stall Test"
     - The GUI will command the servo to lift the weight
     - It calculates the torque based on the reduction in force on the load cell. The result will appear
   - **Rated Torque Test**:
     - Ensure the arm and weight are attached
     - Click "Run Rated Test"
     - The GUI will command a fast servo sweep and measure the angular acceleration from the IMU
     - It calculates torque using Torque = I × α (where I is the total inertia you entered). The result will appear
