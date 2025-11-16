import sys
import json
import time
from collections import deque
import serial
import serial.tools.list_ports
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QGroupBox, QSlider,
    QGridLayout, QDoubleSpinBox, QMessageBox
)
from PyQt6.QtCore import (
    QThread, QObject, pyqtSignal, Qt, QTimer, QSize
)
import pyqtgraph as pg

# --- Constants ---
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 1
DATA_BUFFER_SIZE = 200  # Number of data points for graphs
GRAPH_UPDATE_MS = 50    # Update graphs every 50ms (20Hz)
TEST_MOVE_MS = 1000     # Duration for automated test moves

# --- Serial Communication Worker ---
class SerialWorker(QObject):
    """
    Runs in a separate thread to handle serial communication
    non-blockingly.
    """
    data_received = pyqtSignal(dict)
    status_received = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.serial_port = None
        self.running = True

    def run(self):
        try:
            self.serial_port = serial.Serial(
                self.port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT
            )
            self.status_received.emit("Connected")
        except serial.SerialException as e:
            self.status_received.emit(f"Error: {e}")
            return

        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            print(f"Bad JSON: {line}")
            except (serial.SerialException, OSError) as e:
                self.status_received.emit(f"Serial Error: {e}")
                self.connection_lost.emit()
                break
            except Exception as e:
                print(f"Unhandled worker error: {e}")
        
        if self.serial_port:
            self.serial_port.close()

    def write_command(self, cmd_dict):
        if self.serial_port and self.serial_port.is_open:
            cmd_str = json.dumps(cmd_dict) + '\n'
            try:
                self.serial_port.write(cmd_str.encode('utf-8'))
            except serial.SerialException as e:
                self.status_received.emit(f"Write Error: {e}")
                self.connection_lost.emit()

    def stop(self):
        self.running = False
        if self.serial_port:
            self.serial_port.close()

# --- Main Application Window ---
class MainWindow(QMainWindow):
    # Signal to send command to worker thread
    send_command_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Servo Torque Tester GUI")
        self.setGeometry(100, 100, 1200, 800)

        # Serial communication
        self.serial_thread = None
        self.serial_worker = None
        
        # State variables
        self.load_cell_offset = 0
        self.load_cell_scale = 1.0
        self.is_connected = False
        self.test_in_progress = False

        # Data buffers for graphing
        self.time_data = deque(maxlen=DATA_BUFFER_SIZE)
        self.load_data_g = deque(maxlen=DATA_BUFFER_SIZE)
        self.torque_data_nm = deque(maxlen=DATA_BUFFER_SIZE)
        self.load_data_raw = deque(maxlen=DATA_BUFFER_SIZE) # <-- ADD THIS LINE
        
        # Buffers for test calculations
        self.test_time_buffer = []
        self.test_gyro_buffer = []
        self.test_load_buffer = []
        self.test_start_time = 0

        self.start_time = time.time()

        # Initialize UI
        self.init_ui()
        self.populate_com_ports()

        # Graph update timer
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.start(GRAPH_UPDATE_MS)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # --- Left Column (Controls) ---
        control_layout = QVBoxLayout()
        control_layout.setSpacing(15)

        # Connection Box
        conn_box = QGroupBox("1. Connection")
        conn_layout = QGridLayout()
        conn_layout.addWidget(QLabel("COM Port:"), 0, 0)
        self.com_combo = QComboBox()
        conn_layout.addWidget(self.com_combo, 0, 1)
        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.populate_com_ports)
        conn_layout.addWidget(self.refresh_ports_btn, 0, 2)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setCheckable(True)
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn, 1, 1)
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: red;")
        conn_layout.addWidget(self.status_label, 1, 0, 1, 1)
        conn_box.setLayout(conn_layout)
        control_layout.addWidget(conn_box)

        # Calibration Box
        cal_box = QGroupBox("2. Load Cell Calibration")
        cal_layout = QGridLayout()
        self.tare_btn = QPushButton("Tare (Remove Weight)")
        self.tare_btn.clicked.connect(self.send_tare_command)
        cal_layout.addWidget(self.tare_btn, 0, 0, 1, 2)
        cal_layout.addWidget(QLabel("Known Weight (g):"), 1, 0)
        self.known_weight_input = QLineEdit("100.0")
        cal_layout.addWidget(self.known_weight_input, 1, 1)
        self.calibrate_btn = QPushButton("Calibrate (Place Weight)")
        self.calibrate_btn.clicked.connect(self.send_calibrate_command)
        cal_layout.addWidget(self.calibrate_btn, 2, 0, 1, 2)
        self.offset_label = QLabel("Offset: N/A")
        cal_layout.addWidget(self.offset_label, 3, 0)
        self.scale_label = QLabel("Scale: N/A")
        cal_layout.addWidget(self.scale_label, 3, 1)
        cal_box.setLayout(cal_layout)
        control_layout.addWidget(cal_box)

        # Parameters Box
        param_box = QGroupBox("3. Physical Parameters")
        param_layout = QGridLayout()
        param_layout.addWidget(QLabel("Lever Arm (m):"), 0, 0)
        self.lever_arm_input = QDoubleSpinBox()
        self.lever_arm_input.setDecimals(4)
        self.lever_arm_input.setSingleStep(0.001)
        self.lever_arm_input.setValue(0.05)
        param_layout.addWidget(self.lever_arm_input, 0, 1)
        
        param_layout.addWidget(QLabel("Inertia (Arm) (kg*m^2):"), 1, 0)
        self.inertia_arm_input = QDoubleSpinBox()
        self.inertia_arm_input.setDecimals(6)
        self.inertia_arm_input.setSingleStep(0.00001)
        param_layout.addWidget(self.inertia_arm_input, 1, 1)
        
        param_layout.addWidget(QLabel("Inertia (Weight) (kg*m^2):"), 2, 0)
        self.inertia_weight_input = QDoubleSpinBox()
        self.inertia_weight_input.setDecimals(6)
        self.inertia_weight_input.setSingleStep(0.00001)
        param_layout.addWidget(self.inertia_weight_input, 2, 1)
        param_box.setLayout(param_layout)
        control_layout.addWidget(param_box)

        # Manual Control Box
        manual_box = QGroupBox("4. Manual Servo Control")
        manual_layout = QGridLayout()
        self.servo_slider = QSlider(Qt.Orientation.Horizontal)
        self.servo_slider.setRange(0, 180)
        self.servo_slider.setValue(0)
        self.servo_slider.valueChanged.connect(self.update_slider_label)
        manual_layout.addWidget(self.servo_slider, 0, 0, 1, 2)
        self.slider_label = QLabel("Angle: 0")
        manual_layout.addWidget(self.slider_label, 0, 2)
        self.move_servo_btn = QPushButton("Move Servo")
        self.move_servo_btn.clicked.connect(self.send_move_command)
        manual_layout.addWidget(self.move_servo_btn, 1, 0, 1, 3)
        manual_box.setLayout(manual_layout)
        control_layout.addWidget(manual_box)

        # Automated Test Box
        test_box = QGroupBox("5. Automated Testing")
        test_layout = QVBoxLayout()
        self.stall_test_btn = QPushButton("Run Stall Torque Test")
        self.stall_test_btn.clicked.connect(self.run_stall_test)
        test_layout.addWidget(self.stall_test_btn)
        self.rated_test_btn = QPushButton("Run Rated Torque Test")
        self.rated_test_btn.clicked.connect(self.run_rated_test)
        test_layout.addWidget(self.rated_test_btn)
        self.stall_result_label = QLabel("Stall Torque: --- Nm")
        self.stall_result_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        test_layout.addWidget(self.stall_result_label)
        self.rated_result_label = QLabel("Rated Torque: --- Nm")
        self.rated_result_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        test_layout.addWidget(self.rated_result_label)
        test_box.setLayout(test_layout)
        control_layout.addWidget(test_box)

        control_layout.addStretch()
        main_layout.addLayout(control_layout, 1)

        # --- Right Column (Graphs) ---
        graph_layout = QVBoxLayout()
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.load_plot = pg.PlotWidget(title="Live Load Cell (grams)")
        self.load_plot.setLabel('left', 'Load (g)')
        self.load_plot.setLabel('bottom', 'Time (s)')
        self.load_plot.showGrid(x=True, y=True)
        self.load_curve = self.load_plot.plot(pen=pg.mkPen('g', width=2))
        graph_layout.addWidget(self.load_plot, 1)

        # <-- ADD THIS NEW PLOT WIDGET -->
        self.raw_load_plot = pg.PlotWidget(title="Raw Load Cell (ADC Value)")
        self.raw_load_plot.setLabel('left', 'Raw ADC Value')
        self.raw_load_plot.setLabel('bottom', 'Time (s)')
        self.raw_load_plot.showGrid(x=True, y=True)
        self.raw_load_curve = self.raw_load_plot.plot(pen=pg.mkPen('b', width=2))
        graph_layout.addWidget(self.raw_load_plot, 1)
        # <-- END OF ADDED BLOCK -->

        self.torque_plot = pg.PlotWidget(title="Live Torque (Nm)")
        self.torque_plot.setLabel('left', 'Torque (Nm)')
        self.torque_plot.setLabel('bottom', 'Time (s)')
        self.torque_plot.showGrid(x=True, y=True)
        self.torque_curve = self.torque_plot.plot(pen=pg.mkPen('r', width=2))
        graph_layout.addWidget(self.torque_plot, 1)

        main_layout.addLayout(graph_layout, 2)

    def populate_com_ports(self):
        self.com_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.com_combo.addItem(port.device)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.com_combo.currentText()
            if not port:
                self.show_error("No COM port selected.")
                self.connect_btn.setChecked(False)
                return

            self.serial_thread = QThread()
            self.serial_worker = SerialWorker(port)
            self.serial_worker.moveToThread(self.serial_thread)

            # Connect signals
            self.serial_worker.data_received.connect(self.handle_data)
            self.serial_worker.status_received.connect(self.update_status)
            self.serial_worker.connection_lost.connect(self.handle_connection_lost)
            self.send_command_signal.connect(self.serial_worker.write_command)
            self.serial_thread.started.connect(self.serial_worker.run)

            self.serial_thread.start()
            self.connect_btn.setText("Disconnect")
            self.is_connected = True
            
        else:
            if self.serial_worker:
                self.serial_worker.stop()
            if self.serial_thread:
                self.serial_thread.quit()
                self.serial_thread.wait()
            self.connect_btn.setText("Connect")
            self.is_connected = False
            self.update_status("Disconnected")

    def handle_connection_lost(self):
        if self.serial_worker:
            self.serial_worker.stop()
        if self.serial_thread:
            self.serial_thread.quit()
            self.serial_thread.wait()
        self.connect_btn.setChecked(False)
        self.connect_btn.setText("Connect")
        self.is_connected = False
        self.update_status("Connection Lost", is_error=True)

    def update_status(self, message, is_error=False):
        self.status_label.setText(f"Status: {message}")
        if "Connected" in message:
            self.status_label.setStyleSheet("color: green;")
        elif is_error or "Error" in message or "Lost" in message or "Disconnected" in message:
            self.status_label.setStyleSheet("color: red;")
        else:
            self.status_label.setStyleSheet("color: black;")

    def update_slider_label(self, value):
        self.slider_label.setText(f"Angle: {value}")
    
    def send_command(self, cmd_dict):
        if self.is_connected and self.serial_worker:
            self.send_command_signal.emit(cmd_dict)
        elif not self.is_connected:
            self.show_error("Not connected to Arduino.")

    def send_move_command(self):
        angle = self.servo_slider.value()
        self.send_command({"cmd": "move", "value": angle})
        
    def send_tare_command(self):
        self.offset_label.setText("Offset: Taring...")
        self.send_command({"cmd": "tare"})
        
    def send_calibrate_command(self):
        try:
            float(self.known_weight_input.text())
            self.scale_label.setText("Scale: Calibrating...")
            self.send_command({"cmd": "calibrate"})
        except ValueError:
            self.show_error("Invalid known weight. Must be a number.")
            
    def handle_data(self, data):
        data_type = data.get("type")
        
        if data_type == "data":
            # This is live sensor data
            raw_load = data.get("load_raw", 0)
            gyro_z = data.get("gyro_z", 0.0) # Rad/s
            
            # Calculate scaled load
            current_load_g = (raw_load - self.load_cell_offset)
            if self.load_cell_scale != 0:
                current_load_g /= self.load_cell_scale

            # Calculate torque
            try:
                lever_m = self.lever_arm_input.value()
                force_n = (current_load_g / 1000.0) * 9.80665
                current_torque = abs(force_n * lever_m) # Torque in Nm
            except Exception:
                current_torque = 0.0

            # Update graph data
            elapsed_time = time.time() - self.start_time
            self.time_data.append(elapsed_time)
            self.load_data_g.append(current_load_g)
            self.torque_data_nm.append(current_torque)
            self.load_data_raw.append(raw_load) # <-- ADD THIS LINE

            # If a test is running, capture data
            if self.test_in_progress:
                test_elapsed = time.time() - self.test_start_time
                self.test_time_buffer.append(test_elapsed)
                self.test_load_buffer.append(current_load_g)
                self.test_gyro_buffer.append(gyro_z)

        elif data_type == "tare_result":
            self.load_cell_offset = data.get("offset", 0)
            self.offset_label.setText(f"Offset: {self.load_cell_offset}")
            # Automatically send the offset to the Arduino
            self.send_command({"cmd": "set_offset", "value": self.load_cell_offset})

        elif data_type == "calibration_result":
            try:
                known_weight_g = float(self.known_weight_input.text())
                raw_val = data.get("raw_val", 0)
                
                if known_weight_g == 0:
                    self.show_error("Known weight cannot be zero.")
                    self.scale_label.setText("Scale: Error")
                    return
                    
                # Calculate scale: (RawReading - Offset) / KnownWeight
                self.load_cell_scale = (raw_val - self.load_cell_offset) / known_weight_g
                self.scale_label.setText(f"Scale: {self.load_cell_scale:.4f}")
                # Automatically send the scale to the Arduino
                self.send_command({"cmd": "set_scale", "value": self.load_cell_scale})
            except ValueError:
                self.show_error("Invalid known weight.")
                self.scale_label.setText("Scale: Error")
        
        elif data_type == "status" or data_type == "error":
            self.update_status(data.get("msg", "Unknown status"), is_error=(data_type == "error"))

    def update_graphs(self):
        self.load_curve.setData(list(self.time_data), list(self.load_data_g))
        self.raw_load_curve.setData(list(self.time_data), list(self.load_data_raw)) # <-- ADD THIS LINE
        self.torque_curve.setData(list(self.time_data), list(self.torque_data_nm))

    def clear_test_buffers(self):
        self.test_time_buffer.clear()
        self.test_load_buffer.clear()
        self.test_gyro_buffer.clear()
        self.test_in_progress = True
        self.test_start_time = time.time()

    def run_stall_test(self):
        if self.test_in_progress:
            self.show_error("Test already in progress.")
            return
        
        # This test relies on the user's "decrease in force" method.
        # 1. Get baseline (weight at rest)
        if not self.load_data_g:
            self.show_error("No load data. Check connection.")
            return
        
        baseline_load_g = np.mean(list(self.load_data_g)[-20:]) # Avg of last 20 samples
        if baseline_load_g < 1.0:
            self.show_error("Place weight on load cell before starting.")
            return

        self.clear_test_buffers()
        self.stall_result_label.setText("Stall Torque: Testing...")

        # 2. Command servo to move
        self.send_command({"cmd": "move", "value": 90}) # Move to 90 degrees

        # 3. Wait for move to complete and data to be gathered
        QTimer.singleShot(TEST_MOVE_MS, self.calculate_stall_torque)

    def calculate_stall_torque(self):
        self.test_in_progress = False
        
        if len(self.test_load_buffer) < 20:
            self.show_error("Stall Test Failed: Not enough data.")
            self.stall_result_label.setText("Stall Torque: Fail")
            return
        
        # Get baseline force (weight at rest) before the test started
        baseline_load_g = self.test_load_buffer[0]
        
        # Find the minimum load during the lift (servo is providing max force)
        lifted_load_g = np.min(self.test_load_buffer)
        
        # Force supported by servo is the difference
        force_g_by_servo = baseline_load_g - lifted_load_g
        force_n = (force_g_by_servo / 1000.0) * 9.80665
        
        try:
            lever_m = self.lever_arm_input.value()
            if lever_m == 0:
                self.show_error("Lever arm length cannot be zero.")
                self.stall_result_label.setText("Stall Torque: Fail")
                return
                
            stall_torque = force_n * lever_m
            self.stall_result_label.setText(f"Stall Torque: {stall_torque:.4f} Nm")
        except Exception as e:
            self.stall_result_label.setText("Stall Torque: Error")
            self.show_error(f"Calculation Error: {e}")

    def run_rated_test(self):
        if self.test_in_progress:
            self.show_error("Test already in progress.")
            return

        self.clear_test_buffers()
        self.rated_result_label.setText("Rated Torque: Testing...")

        # Command a fast sweep
        self.send_command({"cmd": "move", "value": 0})
        QTimer.singleShot(500, lambda: self.send_command({"cmd": "move", "value": 120}))
        
        # Stop collecting data after the move
        QTimer.singleShot(TEST_MOVE_MS + 500, self.calculate_rated_torque)

    def calculate_rated_torque(self):
        self.test_in_progress = False

        if len(self.test_time_buffer) < 10:
            self.show_error("Rated Test Failed: Not enough data.")
            self.rated_result_label.setText("Rated Torque: Fail")
            return

        try:
            # Differentiate angular velocity (gyro_z) to get angular acceleration (alpha)
            # gyro_z is in rad/s
            time_s = np.array(self.test_time_buffer)
            gyro_rad_s = np.array(self.test_gyro_buffer)
            
            # Use numpy.gradient to find derivative
            alpha_rad_s2 = np.gradient(gyro_rad_s, time_s)
            
            # Find the peak angular acceleration
            peak_alpha = np.max(np.abs(alpha_rad_s2))
            
            # Get total inertia
            i_arm = self.inertia_arm_input.value()
            i_weight = self.inertia_weight_input.value()
            i_total = i_arm + i_weight
            
            if i_total == 0:
                self.show_error("Total inertia is zero. Please set parameters.")
                self.rated_result_label.setText("Rated Torque: Fail")
                return

            # Torque = I * alpha
            rated_torque = i_total * peak_alpha
            self.rated_result_label.setText(f"Rated Torque: {rated_torque:.4f} Nm")
            
        except Exception as e:
            self.rated_result_label.setText("Rated Torque: Error")
            self.show_error(f"Calculation Error: {e}")

    def show_error(self, message):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Icon.Warning)
        msg.setText(message)
        msg.setWindowTitle("Error")
        msg.exec()

    def closeEvent(self, event):
        # Clean up serial thread on exit
        if self.serial_worker:
            self.serial_worker.stop()
        if self.serial_thread:
            self.serial_thread.quit()
            self.serial_thread.wait()
        event.accept()

# --- Main Execution ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())