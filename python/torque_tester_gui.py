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
    QGridLayout, QDoubleSpinBox, QMessageBox, QTextEdit, QCheckBox,
    QDialog
)
from PyQt6.QtCore import (
    QThread, QObject, pyqtSignal, Qt, QTimer, QSize
)
import pyqtgraph as pg

# --- Constants ---
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0  # Non-blocking read
DATA_BUFFER_SIZE = 200
GRAPH_UPDATE_MS = 50
TEST_MOVE_MS = 1000
CALIBRATION_SAMPLES = 20
SLIDER_THROTTLE_MS = 100

# --- Serial Console Window ---
class SerialConsoleWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Raw Serial Console")
        self.setGeometry(200, 200, 600, 400)
        
        layout = QVBoxLayout()
        
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setStyleSheet("font-family: Consolas, Monaco, monospace;")
        layout.addWidget(self.text_edit)
        
        btn_layout = QHBoxLayout()
        self.autoscroll_cb = QCheckBox("Auto-scroll")
        self.autoscroll_cb.setChecked(True)
        btn_layout.addWidget(self.autoscroll_cb)
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.text_edit.clear)
        btn_layout.addWidget(self.clear_btn)
        
        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def append_text(self, text):
        self.text_edit.moveCursor(self.text_edit.textCursor().MoveOperation.End)
        self.text_edit.insertPlainText(text + "\n")
        if self.autoscroll_cb.isChecked():
            self.text_edit.ensureCursorVisible()

# --- Serial Communication Worker ---
class SerialWorker(QObject):
    data_received = pyqtSignal(dict)
    raw_line_received = pyqtSignal(str)
    data_sent = pyqtSignal(str)
    status_received = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.serial_port = None
        self.timer = None

    def run(self):
        """
        Initializes connection and starts the polling timer.
        IMPORTANT: This method returns immediately so the thread's event loop can run.
        """
        try:
            self.serial_port = serial.Serial(
                self.port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT
            )
            self.status_received.emit("Connected")
            
            # Use a QTimer to poll for data instead of a blocking while loop
            self.timer = QTimer()
            self.timer.timeout.connect(self.check_serial)
            self.timer.start(10) # Poll every 10ms
            
        except serial.SerialException as e:
            self.status_received.emit(f"Error: {e}")

    def check_serial(self):
        """
        Called periodically by QTimer to read available data.
        """
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            if self.serial_port.in_waiting > 0:
                # Read all available lines
                while self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        self.raw_line_received.emit(line)
                        try:
                            data = json.loads(line)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            pass
        except (serial.SerialException, OSError) as e:
            self.status_received.emit(f"Serial Error: {e}")
            self.connection_lost.emit()
            self.stop()

    def write_command(self, cmd_dict):
        """
        Slot to write data. Now executed by the event loop freely.
        """
        if self.serial_port and self.serial_port.is_open:
            cmd_str = json.dumps(cmd_dict)
            # CRLF is safer for Windows/Serial
            packet = cmd_str + '\r\n'
            try:
                self.serial_port.write(packet.encode('utf-8'))
                self.serial_port.flush()
                self.data_sent.emit(f"TX: {cmd_str}")
            except serial.SerialException as e:
                self.status_received.emit(f"Write Error: {e}")
                self.connection_lost.emit()

    def stop(self):
        if self.timer:
            self.timer.stop()
        if self.serial_port:
            self.serial_port.close()

# --- Main Application Window ---
class MainWindow(QMainWindow):
    send_command_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Servo Torque Tester GUI")
        self.setGeometry(100, 100, 1200, 800)

        self.serial_thread = None
        self.serial_worker = None
        self.console_window = None
        
        # State variables
        self.load_cell_offset = 0.0
        self.load_cell_scale = 1.0
        self.is_connected = False
        self.test_in_progress = False
        self.motor_is_active = False
        self.last_slider_send_time = 0
        
        # Calibration State
        self.cal_mode = "IDLE"
        self.cal_buffer = []

        # Physics State
        self.last_gyro_z = 0.0
        self.last_data_time = 0.0

        # Buffers
        self.time_data = deque(maxlen=DATA_BUFFER_SIZE)
        self.load_data_g = deque(maxlen=DATA_BUFFER_SIZE)
        self.torque_data_nm = deque(maxlen=DATA_BUFFER_SIZE)
        self.load_data_raw = deque(maxlen=DATA_BUFFER_SIZE)
        self.accel_data_rads2 = deque(maxlen=DATA_BUFFER_SIZE)
        
        self.test_time_buffer = []
        self.test_gyro_buffer = []
        self.test_load_buffer = []
        self.test_start_time = 0

        self.start_time = time.time()

        self.init_ui()
        self.populate_com_ports()

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

        # 1. Connection
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
        self.console_btn = QPushButton("Show Serial Console")
        self.console_btn.clicked.connect(self.show_console_window)
        conn_layout.addWidget(self.console_btn, 1, 2)
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: red;")
        conn_layout.addWidget(self.status_label, 1, 0, 1, 1)
        conn_box.setLayout(conn_layout)
        control_layout.addWidget(conn_box)

        # 2. Calibration
        cal_box = QGroupBox("2. Load Cell Calibration")
        cal_layout = QGridLayout()
        self.tare_btn = QPushButton("Tare (Remove Weight)")
        self.tare_btn.clicked.connect(self.start_tare)
        cal_layout.addWidget(self.tare_btn, 0, 0, 1, 2)
        cal_layout.addWidget(QLabel("Known Weight (g):"), 1, 0)
        self.known_weight_input = QLineEdit("100.0")
        cal_layout.addWidget(self.known_weight_input, 1, 1)
        self.calibrate_btn = QPushButton("Calibrate (Place Weight)")
        self.calibrate_btn.clicked.connect(self.start_calibrate)
        cal_layout.addWidget(self.calibrate_btn, 2, 0, 1, 2)
        self.offset_label = QLabel("Offset: 0")
        cal_layout.addWidget(self.offset_label, 3, 0)
        self.scale_label = QLabel("Scale: 1.0")
        cal_layout.addWidget(self.scale_label, 3, 1)
        cal_box.setLayout(cal_layout)
        control_layout.addWidget(cal_box)

        # 3. Parameters
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

        # 4. Controls
        manual_box = QGroupBox("4. Manual Servo Control")
        manual_layout = QGridLayout()
        self.toggle_motor_btn = QPushButton("Motor Status: OFF (Click to Activate)")
        self.toggle_motor_btn.setCheckable(True)
        self.toggle_motor_btn.setStyleSheet("background-color: #ffcccc; color: black;") 
        self.toggle_motor_btn.clicked.connect(self.toggle_motor_active)
        manual_layout.addWidget(self.toggle_motor_btn, 0, 0, 1, 3)
        self.servo_slider = QSlider(Qt.Orientation.Horizontal)
        self.servo_slider.setRange(0, 180)
        self.servo_slider.setValue(165) 
        self.servo_slider.valueChanged.connect(self.throttled_slider_move)
        manual_layout.addWidget(self.servo_slider, 1, 0, 1, 2)
        self.slider_label = QLabel("Angle: 165")
        manual_layout.addWidget(self.slider_label, 1, 2)
        self.move_servo_btn = QPushButton("Move Servo (Force)")
        self.move_servo_btn.clicked.connect(self.force_move_command)
        manual_layout.addWidget(self.move_servo_btn, 2, 0, 1, 3)
        manual_box.setLayout(manual_layout)
        control_layout.addWidget(manual_box)

        # 5. Testing
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

        # --- Graphs ---
        graph_layout = QGridLayout()
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        
        self.load_plot = pg.PlotWidget(title="Live Load Cell (grams)")
        self.load_plot.setLabel('left', 'Load (g)')
        self.load_plot.showGrid(x=True, y=True)
        self.load_plot.getPlotItem().getViewBox().setLimits(minYRange=50)
        self.load_curve = self.load_plot.plot(pen=pg.mkPen('g', width=2))
        graph_layout.addWidget(self.load_plot, 0, 0)

        self.torque_plot = pg.PlotWidget(title="Live Torque (Nm)")
        self.torque_plot.setLabel('left', 'Torque (Nm)')
        self.torque_plot.showGrid(x=True, y=True)
        self.torque_plot.getPlotItem().getViewBox().setLimits(minYRange=0.2)
        self.torque_curve = self.torque_plot.plot(pen=pg.mkPen('r', width=2))
        graph_layout.addWidget(self.torque_plot, 0, 1)

        self.raw_load_plot = pg.PlotWidget(title="Raw Load Cell (ADC Value)")
        self.raw_load_plot.setLabel('left', 'Raw ADC Value')
        self.raw_load_plot.showGrid(x=True, y=True)
        self.raw_load_plot.getPlotItem().getViewBox().setLimits(minYRange=1000)
        self.raw_load_curve = self.raw_load_plot.plot(pen=pg.mkPen('b', width=2))
        graph_layout.addWidget(self.raw_load_plot, 1, 0)

        self.accel_plot = pg.PlotWidget(title="Gyro Z Acceleration (rad/s²)")
        self.accel_plot.setLabel('left', 'Accel (rad/s²)')
        self.accel_plot.showGrid(x=True, y=True)
        self.accel_plot.getPlotItem().getViewBox().setLimits(minYRange=5.0)
        self.accel_curve = self.accel_plot.plot(pen=pg.mkPen('m', width=2)) 
        graph_layout.addWidget(self.accel_plot, 1, 1)

        main_layout.addLayout(graph_layout, 2)

    # --- Functions ---
    def show_console_window(self):
        if self.console_window is None:
            self.console_window = SerialConsoleWindow(self)
        self.console_window.show()
        self.console_window.raise_()
        self.console_window.activateWindow()

    def update_console(self, text):
        if self.console_window and self.console_window.isVisible():
            self.console_window.append_text(text)

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

            self.serial_worker.data_received.connect(self.handle_data)
            self.serial_worker.raw_line_received.connect(self.update_console)
            self.serial_worker.data_sent.connect(self.update_console)
            self.serial_worker.status_received.connect(self.update_status)
            self.serial_worker.connection_lost.connect(self.handle_connection_lost)
            self.send_command_signal.connect(self.serial_worker.write_command)
            self.serial_thread.started.connect(self.serial_worker.run)

            self.serial_thread.start()
            self.connect_btn.setText("Disconnect")
            self.is_connected = True
            self.cal_mode = "IDLE"
            self.last_gyro_z = 0.0
            self.last_data_time = time.time()
            self.motor_is_active = False
            self.update_motor_ui()
        else:
            if self.serial_worker: self.serial_worker.stop()
            if self.serial_thread: self.serial_thread.quit(); self.serial_thread.wait()
            self.connect_btn.setText("Connect")
            self.is_connected = False
            self.update_status("Disconnected")

    def handle_connection_lost(self):
        if self.serial_worker: self.serial_worker.stop()
        if self.serial_thread: self.serial_thread.quit(); self.serial_thread.wait()
        self.connect_btn.setChecked(False)
        self.connect_btn.setText("Connect")
        self.is_connected = False
        self.update_status("Connection Lost", is_error=True)

    def update_status(self, message, is_error=False):
        self.status_label.setText(f"Status: {message}")
        self.status_label.setStyleSheet("color: red;" if is_error or "Error" in message else ("color: green;" if "Connected" in message else "color: black;"))

    def toggle_motor_active(self):
        if not self.is_connected:
            self.toggle_motor_btn.setChecked(False)
            self.show_error("Connect to Arduino first.")
            return
        self.motor_is_active = self.toggle_motor_btn.isChecked()
        self.send_command({"cmd": "active", "value": 1 if self.motor_is_active else 0})
        self.update_motor_ui()

    def update_motor_ui(self):
        if self.motor_is_active:
            self.toggle_motor_btn.setText("Motor Status: ON (Click to Deactivate)")
            self.toggle_motor_btn.setStyleSheet("background-color: #ccffcc; color: black;") 
            self.toggle_motor_btn.setChecked(True)
        else:
            self.toggle_motor_btn.setText("Motor Status: OFF (Click to Activate)")
            self.toggle_motor_btn.setStyleSheet("background-color: #ffcccc; color: black;") 
            self.toggle_motor_btn.setChecked(False)

    def throttled_slider_move(self, value):
        self.slider_label.setText(f"Angle: {value}")
        now = time.time() * 1000
        if now - self.last_slider_send_time > SLIDER_THROTTLE_MS:
            self.send_command({"cmd": "move", "value": value})
            self.last_slider_send_time = now

    def force_move_command(self):
        angle = self.servo_slider.value()
        self.send_command({"cmd": "move", "value": angle})
    
    def send_command(self, cmd_dict):
        if self.is_connected and self.serial_worker:
            self.send_command_signal.emit(cmd_dict)
        elif not self.is_connected:
            self.show_error("Not connected to Arduino.")

    def start_tare(self):
        if not self.is_connected: return
        self.cal_mode = "TARING"
        self.cal_buffer = []
        self.offset_label.setText("Offset: Sampling...")

    def start_calibrate(self):
        if not self.is_connected: return
        try:
            float(self.known_weight_input.text())
            self.cal_mode = "CALIBRATING"
            self.cal_buffer = []
            self.scale_label.setText("Scale: Sampling...")
        except ValueError: self.show_error("Invalid known weight.")

    def process_calibration(self, raw_val):
        self.cal_buffer.append(raw_val)
        if len(self.cal_buffer) >= CALIBRATION_SAMPLES:
            avg_val = np.mean(self.cal_buffer)
            if self.cal_mode == "TARING":
                self.load_cell_offset = avg_val
                self.offset_label.setText(f"Offset: {self.load_cell_offset:.1f}")
            elif self.cal_mode == "CALIBRATING":
                try:
                    known_g = float(self.known_weight_input.text())
                    if known_g > 0:
                        self.load_cell_scale = (avg_val - self.load_cell_offset) / known_g
                        self.scale_label.setText(f"Scale: {self.load_cell_scale:.4f}")
                except: pass
            self.cal_mode = "IDLE"
            self.cal_buffer = []

    def handle_data(self, data):
        data_type = data.get("type")
        if data_type == "data":
            raw_load = data.get("load_raw", 0)
            gyro_z = data.get("gyro_z", 0.0)
            if self.cal_mode != "IDLE": self.process_calibration(raw_load)
            
            current_load_g = (raw_load - self.load_cell_offset) / self.load_cell_scale if self.load_cell_scale != 0 else 0
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            dt = current_time - self.last_data_time
            current_accel = (gyro_z - self.last_gyro_z) / dt if dt > 0 else 0
            self.last_gyro_z = gyro_z
            self.last_data_time = current_time

            try:
                lever_m = self.lever_arm_input.value()
                force_n = (current_load_g / 1000.0) * 9.80665
                torque = abs(force_n * lever_m)
            except: torque = 0.0

            self.time_data.append(elapsed_time)
            self.load_data_g.append(current_load_g)
            self.torque_data_nm.append(torque)
            self.load_data_raw.append(raw_load)
            self.accel_data_rads2.append(current_accel)

            if self.test_in_progress:
                self.test_time_buffer.append(current_time - self.test_start_time)
                self.test_load_buffer.append(current_load_g)
                self.test_gyro_buffer.append(gyro_z)
        elif data_type == "status": self.update_status(data.get("msg"))
        elif data_type == "error": self.update_status(data.get("msg"), True)

    def update_graphs(self):
        times = list(self.time_data)
        self.load_curve.setData(times, list(self.load_data_g))
        self.torque_curve.setData(times, list(self.torque_data_nm))
        self.raw_load_curve.setData(times, list(self.load_data_raw))
        self.accel_curve.setData(times, list(self.accel_data_rads2))

    def run_stall_test(self):
        if self.test_in_progress or not self.load_data_g: return
        self.test_time_buffer.clear(); self.test_load_buffer.clear(); self.test_gyro_buffer.clear()
        self.test_in_progress = True
        self.test_start_time = time.time()
        self.stall_result_label.setText("Stall Torque: Testing...")
        self.send_command({"cmd": "move", "value": 90}) 
        QTimer.singleShot(TEST_MOVE_MS, self.calculate_stall_torque)

    def calculate_stall_torque(self):
        self.test_in_progress = False
        if len(self.test_load_buffer) < 20:
            self.stall_result_label.setText("Stall Torque: Fail")
            return
        base = self.test_load_buffer[0]
        lifted = np.min(self.test_load_buffer)
        force_n = ((base - lifted) / 1000.0) * 9.80665
        torque = force_n * self.lever_arm_input.value()
        self.stall_result_label.setText(f"Stall Torque: {torque:.4f} Nm")

    def run_rated_test(self):
        if self.test_in_progress: return
        self.test_time_buffer.clear(); self.test_load_buffer.clear(); self.test_gyro_buffer.clear()
        self.test_in_progress = True
        self.test_start_time = time.time()
        self.rated_result_label.setText("Rated Torque: Testing...")
        self.send_command({"cmd": "move", "value": 0})
        QTimer.singleShot(500, lambda: self.send_command({"cmd": "move", "value": 120}))
        QTimer.singleShot(TEST_MOVE_MS + 500, self.calculate_rated_torque)

    def calculate_rated_torque(self):
        self.test_in_progress = False
        if len(self.test_time_buffer) < 10:
            self.rated_result_label.setText("Rated Torque: Fail")
            return
        time = np.array(self.test_time_buffer)
        gyro = np.array(self.test_gyro_buffer)
        alpha = np.gradient(gyro, time)
        peak_alpha = np.max(np.abs(alpha))
        inertia = self.inertia_arm_input.value() + self.inertia_weight_input.value()
        torque = inertia * peak_alpha
        self.rated_result_label.setText(f"Rated Torque: {torque:.4f} Nm")

    def show_error(self, message):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Icon.Warning)
        msg.setText(message)
        msg.exec()

    def closeEvent(self, event):
        if self.serial_worker: self.serial_worker.stop()
        if self.serial_thread: self.serial_thread.quit(); self.serial_thread.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())