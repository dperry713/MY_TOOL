import sys
import time
import csv
import math
from collections import defaultdict
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem, QComboBox, QPushButton, QGridLayout,
    QProgressBar, QLineEdit, QMessageBox, QStatusBar, QFileDialog
)
from PyQt6.QtCore import QTimer, Qt, QRect, QPoint
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont

import pyqtgraph as pg
import obd

# Default configuration
NUM_CYLINDERS = 4
UPDATE_INTERVAL_MS = 500
LOG_FILE = "vehicle_log.csv"
RECONNECT_INTERVAL_MS = 5000  # Try reconnect every 5 seconds if disconnected

# Define bins for VE table
RPM_BINS = list(range(500, 8500, 500))
MAP_BINS = list(range(20, 110, 10))

# Data storage for VE
ve_data = defaultdict(list)

class CircularGauge(QWidget):
    def __init__(self, min_value=0, max_value=100, parent=None):
        super().__init__(parent)
        self.min_value = min_value
        self.max_value = max_value
        self.current_value = min_value
        self.start_angle = 225  # Starting angle in degrees
        self.span_angle = 270   # Span in degrees
        self.setMinimumSize(200, 200)

    def set_value(self, value):
        if self.min_value <= value <= self.max_value:
            self.current_value = value
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height())
        outer_rect = QRect(10, 10, side - 20, side - 20)
        center = outer_rect.center()

        # Background circle
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(50, 50, 50))
        painter.drawEllipse(outer_rect)

        # Background arc
        pen = QPen(QColor(100, 100, 100), 20, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawArc(outer_rect, self.start_angle * 16, -self.span_angle * 16)  # Negative for clockwise

        # Progress arc
        progress = (self.current_value - self.min_value) / (self.max_value - self.min_value)
        angle = self.span_angle * progress
        color = QColor(0, 255, 0) if progress < 0.8 else QColor(255, 0, 0)
        pen = QPen(color, 20, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawArc(outer_rect, self.start_angle * 16, -int(angle * 16))

        # Needle
        needle_angle = self.start_angle - angle
        needle_length = (side / 2) - 40
        dx = math.cos(math.radians(needle_angle)) * needle_length
        dy = -math.sin(math.radians(needle_angle)) * needle_length
        pen = QPen(QColor(255, 255, 255), 5, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawLine(center, QPoint(int(center.x() + dx), int(center.y() + dy)))

        # Value text
        painter.setPen(QColor(255, 255, 255))
        font = QFont("Arial", 20)
        painter.setFont(font)
        painter.drawText(outer_rect, Qt.AlignmentFlag.AlignCenter, f"{int(self.current_value)}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Advanced OBD-II Dashboard")
        self.setGeometry(100, 100, 1200, 800)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected - Select device and connect")

        # OBD connection
        self.connection = None
        self.async_connection = None
        self.supported_commands = []
        self.sensor_values = {}
        self.history = defaultdict(list)
        self.logging = False
        self.log_file = None
        self.log_writer = None
        self.num_cyl = NUM_CYLINDERS

        # Setup UI first (before connection)
        self.setup_ui()

        # Timers
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_dashboard)
        self.update_timer.start(UPDATE_INTERVAL_MS)

        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.reconnect_obd)
        self.reconnect_timer.start(RECONNECT_INTERVAL_MS)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Connection panel
        conn_panel = self.create_connection_panel()
        main_layout.addWidget(conn_panel)

        # Tab widget
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # Dashboard Tab
        dashboard_tab = QWidget()
        dashboard_layout = QGridLayout(dashboard_tab)

        # Gauges
        self.rpm_gauge = CircularGauge(0, 8000)
        self.rpm_label = QLabel("RPM: --")
        self.speed_gauge = CircularGauge(0, 200)
        self.speed_label = QLabel("Speed: -- km/h")

        # Other labels
        self.map_label = QLabel("MAP: -- kPa")
        self.maf_label = QLabel("MAF: -- g/s")
        self.iat_label = QLabel("IAT: -- °C")
        self.coolant_label = QLabel("Coolant: -- °C")
        self.throttle_label = QLabel("Throttle: -- %")
        self.ve_label = QLabel("VE: -- g*K/kPa")
        self.timing_label = QLabel("Ignition Timing: -- °")
        self.o2_b1s1_label = QLabel("O2 B1S1: -- V")
        self.o2_b1s2_label = QLabel("O2 B1S2: -- V")
        self.o2_b2s1_label = QLabel("O2 B2S1: -- V")
        self.o2_b2s2_label = QLabel("O2 B2S2: -- V")

        # Layout
        dashboard_layout.addWidget(self.rpm_label, 0, 0)
        dashboard_layout.addWidget(self.rpm_gauge, 0, 1)
        dashboard_layout.addWidget(self.speed_label, 1, 0)
        dashboard_layout.addWidget(self.speed_gauge, 1, 1)
        dashboard_layout.addWidget(self.map_label, 2, 0)
        dashboard_layout.addWidget(self.maf_label, 3, 0)
        dashboard_layout.addWidget(self.iat_label, 4, 0)
        dashboard_layout.addWidget(self.coolant_label, 5, 0)
        dashboard_layout.addWidget(self.throttle_label, 6, 0)
        dashboard_layout.addWidget(self.ve_label, 7, 0)
        dashboard_layout.addWidget(self.timing_label, 8, 0)
        dashboard_layout.addWidget(self.o2_b1s1_label, 9, 0)
        dashboard_layout.addWidget(self.o2_b1s2_label, 10, 0)
        dashboard_layout.addWidget(self.o2_b2s1_label, 11, 0)
        dashboard_layout.addWidget(self.o2_b2s2_label, 12, 0)

        self.tabs.addTab(dashboard_tab, "Dashboard")

        # Sensors Tab
        sensors_tab = QWidget()
        sensors_layout = QVBoxLayout(sensors_tab)
        self.sensors_table = QTableWidget()
        self.sensors_table.setColumnCount(2)
        self.sensors_table.setHorizontalHeaderLabels(["Sensor", "Value"])
        sensors_layout.addWidget(self.sensors_table)
        self.tabs.addTab(sensors_tab, "Sensors")

        # VE Table Tab
        ve_tab = QWidget()
        ve_layout = QVBoxLayout(ve_tab)
        self.ve_table = QTableWidget(len(MAP_BINS), len(RPM_BINS))
        self.ve_table.setVerticalHeaderLabels([str(m) for m in MAP_BINS])
        self.ve_table.setHorizontalHeaderLabels([str(r) for r in RPM_BINS])
        ve_layout.addWidget(self.ve_table)

        cyl_layout = QHBoxLayout()
        cyl_label = QLabel("Number of Cylinders:")
        self.cyl_input = QLineEdit(str(self.num_cyl))
        cyl_button = QPushButton("Update")
        cyl_button.clicked.connect(self.update_num_cyl)
        cyl_layout.addWidget(cyl_label)
        cyl_layout.addWidget(self.cyl_input)
        cyl_layout.addWidget(cyl_button)
        ve_layout.addLayout(cyl_layout)
        self.tabs.addTab(ve_tab, "VE Table")

        # Charts Tab
        charts_tab = QWidget()
        charts_layout = QVBoxLayout(charts_tab)
        self.sensor_dropdown = QComboBox()
        charts_layout.addWidget(self.sensor_dropdown)

        self.plot_widget = pg.PlotWidget(title="Time Series")
        self.plot_curve = self.plot_widget.plot(pen='y')
        charts_layout.addWidget(self.plot_widget)

        self.hist_widget = pg.PlotWidget(title="Histogram")
        self.hist_curve = self.hist_widget.plot(stepMode=True, fillLevel=0, brush=(0,0,255,150))
        charts_layout.addWidget(self.hist_widget)

        self.sensor_dropdown.currentIndexChanged.connect(self.update_chart)
        self.tabs.addTab(charts_tab, "Charts")

        # Log Timing Tab
        log_timing_tab = QWidget()
        log_timing_layout = QVBoxLayout(log_timing_tab)
        
        # Control buttons
        controls_layout = QHBoxLayout()
        self.clear_log_button = QPushButton("Clear Log")
        self.clear_log_button.clicked.connect(self.clear_log_timing)
        self.export_log_button = QPushButton("Export to CSV")
        self.export_log_button.clicked.connect(self.export_log_timing)
        controls_layout.addWidget(self.clear_log_button)
        controls_layout.addWidget(self.export_log_button)
        controls_layout.addStretch()
        log_timing_layout.addLayout(controls_layout)
        
        # Table for log entries - one row per timestamp with all sensors
        self.log_timing_table = QTableWidget()
        self.log_timing_table.setColumnCount(1)
        self.log_timing_table.setHorizontalHeaderLabels(["Timestamp | All Sensors"])
        self.log_timing_table.horizontalHeader().setStretchLastSection(True)
        log_timing_layout.addWidget(self.log_timing_table)
        
        # Dictionary to store the latest timestamp and sensor readings
        self.current_log_entry_time = None
        self.current_log_sensors = {}
        
        self.tabs.addTab(log_timing_tab, "Log Timing")

        # Logging Tab
        logging_tab = QWidget()
        logging_layout = QVBoxLayout(logging_tab)
        self.log_button = QPushButton("Start Logging")
        self.log_button.clicked.connect(self.toggle_logging)
        logging_layout.addWidget(self.log_button)
        self.tabs.addTab(logging_tab, "Logging")

    def create_connection_panel(self):
        """Create the connection control panel"""
        panel = QWidget()
        layout = QHBoxLayout(panel)
        
        layout.addWidget(QLabel("Device:"))
        self.device_dropdown = QComboBox()
        self.device_dropdown.addItem("(Scan for devices...)")
        layout.addWidget(self.device_dropdown)
        
        self.scan_button = QPushButton("Scan Devices")
        self.scan_button.clicked.connect(self.scan_devices)
        layout.addWidget(self.scan_button)
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_button)
        
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button)
        
        layout.addStretch()
        
        return panel

    def scan_devices(self):
        """Scan for available OBD devices"""
        self.device_dropdown.clear()
        self.device_dropdown.addItem("(Scanning...)")
        
        try:
            ports = obd.scan_serial()
            self.device_dropdown.clear()
            
            if ports:
                for port in ports:
                    self.device_dropdown.addItem(port, port)
                self.status_bar.showMessage(f"Found {len(ports)} device(s)")
            else:
                self.device_dropdown.addItem("(No devices found)")
                self.status_bar.showMessage("No OBD devices found")
        except Exception as e:
            self.device_dropdown.clear()
            self.device_dropdown.addItem(f"Error: {str(e)}")
            self.status_bar.showMessage(f"Scan error: {str(e)}")

    def on_connect_clicked(self):
        """Handle connect button click"""
        port = self.device_dropdown.currentData()
        if port is None:
            QMessageBox.warning(self, "No Device", "Please select a device first")
            return
        
        self.connect_button.setEnabled(False)
        self.scan_button.setEnabled(False)
        self.device_dropdown.setEnabled(False)
        self.status_bar.showMessage(f"Connecting to {port}...")
        self.connect_obd(port)

    def on_disconnect_clicked(self):
        """Handle disconnect button click"""
        self.disconnect_obd()
        self.scan_button.setEnabled(True)
        self.connect_button.setEnabled(True)
        self.device_dropdown.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.status_bar.showMessage("Disconnected")

    def connect_obd(self, port=None):
        try:
            if port:
                self.connection = obd.OBD(port)
            else:
                self.connection = obd.OBD()
            
            if not self.connection.is_connected():
                raise Exception("Connection failed - device not responding")
            
            self.supported_commands = [cmd for cmd in obd.commands.base_commands() if self.connection.supports(cmd)]
            self.async_connection = obd.Async(port) if port else obd.Async()
            for cmd in self.supported_commands:
                self.async_connection.watch(cmd, callback=self.update_sensor)
            self.async_connection.start()
            
            # Populate UI
            self.sensor_dropdown.clear()
            self.sensor_dropdown.addItems([cmd.name for cmd in self.supported_commands])
            self.sensors_table.setRowCount(len(self.supported_commands))
            for i, cmd in enumerate(self.supported_commands):
                self.sensors_table.setItem(i, 0, QTableWidgetItem(cmd.name))
            
            self.status_bar.showMessage("OBD Connected")
            self.disconnect_button.setEnabled(True)
        except Exception as e:
            self.status_bar.showMessage(f"OBD Connection Error: {str(e)}")
            QMessageBox.warning(self, "Connection Error", f"Failed to connect: {str(e)}")
            self.connect_button.setEnabled(True)
            self.scan_button.setEnabled(True)
            self.device_dropdown.setEnabled(True)

    def disconnect_obd(self):
        """Disconnect from OBD device"""
        if self.async_connection:
            self.async_connection.stop()
            self.async_connection = None
        
        if self.connection:
            self.connection.close()
            self.connection = None
        
        self.supported_commands = []
        self.sensor_values = {}
        self.history.clear()

    def reconnect_obd(self):
        if self.connection is None or not self.connection.is_connected():
            # Only auto-reconnect if we were previously connected
            pass

    def update_sensor(self, response):
        try:
            if response is None or response.value is None:
                return
            name = response.command.name
            value = response.value.magnitude if hasattr(response.value, 'magnitude') else response.value
            self.sensor_values[name] = value
            self.history[name].append(value)
            if len(self.history[name]) > 100:
                self.history[name].pop(0)
            
            # Add to log timing table
            self.add_log_timing_entry(name, value)
        except Exception as e:
            print(f"Sensor update error: {str(e)}")  # Log error

    def update_dashboard(self):
        try:
            if not self.connection or not self.connection.is_connected():
                return

            rpm = self.sensor_values.get('RPM', 0)
            speed = self.sensor_values.get('SPEED', 0)
            map_ = self.sensor_values.get('INTAKE_PRESSURE', 0)  # Use INTAKE_PRESSURE for MAP
            maf = self.sensor_values.get('MAF', 0)
            iat = self.sensor_values.get('INTAKE_TEMP', 0)
            coolant = self.sensor_values.get('COOLANT_TEMP', 0)
            throttle = self.sensor_values.get('THROTTLE_POS', 0)
            timing = self.sensor_values.get('TIMING_ADVANCE', 0)
            o2_b1s1 = self.sensor_values.get('O2_B1S1', 0)
            o2_b1s2 = self.sensor_values.get('O2_B1S2', 0)
            o2_b2s1 = self.sensor_values.get('O2_B2S1', 0)
            o2_b2s2 = self.sensor_values.get('O2_B2S2', 0)

            self.rpm_label.setText(f"RPM: {rpm}")
            self.rpm_gauge.set_value(rpm)
            self.speed_label.setText(f"Speed: {speed} km/h")
            self.speed_gauge.set_value(speed)
            self.map_label.setText(f"MAP: {map_} kPa")
            self.maf_label.setText(f"MAF: {maf} g/s")
            self.iat_label.setText(f"IAT: {iat} °C")
            self.coolant_label.setText(f"Coolant: {coolant} °C")
            self.throttle_label.setText(f"Throttle: {throttle} %")
            self.timing_label.setText(f"Ignition Timing: {timing} °")
            self.o2_b1s1_label.setText(f"O2 B1S1: {o2_b1s1} V")
            self.o2_b1s2_label.setText(f"O2 B1S2: {o2_b1s2} V")
            self.o2_b2s1_label.setText(f"O2 B2S1: {o2_b2s1} V")
            self.o2_b2s2_label.setText(f"O2 B2S2: {o2_b2s2} V")

            # VE calculation
            if rpm > 0 and maf > 0 and map_ > 0 and iat is not None:
                temp_k = iat + 273.15
                intakes_per_sec = (rpm * self.num_cyl) / 120.0
                g_cyl = maf / intakes_per_sec if intakes_per_sec > 0 else 0
                ve = (g_cyl * temp_k) / map_ if map_ > 0 else 0
                self.ve_label.setText(f"VE: {ve:.4f} g*K/kPa")

                # Populate VE table
                rpm_idx = min(range(len(RPM_BINS)), key=lambda i: abs(RPM_BINS[i] - rpm))
                map_idx = min(range(len(MAP_BINS)), key=lambda i: abs(MAP_BINS[i] - map_))
                ve_data[(map_idx, rpm_idx)].append(ve)
                avg_ve = np.mean(ve_data[(map_idx, rpm_idx)]) if ve_data[(map_idx, rpm_idx)] else 0
                item = QTableWidgetItem(f"{avg_ve:.4f}")
                self.ve_table.setItem(map_idx, rpm_idx, item)

            # Update sensors table
            for i, cmd in enumerate(self.supported_commands):
                value = self.sensor_values.get(cmd.name, "--")
                self.sensors_table.setItem(i, 1, QTableWidgetItem(str(value)))

            # Logging
            if self.logging and self.log_writer:
                timestamp = time.time()
                row = [timestamp, rpm, map_, maf, iat, self.ve_label.text().split()[-1], speed, coolant, throttle, timing, o2_b1s1, o2_b1s2, o2_b2s1, o2_b2s2]
                self.log_writer.writerow(row)
        except Exception as e:
            self.status_bar.showMessage(f"Dashboard update error: {str(e)}")

    def update_chart(self):
        try:
            sensor = self.sensor_dropdown.currentText()
            if sensor in self.history:
                data = self.history[sensor]
                self.plot_curve.setData(data)
                if data:
                    hist, bins = np.histogram(data, bins=20)
                    self.hist_curve.setData(bins, hist)
        except Exception as e:
            print(f"Chart update error: {str(e)}")

    def toggle_logging(self):
        self.logging = not self.logging
        if self.logging:
            try:
                self.log_file = open(LOG_FILE, 'w', newline='')
                self.log_writer = csv.writer(self.log_file)
                self.log_writer.writerow(['Timestamp', 'RPM', 'MAP', 'MAF', 'IAT', 'VE', 'Speed', 'Coolant', 'Throttle', 'Timing', 'O2_B1S1', 'O2_B1S2', 'O2_B2S1', 'O2_B2S2'])
                self.log_button.setText("Stop Logging")
            except Exception as e:
                QMessageBox.warning(self, "Logging Error", f"Failed to start logging: {str(e)}")
                self.logging = False
        else:
            if self.log_file:
                self.log_file.close()
            self.log_button.setText("Start Logging")

    def update_num_cyl(self):
        try:
            self.num_cyl = int(self.cyl_input.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Invalid number of cylinders")

    def add_log_timing_entry(self, sensor_name, value):
        """Add a sensor reading to the log timing table - groups by timestamp"""
        try:
            # Get current timestamp (rounded to 100ms for grouping)
            current_time = time.time()
            timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(current_time))
            millis = int((current_time % 1) * 1000)
            timestamp_str = f"{timestamp_str}.{millis:03d}"
            
            # Format value
            value_str = f"{value:.2f}" if isinstance(value, (int, float)) else str(value)
            
            # Add sensor to current entry
            self.current_log_sensors[sensor_name] = value_str
            
            # If timestamp changed, write previous line and start new one
            if self.current_log_entry_time != timestamp_str:
                # Write previous entry if it exists
                if self.current_log_entry_time is not None and self.current_log_sensors:
                    self._write_log_line(self.current_log_entry_time, self.current_log_sensors)
                
                # Start new entry
                self.current_log_entry_time = timestamp_str
                self.current_log_sensors = {sensor_name: value_str}
        except Exception as e:
            print(f"Log timing entry error: {str(e)}")

    def _write_log_line(self, timestamp, sensors_dict):
        """Write a single log line with timestamp and all sensors"""
        try:
            row = self.log_timing_table.rowCount()
            self.log_timing_table.insertRow(row)
            
            # Format all sensors into a single string
            sensor_str = " | ".join([f"{name}:{value}" for name, value in sensors_dict.items()])
            log_line = f"{timestamp} | {sensor_str}"
            
            self.log_timing_table.setItem(row, 0, QTableWidgetItem(log_line))
            
            # Scroll to bottom to show latest entry
            self.log_timing_table.scrollToBottom()
            
            # Keep table from growing too large (limit to 500 rows)
            if row > 500:
                self.log_timing_table.removeRow(0)
        except Exception as e:
            print(f"Log line write error: {str(e)}")

    def clear_log_timing(self):
        """Clear all entries from the log timing table"""
        reply = QMessageBox.question(
            self, 
            "Clear Log", 
            "Are you sure you want to clear all log entries?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.log_timing_table.setRowCount(0)
            self.current_log_entry_time = None
            self.current_log_sensors = {}

    def export_log_timing(self):
        """Export log timing entries to CSV file"""
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Export Log Timing",
                "",
                "CSV Files (*.csv);;All Files (*)"
            )
            if not filename:
                return
            
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # Write header
                writer.writerow(["Timestamp and Sensors"])
                
                # Write data
                for row in range(self.log_timing_table.rowCount()):
                    item = self.log_timing_table.item(row, 0)
                    if item:
                        writer.writerow([item.text()])
            
            QMessageBox.information(self, "Export Successful", f"Log timing exported to {filename}")
        except Exception as e:
            QMessageBox.warning(self, "Export Error", f"Failed to export log: {str(e)}")

    def closeEvent(self, event):
        if self.async_connection:
            self.async_connection.stop()
        if self.log_file:
            self.log_file.close()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Apply dark theme
    app.setStyle("Fusion")
    from PyQt6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(30, 30, 30))
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(35, 35, 35))
    palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Button, QColor(40, 40, 40))
    palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(palette)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec())