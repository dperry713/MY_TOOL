import sys
import time
import csv
import math
from collections import defaultdict
import numpy as np
from serial.tools.list_ports import comports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem, QComboBox, QPushButton, QGridLayout,
    QLineEdit, QMessageBox, QStatusBar, QFileDialog, QScrollArea
)
from PyQt6.QtCore import QTimer, Qt, QRect, QPoint
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPalette, QLinearGradient

import pyqtgraph as pg
import obd
import threading
import traceback
import logging

# Logging
LOG_FILE = "my_tool_debug.log"
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE, mode="a"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("MY_TOOL")

# Default configuration
NUM_CYLINDERS = 8  # For 5.3 Vortec V8
UPDATE_INTERVAL_MS = 500
LOG_FILE = "vehicle_log.csv"
RECONNECT_INTERVAL_MS = 5000  # Try reconnect every 5 seconds if disconnected

# Define bins for VE table as per specs
RPM_BINS = list(range(400, 8001, 400))  # 400,800,...,8000 len=20
MAP_BINS = list(range(20, 106, 5))  # 20,25,...,105 len=18

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

        # Background circle with gradient
        gradient = QLinearGradient(0, 0, 0, side)
        gradient.setColorAt(0, QColor(60, 60, 60))
        gradient.setColorAt(1, QColor(30, 30, 30))
        painter.setBrush(QBrush(gradient))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(outer_rect)

        # Background arc
        pen = QPen(QColor(80, 80, 80), 20, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawArc(outer_rect, self.start_angle * 16, -self.span_angle * 16)  # Negative for clockwise

        # Progress arc with gradient color
        progress = (self.current_value - self.min_value) / (self.max_value - self.min_value)
        angle = self.span_angle * progress
        color_start = QColor(0, 200, 0)
        color_end = QColor(255, 50, 0)
        gradient = QLinearGradient(center.x() - side/2, center.y() - side/2, center.x() + side/2, center.y() + side/2)
        gradient.setColorAt(0, color_start)
        gradient.setColorAt(1, color_end if progress > 0.8 else color_start)
        pen = QPen(QBrush(gradient), 20, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawArc(outer_rect, self.start_angle * 16, -int(angle * 16))

        # Needle with shadow
        needle_angle = self.start_angle - angle
        needle_length = (side / 2) - 40
        dx = math.cos(math.radians(needle_angle)) * needle_length
        dy = -math.sin(math.radians(needle_angle)) * needle_length
        pen = QPen(QColor(255, 255, 255), 5, Qt.PenStyle.SolidLine)
        painter.setPen(pen)
        painter.drawLine(center, QPoint(int(center.x() + dx), int(center.y() + dy)))

        # Value text with modern font
        painter.setPen(QColor(220, 220, 220))
        font = QFont("Segoe UI", 20, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(outer_rect, Qt.AlignmentFlag.AlignCenter, f"{int(self.current_value)}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Advanced OBD-II Dashboard for 5.3 Vortec V8")
        self.setGeometry(100, 100, 1200, 800)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected - Select device and connect")

        # OBD connection
        self.connection = None
        self.async_connection = None
        self.supported_commands = []
        self.gm_custom_commands = self.define_gm_pids()
        self.sensor_values = {}
        self.history = defaultdict(list)
        self.logging = False
        self.log_file = None
        self.log_writer = None
        self.num_cyl = NUM_CYLINDERS
        self.connected = False
        self.last_port = None

        # VE data
        self.ve_data_array = np.zeros((len(MAP_BINS), len(RPM_BINS)))
        self.ve_counts = np.zeros_like(self.ve_data_array)

        # Setup UI first (before connection)
        self.setup_ui()

        # Timers
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_dashboard)
        self.update_timer.start(UPDATE_INTERVAL_MS)

        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.reconnect_obd)
        self.reconnect_timer.start(RECONNECT_INTERVAL_MS)

    def define_gm_pids(self):
        # Define GM-specific Mode 22 PIDs for 2002 Silverado 5.3L Vortec
        def trans_temp_decoder(messages):
            d = messages[0].data
            if len(d) > 0:
                return d[0] - 40
            return None

        def oil_temp_decoder(messages):
            d = messages[0].data
            if len(d) > 0:
                return d[0] - 40
            return None

        def oil_pressure_decoder(messages):
            d = messages[0].data
            if len(d) > 0:
                return d[0] * 0.578
            return None

        def gear_position_decoder(messages):
            d = messages[0].data
            if len(d) > 0:
                a = d[0]
                lookup = {5: 'P/N', 4: 'Reverse', 1: 'Drive', 3: '3rd', 7: '2nd', 6: '1st'}
                return lookup.get(a if a <= 5 else a - 32, 'Unknown')
            return None

        return [
            obd.OBDCommand("TRANS_TEMP", "Transmission Temperature", b"221940", 2, trans_temp_decoder),
            obd.OBDCommand("OIL_TEMP", "Engine Oil Temperature", b"221154", 2, oil_temp_decoder),
            obd.OBDCommand("OIL_PRESSURE", "Engine Oil Pressure", b"221470", 2, oil_pressure_decoder),
            obd.OBDCommand("GEAR_POSITION", "Gear Position", b"221951", 2, gear_position_decoder),
        ]

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Connection panel with modern styling
        conn_panel = self.create_connection_panel()
        main_layout.addWidget(conn_panel)

        # Tab widget with modern style
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #444; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #333, stop:1 #222); }
            QTabBar::tab { background: #444; color: white; padding: 8px 16px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
            QTabBar::tab:selected { background: #555; }
        """)
        main_layout.addWidget(self.tabs)

        # Dashboard Tab with scroll for more labels
        dashboard_tab = QWidget()
        dashboard_scroll = QScrollArea()
        dashboard_scroll.setWidgetResizable(True)
        dashboard_scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        dashboard_content = QWidget()
        dashboard_layout = QGridLayout(dashboard_content)
        dashboard_content.setStyleSheet("background: transparent;")

        # Gauges
        self.rpm_gauge = CircularGauge(0, 8000)
        self.rpm_label = QLabel("RPM: --")
        self.rpm_label.setStyleSheet("color: #ddd; font: bold 14px 'Segoe UI';")
        self.speed_gauge = CircularGauge(0, 200)
        self.speed_label = QLabel("Speed: -- km/h")
        self.speed_label.setStyleSheet("color: #ddd; font: bold 14px 'Segoe UI';")

        # Other labels with modern styling
        self.map_label = QLabel("MAP: -- kPa")
        self.map_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.maf_label = QLabel("MAF: -- g/s")
        self.maf_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.iat_label = QLabel("IAT: -- °C")
        self.iat_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.coolant_label = QLabel("Coolant: -- °C")
        self.coolant_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.throttle_label = QLabel("Throttle: -- %")
        self.throttle_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.ve_label = QLabel("VE: -- g*K/kPa")
        self.ve_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.timing_label = QLabel("Ignition Timing: -- °")
        self.timing_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.o2_b1s1_label = QLabel("O2 B1S1: -- V")
        self.o2_b1s1_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.o2_b1s2_label = QLabel("O2 B1S2: -- V")
        self.o2_b1s2_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.o2_b2s1_label = QLabel("O2 B2S1: -- V")
        self.o2_b2s1_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.o2_b2s2_label = QLabel("O2 B2S2: -- V")
        self.o2_b2s2_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.trans_temp_label = QLabel("Trans Temp: -- °C")
        self.trans_temp_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.oil_temp_label = QLabel("Oil Temp: -- °C")
        self.oil_temp_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.oil_pressure_label = QLabel("Oil Pressure: -- psi")
        self.oil_pressure_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")
        self.gear_position_label = QLabel("Gear Position: --")
        self.gear_position_label.setStyleSheet("color: #ddd; font: 12px 'Segoe UI';")

        # Layout with two columns for gauges and labels
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
        dashboard_layout.addWidget(self.trans_temp_label, 13, 0)
        dashboard_layout.addWidget(self.oil_temp_label, 14, 0)
        dashboard_layout.addWidget(self.oil_pressure_label, 15, 0)
        dashboard_layout.addWidget(self.gear_position_label, 16, 0)
        dashboard_scroll.setWidget(dashboard_content)
        dashboard_tab.setLayout(QVBoxLayout())
        dashboard_tab.layout().addWidget(dashboard_scroll)
        self.tabs.addTab(dashboard_tab, "Dashboard")

        # Sensors Tab with modern table style
        sensors_tab = QWidget()
        sensors_layout = QVBoxLayout(sensors_tab)
        self.sensors_table = QTableWidget()
        self.sensors_table.setColumnCount(2)
        self.sensors_table.setHorizontalHeaderLabels(["Sensor", "Value"])
        self.sensors_table.setStyleSheet("""
            QTableWidget { background-color: #333; color: #ddd; gridline-color: #444; }
            QHeaderView::section { background-color: #444; color: white; }
        """)
        sensors_layout.addWidget(self.sensors_table)
        self.tabs.addTab(sensors_tab, "Sensors")

        # VE Table Tab with styled table and heatmap
        ve_tab = QWidget()
        ve_layout = QVBoxLayout(ve_tab)
        self.ve_table = QTableWidget(len(MAP_BINS), len(RPM_BINS))
        self.ve_table.setVerticalHeaderLabels([str(m) for m in MAP_BINS])
        self.ve_table.setHorizontalHeaderLabels([str(r) for r in RPM_BINS])
        self.ve_table.setStyleSheet("""
            QTableWidget { background-color: #333; color: #ddd; gridline-color: #444; }
            QHeaderView::section { background-color: #444; color: white; }
        """)
        ve_layout.addWidget(self.ve_table)

        # VE Heatmap
        self.ve_heatmap = pg.ImageView()
        self.ve_heatmap.setStyleSheet("background-color: transparent;")
        ve_layout.addWidget(self.ve_heatmap)
        # Set colormap (viridis-like)
        pos = [0.0, 0.25, 0.5, 0.75, 1.0]
        color = [[68, 1, 84], [68, 50, 124], [40, 120, 142], [102, 183, 120], [237, 248, 33]]
        cmap = pg.ColorMap(pos, color)
        self.ve_heatmap.setColorMap(cmap)

        cyl_layout = QHBoxLayout()
        cyl_label = QLabel("Number of Cylinders:")
        cyl_label.setStyleSheet("color: #ddd;")
        self.cyl_input = QLineEdit(str(self.num_cyl))
        self.cyl_input.setStyleSheet("background: #444; color: #ddd; border: 1px solid #555; border-radius: 4px;")
        cyl_button = QPushButton("Update")
        cyl_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        cyl_button.clicked.connect(self.update_num_cyl)
        cyl_layout.addWidget(cyl_label)
        cyl_layout.addWidget(self.cyl_input)
        cyl_layout.addWidget(cyl_button)
        ve_layout.addLayout(cyl_layout)
        self.tabs.addTab(ve_tab, "VE Table")

        # Charts Tab with styled plots
        charts_tab = QWidget()
        charts_layout = QVBoxLayout(charts_tab)
        self.sensor_dropdown = QComboBox()
        self.sensor_dropdown.setStyleSheet("background: #444; color: #ddd; border: 1px solid #555; border-radius: 4px;")
        charts_layout.addWidget(self.sensor_dropdown)

        self.plot_widget = pg.PlotWidget(title="Time Series")
        self.plot_widget.setBackground('#333')
        self.plot_curve = self.plot_widget.plot(pen=pg.mkPen('y', width=2))
        charts_layout.addWidget(self.plot_widget)

        self.hist_widget = pg.PlotWidget(title="Histogram")
        self.hist_widget.setBackground('#333')
        self.hist_curve = self.hist_widget.plot(stepMode=True, fillLevel=0, brush=(0,0,255,150))
        charts_layout.addWidget(self.hist_widget)

        self.sensor_dropdown.currentIndexChanged.connect(self.update_chart)
        self.tabs.addTab(charts_tab, "Charts")

        # Log Timing Tab with styled table
        log_timing_tab = QWidget()
        log_timing_layout = QVBoxLayout(log_timing_tab)
        
        # Control buttons with modern style
        controls_layout = QHBoxLayout()
        self.clear_log_button = QPushButton("Clear Log")
        self.clear_log_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.clear_log_button.clicked.connect(self.clear_log_timing)
        self.export_log_button = QPushButton("Export to CSV")
        self.export_log_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
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
        self.log_timing_table.setStyleSheet("""
            QTableWidget { background-color: #333; color: #ddd; gridline-color: #444; }
            QHeaderView::section { background-color: #444; color: white; }
        """)
        log_timing_layout.addWidget(self.log_timing_table)
        
        # Dictionary to store the latest timestamp and sensor readings
        self.current_log_entry_time = None
        self.current_log_sensors = {}
        
        self.tabs.addTab(log_timing_tab, "Log Timing")

        # Logging Tab
        logging_tab = QWidget()
        logging_layout = QVBoxLayout(logging_tab)
        self.log_button = QPushButton("Start Logging")
        self.log_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.log_button.clicked.connect(self.toggle_logging)
        logging_layout.addWidget(self.log_button)
        self.tabs.addTab(logging_tab, "Logging")

    def create_connection_panel(self):
        """Create the connection control panel with modern buttons"""
        panel = QWidget()
        layout = QHBoxLayout(panel)
        
        device_label = QLabel("Device:")
        device_label.setStyleSheet("color: #ddd;")
        layout.addWidget(device_label)
        self.device_dropdown = QComboBox()
        self.device_dropdown.setStyleSheet("background: #444; color: #ddd; border: 1px solid #555; border-radius: 4px;")
        self.device_dropdown.addItem("(Scan for devices...)")
        layout.addWidget(self.device_dropdown)
        
        self.scan_button = QPushButton("Scan Devices")
        self.scan_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.scan_button.clicked.connect(self.scan_devices)
        layout.addWidget(self.scan_button)
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.connect_button.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_button)

        # Test device button to run quick diagnostics without connecting the UI fully
        self.test_button = QPushButton("Test Device")
        self.test_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.test_button.clicked.connect(self.on_test_clicked)
        layout.addWidget(self.test_button)
        
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444); color: white; border: 1px solid #666; border-radius: 4px;")
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button)
        
        layout.addStretch()
        
        return panel

    def scan_devices(self):
        """Scan for available serial/OBD devices using pyserial"""
        self.device_dropdown.clear()
        self.device_dropdown.addItem("(Scanning...)")
        
        try:
            available_ports = [port for port, desc, hwid in comports()]
            self.device_dropdown.clear()
            
            if available_ports:
                self.device_dropdown.addItems(available_ports)
                if 'COM3' in available_ports:
                    self.device_dropdown.setCurrentText('COM3')
                self.status_bar.showMessage(f"Found {len(available_ports)} port(s). Select and click Connect.")
            else:
                self.device_dropdown.addItem("(No serial ports found)")
                self.status_bar.showMessage("No serial ports detected. Check USB connection and drivers.")
        except Exception as e:
            self.device_dropdown.clear()
            self.device_dropdown.addItem(f"Error: {str(e)}")
            self.status_bar.showMessage(f"Scan error: {str(e)}")

    def on_connect_clicked(self):
        """Handle connect button click"""
        port = self.device_dropdown.currentText()
        if not port or port.startswith('('):
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

    def on_test_clicked(self):
        """Run a quick non-blocking device test (in background)"""
        port = self.device_dropdown.currentText()
        if not port or port.startswith('('):
            QMessageBox.warning(self, "No Device", "Please select a device first")
            return
        # Disable test button while running
        self.test_button.setEnabled(False)
        self.status_bar.showMessage(f"Testing {port}...")
        t = threading.Thread(target=self._run_device_test, args=(port,), daemon=True)
        t.start()

    def _run_device_test(self, port: str):
        """Background worker: try to connect and run a few queries to verify vehicle responses."""
        results = []
        try:
            # Try a direct connection (do not disturb existing app connection)
            conn = None
            try:
                conn = obd.OBD(portstr=port, protocol='3', baudrate=None, fast=False, timeout=2)
            except Exception as e:
                results.append((False, f"Failed to open {port}: {e}"))
                conn = None

            if conn and conn.is_connected():
                results.append((True, f"Connected to {port}. Status: {conn.status()}, Protocol: {conn.protocol_id()}"))
                # try a few standard commands
                for cmd in (obd.commands.RPM, obd.commands.SPEED, obd.commands.COOLANT_TEMP):
                    try:
                        rsp = conn.query(cmd, force=True)
                        if rsp and rsp.value is not None:
                            results.append((True, f"{cmd.name}: {rsp.value}"))
                        else:
                            results.append((False, f"{cmd.name}: no data"))
                    except Exception as e:
                        results.append((False, f"{cmd.name}: error {e}"))
                # try GM custom PIDs
                for cmd in self.gm_custom_commands:
                    try:
                        rsp = conn.query(cmd, force=True)
                        if rsp and rsp.value is not None:
                            results.append((True, f"{cmd.name}: {rsp.value}"))
                        else:
                            results.append((False, f"{cmd.name}: no data or not supported"))
                    except Exception as e:
                        results.append((False, f"{cmd.name}: error {e}"))
                conn.close()
            else:
                results.append((False, f"Could not connect to {port}"))
        except Exception as e:
            results.append((False, f"Test error: {e}\n{traceback.format_exc()}"))

        # Re-enable button and show results on UI thread
        def finish():
            self.test_button.setEnabled(True)
            summary = "\n".join([msg for ok, msg in results])
            QMessageBox.information(self, "Device Test Results", summary)
            self.status_bar.showMessage("Test complete")

        QTimer.singleShot(0, finish)

    def connect_obd(self, port):
        """Connect to OBD device using Async and start monitoring"""
        try:
            # Close any existing connection first
            self.disconnect_obd()
            self.status_bar.showMessage(f"Connecting to {port} with J1850 VPW protocol...")
            self.connection = obd.OBD(portstr=port, protocol='3', baudrate=None, fast=False, timeout=2)
            
            if not self.connection.is_connected():
                raise Exception("Connection established but vehicle not responding. Check ignition/OBD port.")
            
            # Get supported mode 01 commands
            self.supported_commands = [cmd for cmd in self.connection.supported_commands if cmd.mode == 1]
            # Add GM custom if supported
            for cmd in self.gm_custom_commands:
                if self.connection.supports(cmd):
                    self.supported_commands.append(cmd)
            logger.debug(f"Supported commands: {[c.name for c in self.supported_commands]}")
            
            if not self.supported_commands:
                self.status_bar.showMessage("Warning: No OBD PIDs available")
            
            # Populate UI
            self.sensor_dropdown.clear()
            self.sensor_dropdown.addItems([cmd.name for cmd in self.supported_commands])
            self.sensors_table.setRowCount(len(self.supported_commands))
            for i, cmd in enumerate(self.supported_commands):
                self.sensors_table.setItem(i, 0, QTableWidgetItem(cmd.name))
            
            self.connected = True
            self.last_port = port
            self.status_bar.showMessage(f"Connected to {port} ✓ Monitoring {len(self.supported_commands)} sensors (Protocol: {self.connection.protocol_id()})")
            self.disconnect_button.setEnabled(True)
            
        except Exception as e:
            error_msg = str(e)
            self.status_bar.showMessage(f"Connection Error: {error_msg[:50]}")
            QMessageBox.warning(
                self, 
                "Connection Error", 
                f"Failed to connect to {port}:\n\n{error_msg}\n\n"
                f"Troubleshooting:\n"
                f"• Is the vehicle ignition ON?\n"
                f"• Is the OBD-II adapter plugged in correctly?\n"
                f"• Try rescanning devices and connecting again\n"
                f"• If protocol error, vehicle may not support J1850 VPW"
            )
            self.connect_button.setEnabled(True)
            self.scan_button.setEnabled(True)
            self.device_dropdown.setEnabled(True)
            self.disconnect_obd()

    def disconnect_obd(self):
        """Disconnect from OBD device and clean up resources"""
        self.connected = False
        try:
            if self.async_connection:
                try:
                    self.async_connection.stop()
                    self.async_connection.close()
                except Exception:
                    pass
                self.async_connection = None

            if self.connection:
                try:
                    self.connection.close()
                except Exception:
                    pass
                self.connection = None
        except:
            pass
        
        self.supported_commands = []
        self.sensor_values = {}
        self.history.clear()

    def reconnect_obd(self):
        if not self.connected and self.last_port:
            self.status_bar.showMessage(f"Attempting reconnect to {self.last_port}...")
            self.connect_obd(self.last_port)

    def update_sensor(self, response):
        try:
            if response.is_null() or response.value is None:
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
            logger.exception("Sensor update error")

    def update_dashboard(self):
        """Update dashboard using latest sensor values"""
        try:
            if not self.connected:
                return
            # If using synchronous connection, poll supported commands for fresh values
            if self.connection and self.supported_commands:
                for cmd in self.supported_commands:
                    try:
                        resp = self.connection.query(cmd, force=True)
                        if resp and not resp.is_null() and resp.value is not None:
                            name = cmd.name
                            value = resp.value
                            self.sensor_values[name] = value
                            self.history[name].append(value)
                            if len(self.history[name]) > 100:
                                self.history[name].pop(0)
                            # Add to log timing
                            self.add_log_timing_entry(name, value)
                    except Exception as e:
                        logger.debug(f"Error polling {cmd.name}: {e}")

            rpm = self.sensor_values.get('RPM', 0)
            speed = self.sensor_values.get('SPEED', 0)
            map_ = self.sensor_values.get('INTAKE_PRESSURE', 0)
            maf = self.sensor_values.get('MAF', 0)
            iat = self.sensor_values.get('INTAKE_TEMP', 0)
            coolant = self.sensor_values.get('COOLANT_TEMP', 0)
            throttle = self.sensor_values.get('THROTTLE_POS', 0)
            timing = self.sensor_values.get('TIMING_ADVANCE', 0)
            o2_b1s1 = self.sensor_values.get('O2_B1S1', 0)
            o2_b1s2 = self.sensor_values.get('O2_B1S2', 0)
            o2_b2s1 = self.sensor_values.get('O2_B2S1', 0)
            o2_b2s2 = self.sensor_values.get('O2_B2S2', 0)
            trans_temp = self.sensor_values.get('TRANS_TEMP', 0)
            oil_temp = self.sensor_values.get('OIL_TEMP', 0)
            oil_pressure = self.sensor_values.get('OIL_PRESSURE', 0)
            gear_position = self.sensor_values.get('GEAR_POSITION', '--')

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
            self.trans_temp_label.setText(f"Trans Temp: {trans_temp} °C")
            self.oil_temp_label.setText(f"Oil Temp: {oil_temp} °C")
            self.oil_pressure_label.setText(f"Oil Pressure: {oil_pressure:.1f} psi")
            self.gear_position_label.setText(f"Gear Position: {gear_position}")

            # VE calculation
            ve = 0
            if rpm > 0 and maf > 0 and map_ > 0 and iat is not None:
                temp_k = iat + 273.15
                intakes_per_sec = rpm * self.num_cyl / 120.0
                g_cyl = maf / intakes_per_sec if intakes_per_sec > 0 else 0
                ve = (g_cyl * temp_k) / map_ if map_ > 0 else 0
                self.ve_label.setText(f"VE: {ve:.4f} g*K/kPa")

                # Populate VE table and heatmap
                map_idx = min(range(len(MAP_BINS)), key=lambda i: abs(MAP_BINS[i] - map_))
                rpm_idx = min(range(len(RPM_BINS)), key=lambda i: abs(RPM_BINS[i] - rpm))
                old_avg = self.ve_data_array[map_idx, rpm_idx]
                count = self.ve_counts[map_idx, rpm_idx]
                new_avg = (old_avg * count + ve) / (count + 1)
                self.ve_data_array[map_idx, rpm_idx] = new_avg
                self.ve_counts[map_idx, rpm_idx] += 1
                item = QTableWidgetItem(f"{new_avg:.4f}")
                # Color cell
                ratio = min(new_avg / 1.5, 1.0)
                red = int(255 * ratio)
                green = int(255 * (1 - ratio))
                item.setBackground(QColor(red, green, 0))
                self.ve_table.setItem(map_idx, rpm_idx, item)
                # Update heatmap
                self.ve_heatmap.setImage(self.ve_data_array.T)

            # Update sensors table
            for i, cmd in enumerate(self.supported_commands):
                value = self.sensor_values.get(cmd.name, "--")
                self.sensors_table.setItem(i, 1, QTableWidgetItem(str(value)))

            # Logging
            if self.logging and self.log_writer:
                timestamp = time.time()
                row = [timestamp, rpm, map_, maf, iat, ve, speed, coolant, throttle, timing, o2_b1s1, o2_b1s2, o2_b2s1, o2_b2s2, trans_temp, oil_temp, oil_pressure, gear_position]
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
                self.log_writer.writerow(['Timestamp', 'RPM', 'MAP', 'MAF', 'IAT', 'VE', 'Speed', 'Coolant', 'Throttle', 'Timing', 'O2_B1S1', 'O2_B1S2', 'O2_B2S1', 'O2_B2S2', 'Trans_Temp', 'Oil_Temp', 'Oil_Pressure', 'Gear_Position'])
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
                    self._write_log_line(self.current_log_entry_time, dict(self.current_log_sensors))
                
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
        """Clean up all resources before closing"""
        try:
            self.disconnect_obd()
        except:
            pass
        
        try:
            if self.log_file:
                self.log_file.close()
        except:
            pass
        
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Apply polished dark theme
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(35, 35, 35))
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(40, 40, 40))
    palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Button, QColor(45, 45, 45))
    palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(palette)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec())