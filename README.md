MY_TOOL — Advanced OBD-II Dashboard for 5.3 Vortec V8
======================================================

A professional PyQt6-based OBD-II live monitoring dashboard with real-time gauges, VE table visualization, and comprehensive data logging. Features a modern dark theme inspired by OBD dash.pro with a floating action button for quick access to tools.

**Features**
- **Professional Dashboard:** Large central RPM gauge (280×280) surrounded by 8 compact sensor gauges (140×160 each) with segmented color arcs (green → yellow → red)
- **Textured Background:** Subtle carbon-fiber-like texture effect on dashboard for enhanced visual depth
- **Real-time Gauges:** RPM, Speed, MAP, Coolant Temp, IAT, Throttle Position, Fuel System Status, Spark Advance, Load, O2 Sensors
- **VE Table Heatmap:** Volumetric efficiency matrix across RPM/MAP bins with color-coded cells
- **Data Logging:** CSV export of sensor values with timestamp grouping
- **Time Series Charts:** Interactive pyqtgraph plots with histogram analysis
- **FAB Button:** Quick access menu with connection status, clear logs, export data, and reset VE table functions
- **Dark Theme:** Modern dark card-style UI with cobalt blue accents matching automotive dashboards

**Quick Start**
- **Create & activate venv (PowerShell):**

```
python -m venv venv
.\venv\Scripts\Activate.ps1
```

- **Install dependencies:**

```
pip install -r requirements.txt
```

- **Run the app:**

```
python main.py
```

**UI Components & Usage**

- **Connection Panel:** 
  - Scan Devices button to find available COM ports
  - Device dropdown selector
  - Connect/Disconnect buttons for OBD adapter
  - Test Device button for non-blocking connectivity verification

- **Dashboard Tab (Main View):**
  - **Central RPM Gauge:** Large 280×280 gauge showing engine RPM (0–8000) with segmented color zones
  - **Compact Peripheral Gauges:** 8 surrounding gauges for Speed, MAP, Coolant, Intake Air Temp, Throttle, Fuel System, Spark Advance, Load, and O2 sensors
  - **Sensor Info Panel:** Right sidebar showing current values for MAP, MAF, IAT, Coolant, Throttle, VE, Ignition Timing, O2 sensors, transmission temp, oil temp/pressure, and gear position
  - **Floating Action Button (FAB):** Blue ⊙ button in bottom-right corner opens quick actions menu:
    - **Connection Status:** Display OBD protocol, status, and port info
    - **Disconnect OBD:** Quick disconnect (shown only when connected)
    - **Clear All Logs:** Remove all logged sensor data
    - **Export Current Data:** Save sensor readings to CSV file
    - **Reset VE Table:** Reset volumetric efficiency to default (100.0)

- **Sensors Tab:** Table of all OBD commands supported by your vehicle with real-time values

- **VE Table Tab:** Interactive heatmap showing volumetric efficiency (g*K/kPa) across RPM and MAP bins. Color intensity represents efficiency ratio.

- **Charts Tab:** 
  - Sensor dropdown selector
  - Time-series plot of selected sensor values
  - Histogram with frequency distribution
  - Refresh and export buttons

- **Log Timing Tab:** Grouped sensor snapshots organized by timestamp with export to CSV and clear button

- **Logging Tab:** Toggle background CSV logging of key parameters (RPM, MAP, MAF, IAT, VE, Speed, Coolant, Throttle, Timing, O2 sensors)

**Technical Details**

**Connection Model**
- **Synchronous Polling:** Timer-driven polling (default 500ms interval) using `obd.OBD()` ensures reliable reads and avoids port locking issues
- **Protocol Support:** Defaults to J1850 VPW (protocol='3'), adjustable for other protocols (e.g., ISO 14230-4 KWP)
- **Supported Commands:** On connect, queries device for supported mode-01 commands and only polls those, reducing traffic and improving reliability
- **Debug Logging:** All events logged to `my_tool_debug.log` (created in app directory on first run)

**Gauge System**
- **CircularGauge Widget** (enhanced_gauge.py):
  - Supports two modes: full-size (280×280) and compact (140×160)
  - Full mode: Background gradient, tick marks with labels, segmented color arcs, centered value text, needle with center cap
  - Compact mode: Thin arc (8px), title label below, minimal font for space efficiency
  - Customizable thresholds: yellow zone at 60% by default, red zone at 85% (adjustable per gauge)
  - Real-time value updates via `set_value()` method

**Color Scheme**
- **Background:** Dark charcoal (#0f0f10) with subtle noise texture
- **Cards:** Gradient dark backgrounds (#1b1b1e to #111113) with 1px borders
- **Accents:** Cobalt blue (#2a7bd6) for primary actions (FAB, selected tabs)
- **Gauge Zones:** Teal/cyan (safe), Yellow (caution), Red (critical)
- **Text:** Light gray (#ddd–#e6e6e6) for contrast on dark backgrounds

**Building a Windows executable (.exe)**
- Use PyInstaller to create a single-file, windowed executable:

```
pyinstaller --noconfirm --onefile --windowed --name MY_TOOL main.py
```

- Result: `dist\MY_TOOL.exe` (build artifacts in `build\` and `dist\`).

Notes & common build issues:
- PyInstaller sometimes warns about `pyqtgraph.opengl` because `PyOpenGL` may be missing. If you use OpenGL features or want to silence that warning, install:

```
pip install PyOpenGL
```

- Qt plugins: PyInstaller's PyQt6 hook should bundle the Qt runtime, but if the built exe fails due to missing Qt platform plugins, you can add `--add-data` entries to include the `PyQt6\Qt6\plugins` folder or adjust the generated `.spec` to include those directories.
- If the exe fails with GUI/DLL issues, run the exe from a console to capture stderr and share the output.

**Troubleshooting**

**Connection Issues**
- If no OBD adapters are found:
  - Verify the adapter is connected and recognized by the OS (check Device Manager for a COM port)
  - Ensure the vehicle ignition is **ON**
  - On Windows, admin privileges may be needed to access serial ports
- If "Test Device" shows connection but Dashboard doesn't update:
  - Check `my_tool_debug.log` for polling errors or unsupported PIDs
  - Try adjusting the OBD protocol (e.g., protocol='2' for ISO 14230-4 KWP)

**Data Display**
- If data shows `--` after connecting, that PID may not be supported by your vehicle or requires a specific ignition state
- Some vehicles only respond to certain PIDs when running or at idle
- Check the Sensors tab to see which commands are actually supported

**GUI Issues**
- If `main.py` fails to import `PyQt6`, run: `pip install PyQt6`
- If the dashboard doesn't render properly, check for Qt platform plugin errors
- Run from console to capture detailed error output
- Verify that enhanced_gauge.py is in the same directory as main.py

**Performance Tuning**
- If polling seems slow, increase `UPDATE_INTERVAL_MS` (e.g., 800ms)
- If adapters freeze, decrease `UPDATE_INTERVAL_MS` and check for timeouts in `my_tool_debug.log`
- On slower systems, consider disabling the textured background (set dashboard background to solid color)

**Development & Diagnostics**
- Check `my_tool_debug.log` for detailed connection and polling output
- Use the "Test Device" button to verify adapter connectivity without connecting the dashboard
- The FAB menu provides quick access to connection status, data export, and log clearing
- All errors are logged with timestamps for debugging

**Files of Interest**
- `main.py` — Main application window, OBD polling, UI setup, VE table, data logging (~1300 lines)
- `enhanced_gauge.py` — Custom CircularGauge widget with full/compact modes and segmented color arcs (~170 lines)
- `requirements.txt` — Python package dependencies
- `MY_TOOL.spec` — PyInstaller spec file (generated during build)
- `dist/MY_TOOL.exe` — Windows executable (created via PyInstaller)
- `my_tool_debug.log` — Debug log with connection and polling details (created on first run)
- `vehicle_log.csv` — Optional CSV data log (created when logging is enabled)

**Advanced Configuration**
- **Polling Interval:** Change `UPDATE_INTERVAL_MS` in `main.py` (default 500ms). Decrease for faster updates but higher adapter load; increase for slower sampling.
- **Number of Cylinders:** Set via UI "Number of cylinders" field (default 8); used in VE calculations.
- **OBD Protocol:** Default is protocol='3' (J1850 VPW). Change in `connect_obd()` method for different vehicles.
- **Gauge Thresholds:** Customize yellow/red zones when creating CircularGauge instances:
  ```python
  gauge = CircularGauge(min_val, max_val, yellow_threshold=0.6, red_threshold=0.85)
  ```
- **Texture Intensity:** Adjust pattern intensity in `create_dashboard_textured_background()` method (default 0.1 = 10% noise)

**Building a Windows Executable**

Use PyInstaller to create a single-file windowed executable:

```
pyinstaller --noconfirm --onefile --windowed --name MY_TOOL main.py
```

Result: `dist\MY_TOOL.exe` (build artifacts in `build\` and `dist\` directories)

**Build Troubleshooting:**
- **PyOpenGL Warning:** If PyInstaller warns about missing `PyOpenGL`, install it: `pip install PyOpenGL`
- **Qt Platform Plugins:** If the exe fails with missing Qt plugins, add `--add-data` entries to include `PyQt6\Qt6\plugins` or adjust the `.spec` file
- **GUI/DLL Issues:** Run exe from console to capture stderr output for debugging

