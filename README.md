MY_TOOL — ELM327 OBD-II Live Monitor
=====================================

A lightweight Qt-based OBD-II live monitoring tool that works with ELM327 adapters (USB/Bluetooth/WiFi).

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

**UI & Usage**
- **Connection panel:** Use the "Scan Devices" button to find available serial/COM devices, select one from the dropdown, then click "Connect." Use "Disconnect" to stop.
- **Tabs:** Dashboard, Sensors, VE Table, Charts, Log Timing and Logging.
- **Log Timing:** Shows grouped sensor snapshots (one line per timestamp) and supports clearing or exporting to CSV.

**Building a Windows executable (.exe)**
- I used PyInstaller to create a single-file, windowed executable:

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
- If `main.py` fails to import `PyQt6`, install it with `pip install PyQt6`.
- If there are no OBD-II adapters found, verify the adapter is connected and drivers installed; check Device Manager for a COM port. On Windows you may need to run with appropriate permissions to access serial ports.
- If some sensors show `--` or don't update, that PID may not be supported by the vehicle or the adapter/connection settings (baud, ignition on).

**Files of interest**
- `main.py` — main application
- `requirements.txt` — Python dependencies
- `MY_TOOL.spec` — PyInstaller spec (generated when building)
- `dist\MY_TOOL.exe` — produced executable after a successful build

If you want, I can:
- Run the produced `dist\MY_TOOL.exe` and capture any runtime errors for debugging.
- Update the PyInstaller `.spec` to include OpenGL/Qt plugin folders and rebuild.
- Create a small installer (NSIS) wrapping the exe.

