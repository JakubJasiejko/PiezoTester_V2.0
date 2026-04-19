# Desktop Application

This directory contains the host-side control software for PiezoTester V2.0. The application is written in Python with PySide6 and acts as the main user interface for calibration, live monitoring, test execution, charting, and report export.

## Features

- Serial-device discovery and connection handling
- Live mass preview during calibration
- Standard, ramp, and hysteresis test control
- Electromagnet advanced calibration workflows
- Interactive result tables and multi-chart views
- CSV export for raw session data
- PDF export for polished experiment summaries
- Optional operator notes included in the generated report

## Runtime Dependencies

- Python 3.11+
- PySide6
- pyserial

Install the dependencies with:

```bash
cd desktop_app
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Launch

```bash
cd desktop_app
python3 main.py
```

## Important Subfolders

- `logs/`
  Local session outputs generated during development and bench testing.
- `.venv/`
  Local virtual environment, intentionally ignored by Git.
- `.idea/`
  Local IDE metadata, intentionally ignored by Git.

## Notes

- The application expects the ESP32 firmware UART protocol defined in `firmware/main/main.c`.
- Exported reports are intentionally stored as regular files so they can be attached to experiments, papers, and manufacturing validation records.
