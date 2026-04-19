# Third-Party Sources and Dependency Notes

This document records the main external building blocks used in PiezoTester V2.0 so the repository stays traceable and publication-friendly.

## Firmware Dependencies

### ESP-IDF

- Role: primary embedded framework for ESP32 firmware
- Usage: GPIO, LEDC PWM, UART, I2C, SPI, NVS, timing, FreeRTOS integration
- Project location: `firmware/`

The firmware currently uses ESP-IDF-native drivers and in-repo application code. There are no separate Git submodules for third-party C libraries in active use at the moment.

## Desktop Application Dependencies

### PySide6

- Role: GUI framework for charts, dialogs, tables, PDF generation, and plotting UI
- Project location: `desktop_app/main.py`

### pyserial

- Role: serial communication with the ESP32 firmware
- Project location: `desktop_app/main.py`

## PCB Libraries and Model Sources

### SamacSys-generated libraries

Several local KiCad symbol / footprint libraries clearly identify themselves with:

- `generator SamacSys_ECAD_Model`

These are used in multiple board revisions and vendor-part library folders, including parts such as:

- LM358P
- MCP6002-I/P
- LM2596S-5.0/NOPB
- LP2950-33LPRE3
- ADS1219IPWR
- CP2102N
- ESP32-WROOM module libraries

These assets are kept locally so the PCB project remains portable and self-contained.

### SparkFun ADS1219 reference material

The repository also uses ADS1219-related reference geometry derived from the SparkFun Qwiic ADC ADS1219 board ecosystem for modeling / representation purposes.

### Espressif reference assets

ESP32-related 3D and module assets were aligned using Espressif reference materials and repository-local model files where required.

### Local project-specific assets

Some 3D files, symbol libraries, and KiCad project libraries were authored or adjusted directly inside the project so that:

- board renders open without missing 3D files
- library dependencies remain repository-local
- revision snapshots stay reproducible

## CAD Sources

The mechanical CAD folder primarily contains project-authored SolidWorks models plus exchange-friendly STEP exports created during device design and fixture iteration.

## Fabrication / Production Outputs

Some PCB revisions contain fabrication-ready exports such as:

- Gerber archives
- drill files
- JLCPCB BOM / CPL outputs

Those files are treated as build artifacts for manufacturing, not as the canonical source. The canonical source remains the KiCad project for the relevant board revision.
