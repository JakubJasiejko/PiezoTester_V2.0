# Firmware

This directory contains the ESP32 firmware for PiezoTester V2.0. It is built with ESP-IDF and implements the real-time measurement, calibration, electromagnet control, session logic, and UART command interface used by the desktop application.

## What This Firmware Does

- Reads the ADS1219-based load path
- Reconstructs piezoresistive sensor resistance
- Calibrates the load cell and the sensor path
- Drives the electromagnet through PWM
- Learns an empirical PWM-to-load model
- Executes standard, ramp, and hysteresis tests
- Stores calibration data and learned model points in NVS
- Streams results and status to the host application over UART

## Key Capabilities

- Load preview with automatic tare initialization
- One-point load-cell calibration with nominal-model correction
- Reference-resistor calibration for the sensor measurement path
- Contact-threshold and movement-threshold learning for the electromagnet
- Adaptive ramp targeting with feedforward plus bounded correction
- Hysteresis trace acquisition at high sample rate
- Export-friendly UART protocol for the desktop application

## Main Source Files

```text
firmware/
├── CMakeLists.txt
├── sdkconfig
└── main/
    ├── main.c        Application logic, test controller, calibration, learning
    ├── ads1219.c/h   ADS1219 ADC driver
    ├── i2c.c/h       I2C transport helpers
    ├── uart.c/h      UART transport helpers
    ├── spi.c/h       SPI helpers used by board support code
    └── adg731.c/h    ADG731 channel switch helper
```

## Build

```bash
cd firmware
idf.py build
```

## Flash

```bash
cd firmware
idf.py -p /dev/ttyUSB0 flash monitor
```

Adjust the serial port to match your machine.

## Firmware Architecture

The firmware centers around three coupled subsystems:

1. Measurement subsystem
   Captures load and sensor-path voltages, converts them to calibrated engineering units, and returns structured test results.

2. Calibration and learning subsystem
   Maintains load-cell calibration, sensor-path calibration, electromagnet thresholds, and the learned PWM-to-load map in NVS.

3. Test execution subsystem
   Runs measurement sessions in standard, ramp, and hysteresis modes while keeping the UART interface deterministic for the desktop application.

## Mathematical / Algorithmic Notes

- Load signal is computed from the absolute difference between zero and measured voltage.
- Load-cell calibration uses a nominal physical model corrected by a one-point scaling factor.
- Sensor resistance is reconstructed from divider voltage and corrected by a calibration factor.
- Ramp control uses:
  - a feedforward estimate from the learned actuator model
  - an iterative correction loop with bounded PWM step size
- Hysteresis acquisition uses burst-style high-rate sampling across loading and unloading phases.

## Documentation Quality

The firmware headers and main application source are being annotated with Doxygen-style comments so the codebase can be turned into browsable technical documentation later.

## Notes About Legacy / Generated Files

This repository has been actively used during hardware bring-up. Some generated or legacy files still exist for traceability, but they are now documented and increasingly fenced off by `.gitignore` rules and folder-level README files.
