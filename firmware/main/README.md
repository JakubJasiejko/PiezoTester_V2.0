# Firmware Main Module

This folder contains the application-layer firmware sources for PiezoTester V2.0.

## File Responsibilities

- `main.c`
  High-level application logic, calibration workflow, test execution, UART command handling, actuator learning, and NVS persistence
- `ads1219.c/h`
  ADS1219 ADC transport and conversion control
- `i2c.c/h`
  I2C bus helper layer
- `uart.c/h`
  UART helper layer
- `spi.c/h`
  SPI helper layer used by board support code
- `adg731.c/h`
  ADG731 switching helper

## Documentation Style

This folder is being annotated with Doxygen-style comments so the firmware can later be published as browsable API / implementation documentation.
