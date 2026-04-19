/**
 * @file ads1219.h
 * @brief ADS1219 ADC helper interface for the ESP32-based PiezoTester firmware.
 */

#ifndef ADS1219_H
#define ADS1219_H

#include <stdint.h>
#include "driver/gpio.h"

#define ADS1219_DRDY_PIN GPIO_NUM_22
#define ADS1219_RST_PIN GPIO_NUM_5
#define ADS1219_ADRESS 0x40

#define ADS1219_MEAS_DIFF_01  0b000
#define ADS1219_MEAS_DIFF_23  0b001
#define ADS1219_MEAS_DIFF_12  0b010
#define ADS1219_MEAS_SINGLE_0  0b011
#define ADS1219_MEAS_SINGLE_1  0b100
#define ADS1219_MEAS_SINGLE_2  0b101
#define ADS1219_MEAS_SINGLE_3  0b110
#define ADS1219_MEAS_REF 0b111

#define ADS1219_GAIN_1 0
#define ADS1219_GAIN_4 1

#define ADS1219_DATA_RATE_20SPS 0b00
#define ADS1219_DATA_RATE_90SPS 0b01
#define ADS1219_DATA_RATE_330SPS 0b10
#define ADS1219_DATA_RATE_1000SPS 0b11

#define ADS1219_CONV_MODE_SINGLE 0
#define ADS1219_CONV_MODE_CONTINOUS 1

#define ADS1219_VREF_INTERNAL 0
#define ADS1219_VREF_EXTERNAL 1

#define WREG 0x40
#define RREG 0x20

/** @brief Initializes GPIO resources used by the ADS1219 device. */
void ADS1219_init(gpio_num_t rstPin, gpio_num_t drdyPin);
/** @brief Performs a hardware reset pulse on the ADC. */
void ads1219_hardReset(gpio_num_t resetPin);
/** @brief Sends the ADS1219 software-reset command. */
uint8_t ads1219_softReset(uint8_t address);
/** @brief Starts or synchronizes a conversion. */
uint8_t ads1219_startSync(uint8_t address);
/** @brief Places the ADC into power-down mode. */
uint8_t ads1219_powerDown(uint8_t address);
/** @brief Triggers a data read transaction. */
uint8_t ads1219_readDataTrigger(uint8_t address);
/** @brief Writes the ADS1219 configuration byte. */
uint8_t ads1219_configureMeasurement(uint8_t address, uint8_t input, uint8_t gain, uint8_t rate, uint8_t mode, uint8_t reference);
/** @brief Returns true when the DRDY interrupt has been observed. */
bool ads1219_data_ready();
/** @brief Reads a 24-bit conversion result from the ADC. */
int32_t ads1219_read(uint8_t address);

#endif // ADS1219_H
