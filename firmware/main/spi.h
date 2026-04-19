#ifndef SPI_H
#define SPI_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>

/**
 * @file spi.h
 * @brief Lightweight SPI helper layer used by optional board-support devices.
 */

#define SPI_HOST       HSPI_HOST
#define SPI_CLK_SPEED  1000000  // 1 MHz

#define SPI_MOSI_PIN    GPIO_NUM_19
#define SPI_MISO_PIN    GPIO_NUM_21
#define SPI_SCLK_PIN    GPIO_NUM_18

#define ADS1263_CS_PIN GPIO_NUM_5
#define ADS1263_DRDY_PIN GPIO_NUM_22
#define ADS1263_START_PIN GPIO_NUM_17
#define ADS1263_RESET_PIN GPIO_NUM_23

#define MAX_SPI_DEVICES 4

/** @brief Shared table of SPI device handles. */
extern spi_device_handle_t spi_handles[MAX_SPI_DEVICES];

/** @brief Initializes the SPI bus without binding a specific chip-select line. */
uint8_t spi_init(gpio_num_t mosi, gpio_num_t miso, gpio_num_t sclk);
/** @brief Registers an SPI device on the shared bus. */
uint8_t addSpiDevice(gpio_num_t cs_pin, int index);
/** @brief Sends one byte over SPI. */
uint8_t spi_write(uint8_t* data, int index);
/** @brief Sends three bytes over SPI. */
uint8_t spi_write24(uint8_t* data, int index);
/** @brief Performs a full-duplex SPI transfer. */
uint8_t spi_read(uint8_t* tx_data, uint8_t* rx_data, size_t length, int index);
/** @brief Sends a raw ADS1263 command. */
uint8_t ads1263_sendCommand(uint8_t cmd, int index);
/** @brief Writes an ADS1263 register. */
uint8_t ads1263_writeReg(uint8_t reg, uint8_t data, int index);
/** @brief Reads an ADS1263 register using a 24-bit transaction. */
uint32_t ads1263_readReg(uint8_t reg, uint8_t index);
/** @brief Sends two bytes over SPI. */
uint8_t spi_write16(uint8_t* data, int index);

#endif // SPI_H
