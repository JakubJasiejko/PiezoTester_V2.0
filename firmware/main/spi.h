#ifndef SPI_H
#define SPI_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>

//szajs wyjebać do smieci bo to zapierdolone gówno
// SPI pins
#define SPI_HOST       HSPI_HOST
#define SPI_CLK_SPEED  1000000  // 1 MHz

#define SPI_MOSI_PIN    GPIO_NUM_19
#define SPI_MISO_PIN    GPIO_NUM_21
#define SPI_SCLK_PIN    GPIO_NUM_18


//#define ADS1263_RREG 0x20
//#define ADS1263_WREG 0x40
//#define ADS1263_REG_ID 0x00

#define ADS1263_CS_PIN GPIO_NUM_5
#define ADS1263_DRDY_PIN GPIO_NUM_22
#define ADS1263_START_PIN GPIO_NUM_17
#define ADS1263_RESET_PIN GPIO_NUM_23


#define MAX_SPI_DEVICES 4   // Możesz zmienić według potrzeb

extern spi_device_handle_t spi_handles[MAX_SPI_DEVICES];  // Tablica uchwytów SPI

// Inicjalizacja magistrali SPI (bez CS)
uint8_t spi_init(gpio_num_t mosi, gpio_num_t miso, gpio_num_t sclk);

// Dodanie nowego urządzenia SPI z danym pinem CS i indeksem uchwytu
uint8_t addSpiDevice(gpio_num_t cs_pin, int index);

// Wysyła 1 bajt przez SPI
uint8_t spi_write(uint8_t* data, int index);

// Wysyła 3 bajty (24 bity)
uint8_t spi_write24(uint8_t* data, int index);

// Odczyt danych z urządzenia SPI
uint8_t spi_read(uint8_t* tx_data, uint8_t* rx_data, size_t length, int index);

// Wysyła komendę do ADS1263 (np. START, RESET)
uint8_t ads1263_sendCommand(uint8_t cmd, int index);

// Zapisuje do rejestru ADS1263 (WREG)
uint8_t ads1263_writeReg(uint8_t reg, uint8_t data, int index);

// Czyta 3 bajty z rejestru (RREG + 2 dummy bajty)
uint32_t ads1263_readReg(uint8_t reg, uint8_t index);

uint8_t spi_write16(uint8_t* data, int index);

#endif // SPI_H
