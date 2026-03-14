#include "spi.h"
#include <string.h>

#define ADS1263_WREG 0x40
#define ADS1263_RREG 0x20

spi_device_handle_t spi_handles[MAX_SPI_DEVICES];


uint8_t spi_init(gpio_num_t mosi, gpio_num_t miso, gpio_num_t sclk) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    return (spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) ? 0x00 : 0xFF;
}

uint8_t addSpiDevice(gpio_num_t cs_pin, int index) {
    if (index >= MAX_SPI_DEVICES) return 0xFF;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_SPEED,
        .mode = 0,
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };

    return (spi_bus_add_device(SPI_HOST, &devcfg, &spi_handles[index]) == ESP_OK) ? 0x00 : 0xFF;
}

uint8_t spi_write(uint8_t* data, int index) {
    spi_transaction_t t = {
        .length = 8 * sizeof(uint8_t),  // Zakładamy 1 bajt
        .tx_buffer = data,
        .rx_buffer = NULL,
    };
    return (spi_device_transmit(spi_handles[index], &t) == ESP_OK) ? 0x00 : 0xFF;
}

uint8_t spi_write24(uint8_t* data, int index) {
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = data,
        .rx_buffer = NULL,
    };
    return (spi_device_transmit(spi_handles[index], &t) == ESP_OK) ? 0x00 : 0xFF;
}

uint8_t spi_write16(uint8_t* data, int index) {
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = data,
        .rx_buffer = NULL,
    };
    return (spi_device_transmit(spi_handles[index], &t) == ESP_OK) ? 0x00 : 0xFF;
}

uint8_t spi_read(uint8_t* tx_data, uint8_t* rx_data, size_t length, int index) {
    spi_transaction_t t = {
        .length = length * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return (spi_device_transmit(spi_handles[index], &t) == ESP_OK) ? 0x00 : 0xFF;
}

uint32_t ads1263_readReg(uint8_t reg, uint8_t index) {
    uint8_t tx[3] = { ADS1263_RREG | reg , 0x00, 0xFF };
    uint8_t rx[3] = { 0 };

    if (spi_read(tx, rx, 3, index) != 0x00)
        return 0xFFFFFFFF;

    uint32_t value = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    return value;
}

uint8_t ads1263_writeReg(uint8_t reg, uint8_t data, int index) {
    uint8_t cmd[3];
    cmd[0] = ADS1263_WREG | reg ;  // WREG z adresem rejestru
    cmd[1] = 0x00;                         // tylko jeden bajt do zapisu
    cmd[2] = data;

    return spi_write24(cmd, index);
}


uint8_t ads1263_sendCommand(uint8_t cmd, int index) {
    return spi_write(&cmd, index);  // cmd jako wskaźnik
}




