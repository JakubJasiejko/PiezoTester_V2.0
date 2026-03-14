#include "adg731.h"
#include "spi.h"
//#include "config.h"

// ADG731 odbiera 8-bitowe słowo: EN, CS, X, A4..A0 (MSB->LSB).
// Dane są próbkowane na opadającym zboczu SCLK (SPI mode 1).

uint8_t adg731_init(gpio_num_t cs_pin, int index) {
    // Dodaj urządzenie do magistrali (mode=1 masz w addSpiDevice)
    if (addSpiDevice(cs_pin, index) != 0x00) {
        return 0xFF;
    }

    // All OFF: EN=1 -> 1000 0000
    uint8_t off = 0x80;
    return spi_write(&off, index);   // ✔️ wskaźnik
}

uint8_t adg731_setChannel(uint8_t channel, int index) {
    if (channel > 31) {
        return 0xFF;                 // ✔️ pilnujemy zakresu 0..31
    }

    uint8_t cmd = (uint8_t)(channel & 0x1F);  // ✔️ tylko A4..A0
    return spi_write(&cmd, index);            // ✔️ wskaźnik
}

uint8_t adg731_allOff(int index) {
    uint8_t off = 0x80; // EN=1
    return spi_write(&off, index);
}
