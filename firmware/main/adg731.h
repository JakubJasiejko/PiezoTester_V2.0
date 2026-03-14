#pragma once
#include <stdint.h>
#include "driver/gpio.h"

// Zwraca 0x00 gdy OK, 0xFF w razie błędu.
// index – indeks w tablicy spi_handles[] z Twojej biblioteki
uint8_t adg731_init(gpio_num_t cs_pin, int index);

// Ustawia aktywny kanał (1..32).
// channel = 0 -> wszystkie OFF (global disable)
// Zwraca 0x00 gdy OK, 0xFF w razie błędu.
uint8_t adg731_setChannel(uint8_t channel, int index);

uint8_t adg731_allOff(int index);
