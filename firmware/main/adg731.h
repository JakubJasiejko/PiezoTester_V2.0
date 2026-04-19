#pragma once

#include <stdint.h>
#include "driver/gpio.h"

/**
 * @file adg731.h
 * @brief Minimal ADG731 interface used by the PiezoTester firmware.
 */

/**
 * @brief Initializes an ADG731 device on the shared SPI bus.
 *
 * @param cs_pin Chip-select GPIO assigned to this device.
 * @param index Slot index inside the shared SPI handle table.
 * @return 0x00 on success, 0xFF on error.
 */
uint8_t adg731_init(gpio_num_t cs_pin, int index);

/**
 * @brief Selects the active ADG731 channel.
 *
 * @param channel Channel in the range 1..32. Value 0 disables all channels.
 * @param index Slot index inside the shared SPI handle table.
 * @return 0x00 on success, 0xFF on error.
 */
uint8_t adg731_setChannel(uint8_t channel, int index);

/**
 * @brief Disables all ADG731 channels.
 *
 * @param index Slot index inside the shared SPI handle table.
 * @return 0x00 on success, 0xFF on error.
 */
uint8_t adg731_allOff(int index);
