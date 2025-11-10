/**
 * @file ads1219.c
 * @author inż. Jakub Jasiejko
 * @date 2025-05-29
 * @brief Obsługa przetwornika ADC ADS1219 poprzez magistralę I2C.
 *
 * @details
 * Biblioteka implementuje podstawowe funkcje do obsługi ADS1219,
 * w tym inicjalizację, konfigurację, odczyt danych, obsługę resetu
 * sprzętowego i programowego, a także zarządzanie pinem DRDY (przerwania).
 *
 * Przeznaczona do współpracy z ESP32, zintegrowana z FreeRTOS.
 */

#include "ads1219.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "i2c.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

bool drdy_flag = false;

static void IRAM_ATTR drdy_isr_handler(void* arg) {
    drdy_flag = true;
}

bool ads1219_data_ready() {
    if (drdy_flag) {
        drdy_flag = false;
        return true;
    }
    return false;
}

void ADS1219_init(gpio_num_t rstPin, gpio_num_t drdyPin) {
    gpio_set_direction(rstPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(drdyPin, GPIO_MODE_INPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << drdyPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(drdyPin, drdy_isr_handler, NULL);
}

void ads1219_hardReset(gpio_num_t resetPin) {
    gpio_set_level(resetPin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(resetPin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(resetPin, 1);
}

uint8_t ads1219_softReset(uint8_t address) {
    return i2c_send_command(address, 0x06);
}

uint8_t ads1219_startSync(uint8_t address) {
    return i2c_send_command(address, 0x08);
}

uint8_t ads1219_powerDown(uint8_t address) {
    return i2c_send_command(address, 0x02);
}

uint8_t ads1219_readDataTrigger(uint8_t address) {
    return i2c_send_command(address, 0x10);
}

uint8_t ads1219_configureMeasurement(uint8_t address, uint8_t input, uint8_t gain, uint8_t rate, uint8_t mode, uint8_t reference) {
    uint8_t configData = (input << 5) | (gain << 4) | (rate << 2) | (mode << 1) | reference;
    return i2c_write_byte(address, WREG, configData);
}

int32_t ads1219_read(uint8_t address) {
    //isr
    while (!ads1219_data_ready()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (ads1219_readDataTrigger(address) == 0xFF) {
        return 0x7FFFFFFF;
    }

    int32_t result = i2c_read24(address);
    return (result == 0x7FFFFFFF) ? 0x7FFFFFFF : result;
}








