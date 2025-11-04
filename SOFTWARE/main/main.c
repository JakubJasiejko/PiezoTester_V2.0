#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "uart.h"
#include "i2c.h"
#include "ads1219.h"

// Jeśli masz driver z BC547 + pull-up do 12V (czyli LOW = ON):
#define MOSFET_PIN GPIO_NUM_17
#define MOSFET_ON_LEVEL   0
#define MOSFET_OFF_LEVEL  1

static float ads1219_measure(uint8_t mux_sel, uint8_t gain_sel, float vref)
{
    ads1219_configureMeasurement(
        ADS1219_ADRESS,
        mux_sel,
        gain_sel,
        ADS1219_DATA_RATE_20SPS,
        ADS1219_CONV_MODE_SINGLE,
        ADS1219_VREF_INTERNAL
    );

    ads1219_startSync(ADS1219_ADRESS);
    int32_t raw = ads1219_read(ADS1219_ADRESS);
    return (raw / 8388608.0f) * vref;  // 24-bit ±Vref -> 2^23
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MOSFET_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(MOSFET_PIN, MOSFET_OFF_LEVEL); // elektromagnes w górze

    initUART();
    initI2C();
    ADS1219_init(ADS1219_RST_PIN, ADS1219_DRDY_PIN);
    gpio_set_level(ADS1219_RST_PIN, 1);

    const float vref = 2.048f;
    uint32_t nr = 0;

    while (1) {

        // 1️⃣ Elektromagnes w dół
        gpio_set_level(MOSFET_PIN, MOSFET_ON_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 2️⃣ Pomiary:
        float v_single2 = ads1219_measure(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref);
        float v_diff01  = ads1219_measure(ADS1219_MEAS_DIFF_01,  ADS1219_GAIN_1, vref);

        // 3️⃣ Wysyłanie danych (1 s)
        char msg[96];
        int len = snprintf(msg, sizeof(msg),
                           "%lu %.6f %.6f\r\n",
                           (unsigned long)nr, v_single2, v_diff01);
        uart_write_bytes(UART_PORT, msg, len);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 4️⃣ Elektromagnes w górę
        gpio_set_level(MOSFET_PIN, MOSFET_OFF_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(2000));

        nr++;
    }
}
