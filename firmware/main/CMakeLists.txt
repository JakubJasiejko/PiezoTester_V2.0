idf_component_register(
    SRCS "adg731.c" "spi.c" "uart.c" "i2c.c" "ads1219.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash
)
