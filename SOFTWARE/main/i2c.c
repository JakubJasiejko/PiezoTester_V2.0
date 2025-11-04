/**
 * @file i2c.c
 * @author inż. Jakub Jasiejko
 * @date 2025-03-30
 * @brief Implementacja podstawowej obsługi magistrali I2C w trybie master.
 *
 * @details
 * Plik zawiera zestaw funkcji umożliwiających komunikację z urządzeniami I2C
 * w trybie master przy użyciu ESP32. Implementuje funkcje inicjalizacji
 * magistrali, odczytu i zapisu pojedynczych bajtów oraz 16-bitowych słów,
 * a także dedykowaną funkcję do obsługi multipleksera PCA9548.
 *
 *
 * Obsługa błędów została zrealizowana poprzez zwracanie kodów statusu
 * oraz opcjonalne informowanie o błędach za pośrednictwem UART.
 */


#include "i2c.h"
#include "uart.h"

/**
 * @brief Inicjalizuje magistralę I2C w trybie master.
 *
 * Funkcja konfiguruje piny SDA i SCL, włącza rezystory podciągające oraz
 * ustawia prędkość zegara zgodnie z parametrami zdefiniowanymi w pliku config.h.
 * Następnie rejestruje sterownik I2C dla określonego portu.
 */
void initI2C() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Wysyła jeden bajt danych do wskazanego rejestru urządzenia I2C.
 *
 * Funkcja tworzy transakcję I2C i przesyła adres urządzenia, adres rejestru
 * oraz wartość bajtu, który ma zostać zapisany. Transakcja kończy się sygnałem STOP.
 *
 * @param dev_addr Adres 7-bitowy urządzenia I2C (bez bitu R/W)
 * @param reg_addr Adres rejestru w urządzeniu, do którego ma zostać zapisany bajt
 * @param data     Wartość bajtu do zapisania w rejestrze
 * @return 0x00 jeśli transmisja zakończyła się powodzeniem, 0xFF w przypadku błędu
 */
uint8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? 0x00 : 0xFF;
}

/**
 * @brief Odczytuje jeden bajt danych z wybranego rejestru urządzenia I2C.
 *
 * Funkcja wysyła adres urządzenia i adres rejestru, a następnie inicjuje
 * odczyt jednego bajtu danych z tego rejestru.
 *
 * @param dev_addr Adres 7-bitowy urządzenia I2C
 * @param reg_addr Adres rejestru w urządzeniu, z którego ma zostać odczytany bajt
 * @param data     Wskaźnik do zmiennej, w której zostanie zapisany odczytany bajt
 * @return 0x00 jeśli odczyt zakończył się powodzeniem, 0xFF w przypadku błędu lub NULL jako wskaźnik
 */
uint8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    if (data == NULL) return 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return 0xFF;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0x00 : 0xFF;
}

/**
 * @brief Wysyła 16-bitową wartość do urządzenia I2C.
 *
 * Funkcja zapisuje dwa bajty (MSB i LSB) pod wskazanym adresem rejestru.
 * Dane są przesyłane w jednej transakcji, co gwarantuje ich spójność.
 *
 * @param dev_addr  Adres 7-bitowy urządzenia I2C
 * @param reg_addr  Adres rejestru początkowego (MSB)
 * @param value     16-bitowa wartość do zapisania (MSB:LSB)
 * @return 0x0000 jeśli operacja się powiodła, 0xFFFF w przypadku błędu
 */
uint16_t i2c_write16(uint8_t dev_addr, uint8_t reg_addr, uint16_t value) {
    uint8_t msb = (value >> 8) & 0xFF;
    uint8_t lsb = value & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, msb, true);
    i2c_master_write_byte(cmd, lsb, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? 0x0000 : 0xAAAA;
}

/**
 * @brief Odczytuje 16-bitową wartość z urządzenia I2C.
 *
 * Funkcja wysyła adres rejestru i wykonuje odczyt dwóch kolejnych bajtów,
 * interpretowanych jako wartość 16-bitowa (MSB pierwsze).
 *
 * @param dev_addr  Adres 7-bitowy urządzenia I2C
 * @param reg_addr  Adres rejestru początkowego (MSB)
 * @return Zwraca 16-bitową wartość lub 0xFFFF w przypadku błędu
 */
uint16_t i2c_read16(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data[2] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return 0xAAAA;

    return ((uint16_t)data[0] << 8) | data[1];
}

/**
  * @brief Wysyła pojedynczą komendę do urządzenia I2C (np. ADS1219).
  *
  * Funkcja inicjuje transmisję I²C, wysyłając sam bajt komendy (np. RESET, START, POWERDOWN, RDATA)
  * bez podawania adresu rejestru. Używana do prostych komend sterujących w protokole ADS1219.
  *
  * @param dev_addr Adres 7-bitowy urządzenia I2C (bez bitu R/W)
  * @param command Bajt komendy do wysłania (np. 0x06 dla RESET)
  * @return 0x00 jeśli komenda została poprawnie wysłana, 0xFF w przypadku błędu
  */

uint8_t i2c_send_command(uint8_t dev_addr, uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? 0x00 : 0xFF;
}

/**
 * @brief Odczytuje 24-bitową wartość (3 bajty) z urządzenia I2C.
 *
 * Funkcja inicjuje odczyt trzech bajtów danych z urządzenia, np. z konwersji ADS1219.
 * Wynik zwracany jest jako liczba 32-bitowa (int32_t), z poprawnym rozszerzeniem znaku
 * dla wartości ujemnych (format 2's complement).
 *
 * @param dev_addr  Adres 7-bitowy urządzenia I2C (bez bitu R/W)
 * @param result    Wskaźnik do zmiennej, w której zostanie zapisany wynik (int32_t)
 * @return 0x00 jeśli odczyt zakończył się powodzeniem, 0xFF w przypadku błędu lub NULL jako wskaźnik
 */
int32_t i2c_read24(uint8_t dev_addr) {
    uint8_t data[3] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return 0x7FFFFFFF;  // wartość błędu

    int32_t result = (data[0] << 16) | (data[1] << 8) | data[2];
    if (result & 0x800000) result |= 0xFF000000;  // rozszerzenie znaku dla wartości ujemnych

    return result;
}




/**
 * @brief Ustawia aktywny kanał multipleksera PCA9548 poprzez zapis bitmaski.
 *
 * Funkcja aktywuje wybrany kanał multipleksera PCA9548, wysyłając wartość
 * bitmaski na jego adres. W przypadku błędu, wysyłany jest pakiet diagnostyczny
 * przez UART w formacie: [0xAA][0x01][checksum].
 *
 * @param channel_addr 7-bitowy adres urządzenia PCA9548
 * @param bitmask      Maska bitowa wskazująca kanał do aktywacji (np. 0x01 dla CH0, 0x02 dla CH1)
 */
void pca9548channel(uint8_t channel_addr, uint8_t bitmask) {
    uint8_t startByte = 0xAA;
    uint8_t error_byte = 0xBB;

    if (i2c_write_byte(channel_addr, 0x00, bitmask) == 0x00) {
        return; // OK – nie wysyłamy nic
    }

    uint8_t timeoutSum = checksum(startByte, error_byte);
    uart_write_bytes(UART_PORT, &startByte, 1);
    uart_write_bytes(UART_PORT, &error_byte, 1);
    uart_write_bytes(UART_PORT, &timeoutSum, 1);
}
