/**
 * @file i2c.h
 * @author inż. Jakub Jasiejko
 * @date 2025-03-30
 * @brief Deklaracje funkcji I2C dla ESP32.
 *
 * @details
 * Plik nagłówkowy zawierający definicje makr oraz deklaracje funkcji służących
 * do obsługi magistrali I2C w trybie master. Udostępnia funkcje inicjalizacji,
 * odczytu i zapisu danych (zarówno 8- jak i 16-bitowych), a także funkcję
 * sterowania multiplekserem I2C PCA9548.
 *
 */

 #ifndef I2C_H
 #define I2C_H
 
 #include "driver/i2c.h"
 #include <stdint.h>
 
// I2C
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       19
 
 /**
  * @brief Inicjalizuje magistralę I2C w trybie master.
  *
  * Konfiguruje piny SDA/SCL, prędkość zegara i instaluje sterownik I2C.
  */
 void initI2C(void);
 
 /**
  * @brief Wysyła pojedynczy bajt do urządzenia I2C.
  *
  * @param dev_addr  7-bitowy adres urządzenia I2C
  * @param reg_addr  Adres rejestru docelowego w urządzeniu
  * @param data      Wartość bajtu do zapisania
  * @return 0x00 jeśli operacja zakończona sukcesem, 0xFF w przypadku błędu
  */
 uint8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
 
 /**
  * @brief Odczytuje pojedynczy bajt z urządzenia I2C.
  *
  * @param dev_addr  7-bitowy adres urządzenia I2C
  * @param reg_addr  Adres rejestru, z którego ma zostać odczytany bajt
  * @param data      Wskaźnik na zmienną, do której zapisany zostanie bajt
  * @return 0x00 jeśli odczyt zakończony sukcesem, 0xFF w przypadku błędu
  */
 uint8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
 
 /**
  * @brief Zapisuje 16-bitową wartość do urządzenia I2C (w jednej transakcji).
  *
  * @param dev_addr  7-bitowy adres urządzenia I2C
  * @param reg_addr  Adres początkowy rejestru (MSB)
  * @param value     16-bitowa wartość do zapisania (kolejność: MSB, LSB)
  * @return 0x0000 jeśli OK, 0xFFFF w przypadku błędu transmisji
  */
 uint16_t i2c_write16(uint8_t dev_addr, uint8_t reg_addr, uint16_t value);
 
 /**
  * @brief Odczytuje 16-bitową wartość z urządzenia I2C (w jednej transakcji).
  *
  * @param dev_addr  7-bitowy adres urządzenia I2C
  * @param reg_addr  Adres początkowy rejestru (MSB)
  * @return Zwraca wartość 16-bitową (MSB:LSB) lub 0xFFFF w przypadku błędu
  */
 uint16_t i2c_read16(uint8_t dev_addr, uint8_t reg_addr);
 
 /**
  * @brief Wybiera aktywny kanał na multiplekserze PCA9548.
  *
  * @param channel_addr Adres 7-bitowy układu PCA9548
  * @param bitmask      Maska bitowa określająca kanał do aktywacji (np. PCA_CH1 = 0x02)
  *
  * @details
  * W przypadku niepowodzenia transmisji I2C, funkcja wysyła ramkę diagnostyczną
  * przez UART w formacie: [0xAA][0xBB][checksum], gdzie checksum to suma kontrolna.
  */
 void pca9548channel(uint8_t channel_addr, uint8_t bitmask);
 


int32_t i2c_read24(uint8_t dev_addr);

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

uint8_t i2c_send_command(uint8_t dev_addr, uint8_t command);
 
 
 #ifdef __cplusplus
 
 #endif
 
 #endif // I2C_H
 