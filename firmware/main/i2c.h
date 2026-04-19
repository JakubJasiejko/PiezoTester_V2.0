/**
 * @file i2c.h
 * @brief I2C transport helpers used by the PiezoTester firmware.
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
#define I2C_MASTER_SDA_IO       19
#define I2C_MASTER_SCL_IO       21
 
 /**
  * @brief Initializes the ESP32 I2C master peripheral.
  */
 void initI2C(void);
 
 /**
  * @brief Writes a single byte to an I2C register.
  */
 uint8_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
 
 /**
  * @brief Reads a single byte from an I2C register.
  */
 uint8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
 
 /**
  * @brief Writes a 16-bit value to an I2C device.
  */
 uint16_t i2c_write16(uint8_t dev_addr, uint8_t reg_addr, uint16_t value);
 
 /**
  * @brief Reads a 16-bit value from an I2C device.
  */
 uint16_t i2c_read16(uint8_t dev_addr, uint8_t reg_addr);
 
 /**
  * @brief Selects an active channel on the PCA9548 I2C multiplexer.
  */
 void pca9548channel(uint8_t channel_addr, uint8_t bitmask);
 


int32_t i2c_read24(uint8_t dev_addr);

/**
 * @brief Sends a command-only I2C frame to a device such as the ADS1219.
 */
uint8_t i2c_send_command(uint8_t dev_addr, uint8_t command);
 
 
 #ifdef __cplusplus
 
 #endif
 
 #endif // I2C_H
 
