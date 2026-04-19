/**
 * @file uart.h
 * @brief UART transport helpers used by the PiezoTester firmware.
 */

 #ifndef UART_H
 #define UART_H
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "driver/uart.h"
 #include "esp_system.h"
 #include "esp_timer.h"
 #include "sdkconfig.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 

// UART
#define BAUDRATE        115200
#define UART_PORT       UART_NUM_0
#define TX_PIN          1
#define RX_PIN          3
#define BUF_SIZE        1024
#define TIMEOUT_MS      10000

 /**
  * @brief Computes a simple XOR checksum from two bytes.
  */
 uint8_t checksum(uint8_t byte1, uint8_t byte2);
 
 /**
  * @brief Initializes the UART peripheral used for host communication.
  */
 void initUART();
 
 /**
  * @brief Starts the UART handshake with the host.
  */
 void beginSerialCommunication();
 
 /**
  * @brief Sends a framed UART error message.
  */
 void errorOnUART(uint8_t errorMask);
 
 /**
  * @brief Prints a 16-bit value as an ASCII binary string over UART.
  */
 void binaryDebug(uint16_t value);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // UART_H
 
