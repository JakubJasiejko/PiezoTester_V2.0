/**
 * @file uart.h
 * @author inż. Jakub Jasiejko
 * @date 2025-03-15
 * @brief Nagłówek do obsługi komunikacji UART-USB z sumą kontrolną XOR.
 *
 * @details
 * Plik nagłówkowy zawiera deklaracje funkcji służących do inicjalizacji
 * portu UART, rozpoczęcia komunikacji z hostem oraz przesyłania komunikatów
 * diagnostycznych lub błędów. Komunikacja oparta jest na prostym protokole
 * z sumą kontrolną XOR dla weryfikacji danych.
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
  * @brief Oblicza sumę kontrolną XOR dwóch bajtów.
  *
  * @param byte1 Pierwszy bajt
  * @param byte2 Drugi bajt
  * @return uint8_t Wynik XOR jako suma kontrolna
  */
 uint8_t checksum(uint8_t byte1, uint8_t byte2);
 
 /**
  * @brief Inicjalizuje port UART z ustawieniami z pliku konfiguracyjnego.
  *
  * Parametry transmisji:
  * - Prędkość transmisji (BAUDRATE)
  * - 8 bitów danych
  * - 1 bit stopu
  * - Brak parzystości i flow control
  */
 void initUART();
 
 /**
  * @brief Rozpoczyna komunikację UART z komputerem lub urządzeniem nadrzędnym.
  *
  * Czeka na ramkę startową [0xAA][0x01][checksum], weryfikuje poprawność danych
  * i wysyła odpowiedź potwierdzającą lub komunikat o błędzie (timeout).
  */
 void beginSerialCommunication();
 
 /**
  * @brief Wysyła ramkę błędu w formacie [0xAA][errorMask][checksum].
  *
  * @param errorMask Kod błędu do przesłania przez UART (np. 0xCC, 0xDD)
  */
 void errorOnUART(uint8_t errorMask);
 
 /**
  * @brief Wysyła 16-bitową wartość w formacie binarnym (ASCII '0'/'1') przez UART.
  *
  * Umożliwia debugowanie wartości konfiguracyjnych (np. rejestry FDC1004).
  *
  * @param value Wartość 16-bitowa do wyświetlenia binarnie
  */
 void binaryDebug(uint16_t value);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // UART_H
 