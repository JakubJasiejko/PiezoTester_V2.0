/**
 * @file uart.c
 * @author inż. Jakub Jasiejko
 * @date 2025-03-15
 * @brief Obsługa komunikacji UART-USB dla systemu pedobarograficznego.
 *
 * @details
 * Biblioteka realizuje komunikację UART z sumą kontrolną XOR, wykorzystywaną
 * do połączenia z aplikacją zewnętrzną przez USB. Zawiera funkcje inicjalizacji portu,
 * ustanowienia protokołu komunikacji oraz przesyłania błędów i danych diagnostycznych.
 *
 * Projekt: Pedobarograf PODOLOGIA.PL
 */

 #include "uart.h"
 #include <stdio.h>
 #include <string.h>
 #include "sdkconfig.h"
 
 /**
  * @brief Oblicza sumę kontrolną XOR dla dwóch bajtów.
  *
  * Służy do prostego zabezpieczenia ramki danych przez XOR obu bajtów.
  *
  * @param byte1 Pierwszy bajt
  * @param byte2 Drugi bajt
  * @return uint8_t Suma kontrolna (XOR bitowy)
  */
 uint8_t checksum(uint8_t byte1, uint8_t byte2) {
     return byte1 ^ byte2;
 }
 
 /**
  * @brief Inicjalizuje port UART zgodnie z parametrami zdefiniowanymi w config.h.
  *
  * Parametry:
  * - Prędkość transmisji (baud rate)
  * - 8 bitów danych
  * - 1 bit stopu
  * - Brak parzystości i flow control
  */
 void initUART() {
     uart_config_t uart_config = {
         .baud_rate = BAUDRATE,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_APB,
     };
 
     uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
     uart_param_config(UART_PORT, &uart_config);
     uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 }
 
 /**
  * @brief Rozpoczyna komunikację UART z urządzeniem nadrzędnym (np. komputerem).
  *
  * Oczekuje na ramkę: [0xAA][0x01][CHECKSUM], weryfikuje ją i odpowiada:
  * - Poprawna: wysyła [0xAA][0x02][CHECKSUM]
  * - Błędna: nic nie wysyła (lub wypisuje debug)
  * - Timeout (10s): wysyła [0xAA][0xFF][CHECKSUM]
  */
 void beginSerialCommunication() {
     initUART();
 
     while (true) {
         #ifdef UART_DEBUG
             uart_write_bytes(UART_PORT, "UART init\n", 10);
         #endif
 
         uint8_t received_data[3];
         uint8_t startByte = 0xAA;
         uint8_t comStartByte = 0x01;
         uint8_t response_byte = 0x02;
         uint8_t error_byte = 0xFF;
 
         int64_t start_time = esp_timer_get_time();
 
         #ifdef UART_DEBUG
             uart_write_bytes(UART_PORT, "Czekam na 0xAA 0x01...\n", 22);
         #endif
 
         while ((esp_timer_get_time() - start_time) / 1000 < 10000) {
             int len = uart_read_bytes(UART_PORT, received_data, 3, pdMS_TO_TICKS(100));
 
             if (len == 3) {
                 uint8_t calculatedChecksum = checksum(received_data[0], received_data[1]);
 
                 #ifdef UART_DEBUG
                     uart_write_bytes(UART_PORT, "Odebrano: ", 10);
                     uart_write_bytes(UART_PORT, received_data, 3);
                     uart_write_bytes(UART_PORT, "\n", 1);
                 #endif
 
                 if (received_data[0] == startByte &&
                     received_data[1] == comStartByte &&
                     received_data[2] == calculatedChecksum) {
 
                     uint8_t responseChecksum = checksum(startByte, response_byte);
 
                     uart_write_bytes(UART_PORT, &startByte, 1);
                     uart_write_bytes(UART_PORT, &response_byte, 1);
                     uart_write_bytes(UART_PORT, &responseChecksum, 1);
 
                     #ifdef UART_DEBUG
                         uart_write_bytes(UART_PORT, "\nPotwierdzenie wysłane (0x02)\n", 30);
                     #endif
                     return;
                 } else {
                     #ifdef UART_DEBUG
                         uart_write_bytes(UART_PORT, "Błąd sumy kontrolnej!\n", 24);
                     #endif
                 }
             }
         }
 
         // Timeout – brak danych
         uint8_t timeoutSum = checksum(startByte, error_byte);
 
         uart_write_bytes(UART_PORT, &startByte, 1);
         uart_write_bytes(UART_PORT, &error_byte, 1);
         uart_write_bytes(UART_PORT, &timeoutSum, 1);
 
         #ifdef UART_DEBUG
             uart_write_bytes(UART_PORT, "\nTimeout - brak odpowiedzi, wysłano 0xFF\n", 41);
         #endif
     }
 }
 
 /**
  * @brief Wysyła ramkę błędu przez UART w formacie [0xAA][errorMask][checksum].
  *
  * @param errorMask Kod błędu do wysłania (np. 0xCC, 0xDD itd.)
  */
 void errorOnUART(uint8_t errorMask) {
     uint8_t startByte = 0xAA;
     uint8_t sum = checksum(startByte, errorMask);
 
     uart_write_bytes(UART_PORT, &startByte, 1);
     uart_write_bytes(UART_PORT, &errorMask, 1);
     uart_write_bytes(UART_PORT, &sum, 1);
 }
 
 /**
  * @brief Wysyła wartość binarną (16 bitów) przez UART jako ciąg znaków ASCII ('1' i '0').
  *
  * Umożliwia szybki podgląd zawartości rejestru lub konfiguracji np. FDC1004.
  *
  * @param value Wartość 16-bitowa do wysłania
  */
 void binaryDebug(uint16_t value) {
     char binary_str[17];
 
     for (int i = 15; i >= 0; i--) {
         binary_str[15 - i] = (value & (1 << i)) ? '1' : '0';
     }
     binary_str[16] = '\0';
 
     uart_write_bytes(UART_PORT, binary_str, 16);
 }
 