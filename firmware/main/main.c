#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "uart.h"
#include "i2c.h"
#include "ads1219.h"

#define DEBUG

// =======================
// 2-punktowa kalibracja
// =======================

float V = 3.3f;
const float vref_ext = 3.3f;
const float vref_int = 2.048f;
uint32_t nr = 0;
float loadCellZero = 0.0f;

// wpisane wartości:
float CAL_DIFF_1 = 0.001722f;   // zmierzone diff dla 28g
float CAL_DIFF_2 = 0.003433f;   // zmierzone diff dla 57g

float CAL_MASS_1 = 0.028f;      // 28 g w kg
float CAL_MASS_2 = 0.057f;      // 57 g w kg

// współczynniki do regresji liniowej
static float A = 1.0f;
static float B = 0.0f;

// rezystancja czujnika

const float R1 = 200000.0f;

const float resistance_measured = 198496.f;
const float resistance_omomether = 197600.f;

#define MOSFET_PIN        GPIO_NUM_17
#define MOSFET_ON_LEVEL   0
#define MOSFET_OFF_LEVEL  1

#define ZERO_AVG_SAMPLES  8
#define AVG_SAMPLES 8   // ilość próbek do uśrednienia

static float ads1219_measure(uint8_t mux_sel, uint8_t gain_sel, float vref, uint8_t type)
{
    ads1219_configureMeasurement(
        ADS1219_ADRESS,
        mux_sel,
        gain_sel,
        ADS1219_DATA_RATE_20SPS,
        ADS1219_CONV_MODE_SINGLE,
        type
    );

    vTaskDelay(pdMS_TO_TICKS(2)); // stabilizacja

    // Wybór wzmocnienia (2 wartości: 1 albo 4)
    float gain = (gain_sel == ADS1219_GAIN_4) ? 4.0f : 1.0f;

    float sum = 0.0f;

    for (int i = 0; i < AVG_SAMPLES + 1; i++)   // +1 → pierwsza próbka = dummy
    {
        ads1219_startSync(ADS1219_ADRESS);
        int32_t raw = ads1219_read(ADS1219_ADRESS); // czeka na DRDY

        if (i > 0)   // pomijamy pierwszy pomiar (dummy sample)
        {
            float voltage = (raw / 8388608.0f) * vref; // po PGA
            sum += (voltage / gain);                   // przed PGA
        }
    }

    return sum / AVG_SAMPLES;  // średnia wartość napięcia [V]
}

static float setZero(int8_t type, int8_t gain, float vrefn)
{
    float acc = 0.0f;
    for (int i = 0; i < ZERO_AVG_SAMPLES; ++i) {
        acc += ads1219_measure((uint8_t)type, (uint8_t)gain, vrefn, ADS1219_VREF_INTERNAL);
    }
    return acc / (float)ZERO_AVG_SAMPLES;
}

/*
 ======================================================================
 KALIBRACJA 2-PUNKTOWA (REGRESJA LINIOWA NA DWÓCH PUNKTACH)

 
 Mostek tensometryczny daje sygnał napięciowy, który jest prawie
 liniowo zależny od obciążenia. Oznacza to, że możemy opisać zależność
 masa → napięcie za pomocą równania prostej:

        y = A * x + B

 gdzie:
        x – zmierzone napięcie różnicowe [V] (po odjęciu wartości zerowej)
        y – masa [kg] obciążająca czujnik
        A – nachylenie prostej (ile kg przypada na 1 V różnicy napięcia)
        B – przesunięcie (ile kg byłoby dla x = 0)

 ----------------------------------------------------------------------
 WYZNACZANIE A I B NA PODSTAWIE DWÓCH PUNKTÓW KALIBRACYJNYCH:

 W praktyce wybieramy dwa znane obciążenia, np. 28g i 57g.
 Dla każdego z tych obciążeń mierzymy napięcie różnicowe (x1 i x2).
 Mamy więc dwa punkty na układzie współrzędnych:

        P1 = (x1, y1)
        P2 = (x2, y2)

 gdzie y1 i y2 to masy (28g i 57g), a x1 i x2 to zmierzone napięcia.

 Równanie prostej przechodzącej przez dwa punkty jest jednoznaczne.
 Podstawiając punkty do równania y = A * x + B otrzymujemy:
        y1 = A * x1 + B
        y2 = A * x2 + B

 Odejmując równania eliminujemy B:

        y2 - y1 = A * (x2 - x1)

 Stąd:
        A = (y2 - y1) / (x2 - x1)

 Następnie podstawiamy A do jednej z funkcji (np. dla punktu 1),
 aby obliczyć B:

        B = y1 - (A * x1)

 Ostatecznie mamy pełne równanie konwersji napięcia na masę:

        masa(x) = A * x + B

 ----------------------------------------------------------------------
 INTERPRETACJA:

 • A mówi, jak "stromo" rośnie masa przy zwiększaniu się napięcia.
   (Im większe A, tym małe zmiany napięcia dają duże zmiany masy.)

 • B koryguje przesunięcie układu, aby punkt (x1, y1) i (x2, y2)
   były odwzorowane dokładnie — inaczej linia nie przechodziłaby
   przez pomiary kalibracyjne.

 ----------------------------------------------------------------------
 DLACZEGO TO DZIAŁA DOBRZE?

 Dla typowych belki tensometrycznych zależność masa → odkształcenie
 → zmiana rezystancji → zmiana napięcia jest liniowa w szerokim zakresie.
 Dlatego dopasowanie pojedynczej prostej zapewnia poprawne wyniki,
 pod warunkiem że kalibracja została wykonana poprawnie i stabilnie.

 ======================================================================
*/

static void calibrate_two_points(void)
{
    A = (CAL_MASS_2 - CAL_MASS_1) / (CAL_DIFF_2 - CAL_DIFF_1);
    B = CAL_MASS_1 - A * CAL_DIFF_1;
}

static inline float calculateLoad(float diffVoltage)
{
    return ((A * diffVoltage + B));
}

static inline float calculateResistance(float Voltage){

    return ((Voltage * R1) / (V - Voltage)) * (resistance_omomether / resistance_measured);

}

static int uart_read_line(char *buf, int maxlen, TickType_t timeout_ticks)
{
    int idx = 0;
    uint8_t ch;
    int64_t t_end = esp_timer_get_time() + (int64_t)timeout_ticks * (1000000 / configTICK_RATE_HZ);

    while (1) {
        int n = uart_read_bytes(UART_PORT, &ch, 1, 10 / portTICK_PERIOD_MS);
        if (n == 1) {
            if (ch == '\n') break;
            if (ch == '\r') continue;
            if (idx < maxlen - 1) {
                buf[idx++] = (char)ch;
                // echo (opcjonalnie)
                uart_write_bytes(UART_PORT, (const char *)&ch, 1);
            }
        }
        if (esp_timer_get_time() > t_end) break; // timeout
    }
    buf[idx] = '\0';
    return idx; // długość (0 przy pustej/timeout)
}

// --- prompt → float ---
static float uart_read_float_prompt(const char *prompt, TickType_t timeout_ticks)
{
    char line[64];

    // pokaż prompt i przejdź do nowej linii (ważne dla Serial Monitora)
    uart_write_bytes(UART_PORT, prompt, strlen(prompt));
    uart_write_bytes(UART_PORT, "\r\n", 2);

    int n = uart_read_line(line, sizeof(line), timeout_ticks);
    if (n <= 0) return 0.0f;           // brak danych → 0.0 (prosto)
    return strtof(line, NULL);          // konwersja na float
}

// --- Twoja kalibracja: 4 wartości po kolei ---
void calibration(void)
{
    char msg[64];

    uart_write_bytes(UART_PORT, "CALIBRATION\r\n", strlen("CALIBRATION\r\n"));

    loadCellZero = setZero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int);
    vTaskDelay(pdMS_TO_TICKS(50));

    CAL_MASS_1 = uart_read_float_prompt("Podaj MASA 1 [kg]:", pdMS_TO_TICKS(30000));
    snprintf(msg, sizeof(msg), "M1 = %.6f\r\n", CAL_MASS_1);
    uart_write_bytes(UART_PORT, msg, strlen(msg));

    float relative_calibration  = ads1219_measure(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int, ADS1219_VREF_INTERNAL);
    CAL_DIFF_1 = relative_calibration - loadCellZero;

    CAL_MASS_2 = uart_read_float_prompt("Podaj MASA 2 [kg]:", pdMS_TO_TICKS(30000));
    snprintf(msg, sizeof(msg), "M2 = %.6f\r\n", CAL_MASS_2);
    uart_write_bytes(UART_PORT, msg, strlen(msg));

    relative_calibration  = ads1219_measure(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int, ADS1219_VREF_INTERNAL);
    CAL_DIFF_2 = relative_calibration - loadCellZero;

    vTaskDelay(pdMS_TO_TICKS(50));

    
    // podsumowanie
    uart_write_bytes(UART_PORT, "\r\n=== PODSUMOWANIE ===\r\n", strlen("\r\n=== PODSUMOWANIE ===\r\n"));
    snprintf(msg, sizeof(msg), "M1 = %.6f\r\n", CAL_MASS_1);
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    snprintf(msg, sizeof(msg), "D1 = %.6f\r\n", CAL_DIFF_1);
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    snprintf(msg, sizeof(msg), "M2 = %.6f\r\n", CAL_MASS_2);
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    snprintf(msg, sizeof(msg), "D2 = %.6f\r\n\r\n", CAL_DIFF_2);
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    uart_write_bytes(UART_PORT, "KALIBRACJA OK\r\n", strlen("KALIBRACJA OK\r\n"));
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

    gpio_set_level(MOSFET_PIN, MOSFET_OFF_LEVEL);

    initUART();
    initI2C();
    ADS1219_init(ADS1219_RST_PIN, ADS1219_DRDY_PIN);
    gpio_set_level(ADS1219_RST_PIN, 1);

    calibration();

    calibrate_two_points();

    uart_write_bytes(UART_PORT, "START\r\n", strlen("START\r\n"));

    loadCellZero = setZero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int);

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {

       loadCellZero = setZero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int);
        
        #ifdef DEBUG
        uart_write_bytes(UART_PORT, "put", 3);
        #endif
        
        gpio_set_level(MOSFET_PIN, MOSFET_ON_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(2000));

        float relative  = ads1219_measure(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, vref_int, ADS1219_VREF_INTERNAL);
        float v_single2 = relative - loadCellZero;
        float massKg    = calculateLoad(v_single2);
        float massG = massKg * 1000;

        float v_diff01  = ads1219_measure(ADS1219_MEAS_DIFF_01, ADS1219_GAIN_1,  vref_ext, ADS1219_VREF_EXTERNAL);
        float resistance = calculateResistance(v_diff01);
#ifdef DEBUG
        {
            char msg[192];
            int len = snprintf(msg, sizeof(msg),
                               "%lu diff=%.6f diff01=%.6f zero=%.6f rel=%.6f mass=%.3fkg slope=%.6f intercept=%.6f\r\n",
                               (unsigned long)nr, v_diff01, resistance, loadCellZero, relative,
                               massG, A, B);
            if (len > 0) uart_write_bytes(UART_PORT, msg, len);
        }
#endif
#ifndef DEBUG
        // DOCElowe (po #ifndef DEBUG): numer  masa[g]  wartosc_na_czujniku
        {
            char msg[96];
            int len = snprintf(msg, sizeof(msg),
                "%lu %.1f %.0f\r\n", (unsigned long)nr, massG, resistance);
            if (len > 0) uart_write_bytes(UART_PORT, msg, len);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(MOSFET_PIN, MOSFET_OFF_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(2000));

        nr++;
    }
}
