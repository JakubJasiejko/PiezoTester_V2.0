#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "uart.h"
#include "i2c.h"
#include "ads1219.h"

static const float V_SUPPLY = 3.3f;
static const float VREF_EXT = 3.3f;
static const float R1 = 100000.0f;
static const float RESISTANCE_MEASURED = 198496.0f;
static const float RESISTANCE_OMOMETHER = 197600.0f;
static const float LOAD_CELL_CAPACITY_KG = 5.0f;
static const float LOAD_CELL_SENSITIVITY_MV_V = 2.0f;
static const float LOAD_CELL_EXCITATION_V = 3.3f;
static const float LOAD_CELL_HARDWARE_GAIN = 62.0f;
static const uint8_t LOAD_CALIBRATION_VERSION = 2;

#define MOSFET_PIN GPIO_NUM_26
#define MOSFET_ON_LEVEL 0
#define MOSFET_OFF_LEVEL 1

#define ZERO_AVG_SAMPLES 8
#define AVG_SAMPLES 8
#define CAL_STAGE_MEASUREMENTS 10

typedef struct {
    bool valid;
    float a;
    float b;
    float cal_mass_1;
    float cal_mass_2;
    float cal_diff_1;
    float cal_diff_2;
} calibration_data_t;

typedef struct {
    bool valid;
    float factor;
    float reference_resistance;
    float raw_resistance;
    float measured_voltage;
} sensor_calibration_t;

typedef struct {
    float load_voltage;
    float load_zero;
    float load_relative;
    float mass_kg;
    float mass_g;
    float sensor_voltage;
    float sensor_resistance;
} test_result_t;

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_WAIT_ZERO,
    CAL_STATE_WAIT_POINT,
    CAL_STATE_READY_TO_SAVE,
} calibration_state_t;

static calibration_data_t g_calibration = {
    .valid = false,
    .a = 1.0f,
    .b = 1.0f,
};

static sensor_calibration_t g_sensor_calibration = {
    .valid = false,
    .factor = 1.0f,
    .reference_resistance = 0.0f,
    .raw_resistance = 0.0f,
    .measured_voltage = 0.0f,
};

static float g_calibration_zero = 0.0f;
static float g_preview_zero = 0.0f;
static bool g_preview_zero_valid = false;
static int g_calibration_points = 0;
static uint32_t g_test_counter = 0;
static calibration_state_t g_calibration_state = CAL_STATE_IDLE;

static inline float calculate_load_signal(float zero_voltage, float measured_voltage);

static inline float load_cell_full_scale_voltage(void)
{
    return (LOAD_CELL_SENSITIVITY_MV_V / 1000.0f) * LOAD_CELL_EXCITATION_V * LOAD_CELL_HARDWARE_GAIN;
}

static inline float load_cell_nominal_kg_per_v(void)
{
    const float full_scale_voltage = load_cell_full_scale_voltage();
    if (!(full_scale_voltage > 0.0f) || !isfinite(full_scale_voltage)) {
        return 0.0f;
    }

    return LOAD_CELL_CAPACITY_KG / full_scale_voltage;
}

static inline float load_cell_current_kg_per_v(void)
{
    if (g_calibration.a > 0.0f && isfinite(g_calibration.a)) {
        return g_calibration.a;
    }

    return load_cell_nominal_kg_per_v();
}

static void calibration_invalidate_model(void)
{
    g_calibration.valid = false;
    g_calibration.a = load_cell_nominal_kg_per_v();
    g_calibration.b = 1.0f;
    g_calibration.cal_mass_1 = 0.0f;
    g_calibration.cal_mass_2 = 0.0f;
    g_calibration.cal_diff_1 = 0.0f;
    g_calibration.cal_diff_2 = 0.0f;
}

static float ads1219_measure(uint8_t mux_sel, uint8_t gain_sel, float vref, uint8_t reference)
{
    ads1219_configureMeasurement(
        ADS1219_ADRESS,
        mux_sel,
        gain_sel,
        ADS1219_DATA_RATE_20SPS,
        ADS1219_CONV_MODE_SINGLE,
        reference
    );

    vTaskDelay(pdMS_TO_TICKS(2));

    const float gain = (gain_sel == ADS1219_GAIN_4) ? 4.0f : 1.0f;
    float sum = 0.0f;

    for (int i = 0; i < AVG_SAMPLES + 1; ++i) {
        ads1219_startSync(ADS1219_ADRESS);
        const int32_t raw = ads1219_read(ADS1219_ADRESS);

        if (i > 0) {
            const float voltage = (raw / 8388608.0f) * vref;
            sum += voltage / gain;
        }
    }

    return sum / AVG_SAMPLES;
}

static float set_zero(int8_t type, int8_t gain, float vref)
{
    float acc = 0.0f;

    for (int i = 0; i < ZERO_AVG_SAMPLES; ++i) {
        acc += ads1219_measure((uint8_t)type, (uint8_t)gain, vref, ADS1219_VREF_EXTERNAL);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return acc / (float)ZERO_AVG_SAMPLES;
}

static float calibration_average_zero(void)
{
    float acc = 0.0f;

    for (int i = 0; i < CAL_STAGE_MEASUREMENTS; ++i) {
        acc += set_zero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, VREF_EXT);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return acc / (float)CAL_STAGE_MEASUREMENTS;
}

static float calibration_average_point(void)
{
    float acc = 0.0f;

    for (int i = 0; i < CAL_STAGE_MEASUREMENTS; ++i) {
        const float relative = ads1219_measure(
            ADS1219_MEAS_SINGLE_2,
            ADS1219_GAIN_4,
            VREF_EXT,
            ADS1219_VREF_EXTERNAL
        );
        acc += calculate_load_signal(g_calibration_zero, relative);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return acc / (float)CAL_STAGE_MEASUREMENTS;
}

static float sensor_average_voltage(void)
{
    float acc = 0.0f;

    for (int i = 0; i < CAL_STAGE_MEASUREMENTS; ++i) {
        acc += ads1219_measure(
            ADS1219_MEAS_DIFF_01,
            ADS1219_GAIN_1,
            VREF_EXT,
            ADS1219_VREF_EXTERNAL
        );
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return acc / (float)CAL_STAGE_MEASUREMENTS;
}

static inline float calculate_load_signal(float zero_voltage, float measured_voltage)
{
    return fabsf(measured_voltage - zero_voltage);
}

static inline float calculate_load(float diff_voltage)
{
    const float signal_voltage = fabsf(diff_voltage);
    if (!(signal_voltage > 0.0f) || !isfinite(signal_voltage)) {
        return 0.0f;
    }

    const float mass_kg = load_cell_current_kg_per_v() * signal_voltage;

    if (!isfinite(mass_kg) || mass_kg < 0.0f) {
        return 0.0f;
    }

    return mass_kg;
}

static inline float calculate_resistance_raw(float voltage)
{
    return (voltage * R1) / (V_SUPPLY - voltage);
}

static inline float calculate_resistance(float voltage)
{
    return calculate_resistance_raw(voltage) * g_sensor_calibration.factor;
}

static void write_line(const char *text)
{
    uart_write_bytes(UART_PORT, text, strlen(text));
    uart_write_bytes(UART_PORT, "\r\n", 2);
}

static void writef(const char *fmt, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    write_line(buffer);
}

static int uart_read_line(char *buf, int maxlen, TickType_t timeout_ticks)
{
    int idx = 0;
    uint8_t ch = 0;
    int64_t t_end = esp_timer_get_time() + (int64_t)timeout_ticks * (1000000 / configTICK_RATE_HZ);

    while (esp_timer_get_time() <= t_end) {
        const int n = uart_read_bytes(UART_PORT, &ch, 1, 10 / portTICK_PERIOD_MS);
        if (n != 1) {
            continue;
        }

        if (ch == '\n') {
            break;
        }

        if (ch == '\r') {
            continue;
        }

        if (idx < maxlen - 1) {
            buf[idx++] = (char)ch;
        }
    }

    buf[idx] = '\0';
    return idx;
}

static esp_err_t calibration_recalculate_model(void)
{
    const float signal_voltage = fabsf(g_calibration.cal_diff_1);
    const float reference_mass_kg = g_calibration.cal_mass_1;
    const float nominal_kg_per_v = load_cell_nominal_kg_per_v();

    if (
        !(reference_mass_kg > 0.0f) ||
        !isfinite(reference_mass_kg) ||
        !(signal_voltage > 0.0f) ||
        !isfinite(signal_voltage) ||
        !(nominal_kg_per_v > 0.0f) ||
        !isfinite(nominal_kg_per_v)
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    const float nominal_mass_kg = signal_voltage * nominal_kg_per_v;
    if (!(nominal_mass_kg > 0.0f) || !isfinite(nominal_mass_kg)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float correction_factor = reference_mass_kg / nominal_mass_kg;
    const float corrected_kg_per_v = nominal_kg_per_v * correction_factor;
    if (
        !(correction_factor > 0.0f) ||
        !isfinite(correction_factor) ||
        !(corrected_kg_per_v > 0.0f) ||
        !isfinite(corrected_kg_per_v)
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    g_calibration.a = corrected_kg_per_v;
    g_calibration.b = correction_factor;
    g_calibration.cal_diff_1 = signal_voltage;
    g_calibration.cal_mass_2 = nominal_mass_kg;
    g_calibration.cal_diff_2 = g_calibration_zero;

    return ESP_OK;
}

static esp_err_t load_calibration_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        calibration_invalidate_model();
        return err;
    }

    uint8_t valid = 0;
    err = nvs_get_u8(handle, "valid", &valid);
    if (err != ESP_OK || valid != 1) {
        nvs_close(handle);
        calibration_invalidate_model();
        return ESP_ERR_NVS_NOT_FOUND;
    }

    uint8_t version = 0;
    err = nvs_get_u8(handle, "cal_ver", &version);
    if (err != ESP_OK || version != LOAD_CALIBRATION_VERSION) {
        nvs_close(handle);
        calibration_invalidate_model();
        return ESP_ERR_NVS_NOT_FOUND;
    }

    size_t size = sizeof(g_calibration);
    err = nvs_get_blob(handle, "cal", &g_calibration, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(g_calibration)) {
        err = calibration_recalculate_model();
        if (err != ESP_OK) {
            calibration_invalidate_model();
            return err;
        }
        g_calibration.valid = true;
    } else {
        calibration_invalidate_model();
    }
    return err;
}

static void sensor_calibration_reset_defaults(void)
{
    g_sensor_calibration.valid = false;
    g_sensor_calibration.factor = RESISTANCE_OMOMETHER / RESISTANCE_MEASURED;
    g_sensor_calibration.reference_resistance = 0.0f;
    g_sensor_calibration.raw_resistance = 0.0f;
    g_sensor_calibration.measured_voltage = 0.0f;
}

static esp_err_t save_calibration_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t valid = 1;
    uint8_t version = LOAD_CALIBRATION_VERSION;
    g_calibration.valid = true;

    err = nvs_set_blob(handle, "cal", &g_calibration, sizeof(g_calibration));
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, "valid", valid);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, "cal_ver", version);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t load_sensor_calibration_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        sensor_calibration_reset_defaults();
        return err;
    }

    size_t size = sizeof(g_sensor_calibration);
    err = nvs_get_blob(handle, "sensor_cal", &g_sensor_calibration, &size);
    nvs_close(handle);

    if (
        err != ESP_OK ||
        size != sizeof(g_sensor_calibration) ||
        !(g_sensor_calibration.factor > 0.0f) ||
        !isfinite(g_sensor_calibration.factor)
    ) {
        sensor_calibration_reset_defaults();
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    g_sensor_calibration.valid = true;
    return ESP_OK;
}

static esp_err_t save_sensor_calibration_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    g_sensor_calibration.valid = true;
    err = nvs_set_blob(handle, "sensor_cal", &g_sensor_calibration, sizeof(g_sensor_calibration));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static void calibration_reset_session(void)
{
    g_calibration_zero = 0.0f;
    g_calibration_points = 0;
    g_calibration_state = CAL_STATE_IDLE;
    g_calibration.cal_mass_1 = 0.0f;
    g_calibration.cal_mass_2 = 0.0f;
    g_calibration.cal_diff_1 = 0.0f;
    g_calibration.cal_diff_2 = 0.0f;
}

static esp_err_t calibration_start(void)
{
    calibration_reset_session();
    g_calibration_state = CAL_STATE_WAIT_ZERO;
    return ESP_OK;
}

static esp_err_t calibration_capture_zero(void)
{
    if (g_calibration_state != CAL_STATE_WAIT_ZERO) {
        return ESP_ERR_INVALID_STATE;
    }

    g_calibration_zero = calibration_average_zero();
    g_preview_zero = g_calibration_zero;
    g_preview_zero_valid = true;
    g_calibration_state = CAL_STATE_WAIT_POINT;
    return ESP_OK;
}

static esp_err_t calibration_capture_point(float mass_kg)
{
    if (g_calibration_state != CAL_STATE_WAIT_POINT) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!(mass_kg > 0.0f) || !isfinite(mass_kg)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float diff = calibration_average_point();
    if (!(diff > 0.0f) || !isfinite(diff)) {
        return ESP_FAIL;
    }

    g_calibration.cal_mass_1 = mass_kg;
    g_calibration.cal_diff_1 = diff;
    g_calibration_points = 1;
    g_calibration_state = CAL_STATE_READY_TO_SAVE;
    return ESP_OK;
}

static esp_err_t calibration_finish(void)
{
    if (g_calibration_points < 1 || g_calibration_state != CAL_STATE_READY_TO_SAVE) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t err = calibration_recalculate_model();
    if (err != ESP_OK) {
        return err;
    }

    return save_calibration_to_nvs();
}

static esp_err_t load_preview_tare(float *zero_voltage_out)
{
    const float zero_voltage = calibration_average_zero();
    if (!isfinite(zero_voltage)) {
        return ESP_FAIL;
    }

    g_preview_zero = zero_voltage;
    g_preview_zero_valid = true;
    if (zero_voltage_out != NULL) {
        *zero_voltage_out = zero_voltage;
    }
    return ESP_OK;
}

static esp_err_t load_preview_read(float *measured_voltage_out, float *signal_voltage_out, float *mass_kg_out)
{
    if (!g_preview_zero_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    const float measured_voltage = ads1219_measure(
        ADS1219_MEAS_SINGLE_2,
        ADS1219_GAIN_4,
        VREF_EXT,
        ADS1219_VREF_EXTERNAL
    );
    if (!isfinite(measured_voltage)) {
        return ESP_FAIL;
    }

    const float signal_voltage = calculate_load_signal(g_preview_zero, measured_voltage);
    const float mass_kg = calculate_load(signal_voltage);

    if (measured_voltage_out != NULL) {
        *measured_voltage_out = measured_voltage;
    }
    if (signal_voltage_out != NULL) {
        *signal_voltage_out = signal_voltage;
    }
    if (mass_kg_out != NULL) {
        *mass_kg_out = mass_kg;
    }

    return ESP_OK;
}

static esp_err_t sensor_calibration_run(float reference_resistance)
{
    if (!(reference_resistance > 0.0f) || !isfinite(reference_resistance)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float measured_voltage = sensor_average_voltage();
    if (!(measured_voltage > 0.0f) || measured_voltage >= (V_SUPPLY - 0.001f) || !isfinite(measured_voltage)) {
        return ESP_FAIL;
    }

    const float raw_resistance = calculate_resistance_raw(measured_voltage);
    if (!(raw_resistance > 0.0f) || !isfinite(raw_resistance)) {
        return ESP_FAIL;
    }

    const float factor = reference_resistance / raw_resistance;
    if (!(factor > 0.0f) || !isfinite(factor)) {
        return ESP_FAIL;
    }

    g_sensor_calibration.valid = true;
    g_sensor_calibration.factor = factor;
    g_sensor_calibration.reference_resistance = reference_resistance;
    g_sensor_calibration.raw_resistance = raw_resistance;
    g_sensor_calibration.measured_voltage = measured_voltage;

    return save_sensor_calibration_to_nvs();
}

static test_result_t run_test_once(void)
{
    test_result_t result = {0};

    result.load_zero = set_zero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, VREF_EXT);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(MOSFET_PIN, MOSFET_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(2000));

    result.load_relative = ads1219_measure(
        ADS1219_MEAS_SINGLE_2,
        ADS1219_GAIN_4,
        VREF_EXT,
        ADS1219_VREF_EXTERNAL
    );
    vTaskDelay(pdMS_TO_TICKS(10));
    result.load_voltage = calculate_load_signal(result.load_zero, result.load_relative);
    result.mass_kg = calculate_load(result.load_voltage);
    result.mass_g = result.mass_kg * 1000.0f;

    result.sensor_voltage = ads1219_measure(
        ADS1219_MEAS_DIFF_01,
        ADS1219_GAIN_1,
        VREF_EXT,
        ADS1219_VREF_EXTERNAL
    );
    vTaskDelay(pdMS_TO_TICKS(10));
    result.sensor_resistance = calculate_resistance(result.sensor_voltage);

    gpio_set_level(MOSFET_PIN, MOSFET_OFF_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(500));

    return result;
}

static void send_status(void)
{
    const float nominal_kg_per_v = load_cell_nominal_kg_per_v();
    writef(
        "STATUS calibrated=%d nominal_kg_per_v=%.9f kg_per_v=%.9f correction=%.9f "
        "cal_mass=%.6f nominal_mass=%.6f cal_signal=%.9f cal_zero=%.9f "
        "sensor_calibrated=%d sensor_factor=%.9f sensor_ref=%.6f sensor_raw=%.6f sensor_voltage=%.9f",
        g_calibration.valid ? 1 : 0,
        nominal_kg_per_v,
        g_calibration.a,
        g_calibration.b,
        g_calibration.cal_mass_1,
        g_calibration.cal_mass_2,
        g_calibration.cal_diff_1,
        g_calibration.cal_diff_2,
        g_sensor_calibration.valid ? 1 : 0,
        g_sensor_calibration.factor,
        g_sensor_calibration.reference_resistance,
        g_sensor_calibration.raw_resistance,
        g_sensor_calibration.measured_voltage
    );
}

static void handle_command(const char *line)
{
    if (strcmp(line, "PING") == 0) {
        write_line("OK PONG");
        return;
    }

    if (strcmp(line, "STATUS") == 0) {
        send_status();
        return;
    }

    if (strcmp(line, "CAL_START") == 0) {
        calibration_start();
        write_line("CAL_START_OK step=remove_load_and_save_zero");
        return;
    }

    if (strcmp(line, "CAL_ZERO") == 0) {
        esp_err_t err = calibration_capture_zero();
        if (err != ESP_OK) {
            writef("ERR calibration_zero_failed code=%d", (int)err);
            return;
        }

        writef("CAL_ZERO_OK zero=%.9f next=place_reference_mass", g_calibration_zero);
        return;
    }

    if (strncmp(line, "CAL_POINT ", 10) == 0) {
        const float mass_kg = strtof(line + 10, NULL);
        esp_err_t err = calibration_capture_point(mass_kg);
        if (err != ESP_OK) {
            writef("ERR calibration_point_failed code=%d", (int)err);
            return;
        }

        writef(
            "CAL_POINT_OK mass=%.6f signal=%.9f estimated_mass_g=%.3f",
            g_calibration.cal_mass_1,
            g_calibration.cal_diff_1,
            calculate_load(g_calibration.cal_diff_1) * 1000.0f
        );
        return;
    }

    if (strcmp(line, "CAL_SAVE") == 0) {
        esp_err_t err = calibration_finish();
        if (err != ESP_OK) {
            writef("ERR calibration_save_failed code=%d", (int)err);
            return;
        }

        writef(
            "CAL_SAVED nominal_kg_per_v=%.9f kg_per_v=%.9f correction=%.9f cal_mass=%.6f nominal_mass=%.6f cal_signal=%.9f cal_zero=%.9f",
            load_cell_nominal_kg_per_v(),
            g_calibration.a,
            g_calibration.b,
            g_calibration.cal_mass_1,
            g_calibration.cal_mass_2,
            g_calibration.cal_diff_1,
            g_calibration.cal_diff_2
        );
        return;
    }

    if (strcmp(line, "LOAD_TARE") == 0) {
        float zero_voltage = 0.0f;
        esp_err_t err = load_preview_tare(&zero_voltage);
        if (err != ESP_OK) {
            writef("ERR load_tare_failed code=%d", (int)err);
            return;
        }

        writef("LOAD_TARE_OK zero=%.9f", zero_voltage);
        return;
    }

    if (strcmp(line, "LOAD_PREVIEW") == 0) {
        float measured_voltage = 0.0f;
        float signal_voltage = 0.0f;
        float mass_kg = 0.0f;
        esp_err_t err = load_preview_read(&measured_voltage, &signal_voltage, &mass_kg);
        if (err != ESP_OK) {
            writef("ERR load_preview_failed code=%d", (int)err);
            return;
        }

        writef(
            "LOAD_PREVIEW zero=%.9f measured=%.9f signal=%.9f mass_kg=%.6f mass_g=%.3f",
            g_preview_zero,
            measured_voltage,
            signal_voltage,
            mass_kg,
            mass_kg * 1000.0f
        );
        return;
    }

    if (strncmp(line, "SENSOR_CAL ", 11) == 0) {
        const float reference_resistance = strtof(line + 11, NULL);
        esp_err_t err = sensor_calibration_run(reference_resistance);
        if (err != ESP_OK) {
            writef("ERR sensor_calibration_failed code=%d", (int)err);
            return;
        }

        writef(
            "SENSOR_CAL_OK ref=%.6f raw=%.6f factor=%.9f corrected=%.6f sensor_v=%.9f",
            g_sensor_calibration.reference_resistance,
            g_sensor_calibration.raw_resistance,
            g_sensor_calibration.factor,
            g_sensor_calibration.raw_resistance * g_sensor_calibration.factor,
            g_sensor_calibration.measured_voltage
        );
        return;
    }

    if (strcmp(line, "TEST") == 0) {
        const test_result_t result = run_test_once();
        writef(
            "TEST_RESULT idx=%lu load_v=%.9f zero_v=%.9f relative_v=%.9f mass_g=%.3f resistance=%.3f sensor_v=%.9f",
            (unsigned long)g_test_counter++,
            result.load_voltage,
            result.load_zero,
            result.load_relative,
            result.mass_g,
            result.sensor_resistance,
            result.sensor_voltage
        );
        return;
    }

    if (strcmp(line, "HELP") == 0) {
        write_line("OK commands=PING,STATUS,CAL_START,CAL_ZERO,CAL_POINT <kg>,CAL_SAVE,LOAD_TARE,LOAD_PREVIEW,SENSOR_CAL <ohm>,TEST,HELP");
        return;
    }

    write_line("ERR unknown_command");
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

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    const bool load_calibration_ok = (load_calibration_from_nvs() == ESP_OK);
    if (!load_calibration_ok) {
        calibration_reset_session();
    }

    sensor_calibration_reset_defaults();
    load_sensor_calibration_from_nvs();
    write_line(load_calibration_ok ? "READY calibrated=1" : "READY calibrated=0");

    char line[128];
    while (1) {
        const int len = uart_read_line(line, sizeof(line), pdMS_TO_TICKS(1000));
        if (len <= 0) {
            continue;
        }
        handle_command(line);
    }
}
