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
#include "driver/ledc.h"
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
static const uint8_t ELECTROMAGNET_MODEL_VERSION = 2;
static const uint8_t ELECTROMAGNET_PREMODEL_VERSION = 1;

#define MOSFET_PIN GPIO_NUM_26
#define MOSFET_ON_LEVEL 0
#define MOSFET_OFF_LEVEL 1

#define ELECTROMAGNET_PWM_TIMER LEDC_TIMER_0
#define ELECTROMAGNET_PWM_MODE LEDC_LOW_SPEED_MODE
#define ELECTROMAGNET_PWM_CHANNEL LEDC_CHANNEL_0
#define ELECTROMAGNET_PWM_FREQ_HZ 5000
#define ELECTROMAGNET_PWM_RESOLUTION LEDC_TIMER_13_BIT
#define ELECTROMAGNET_PWM_MAX_DUTY ((1U << 13) - 1U)
#define ELECTROMAGNET_SETTLE_MS 1300
#define ELECTROMAGNET_COOLDOWN_MS 300
#define ELECTROMAGNET_TARGET_ATTEMPTS 3
#define ELECTROMAGNET_POINT_TOLERANCE_G 60.0f
#define ELECTROMAGNET_MIN_LEARNED_MASS_G 10.0f
#define ELECTROMAGNET_MODEL_MAX_POINTS 16
#define ELECTROMAGNET_PULSE_DEFAULT_MS 1500

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
    float target_mass_g;
    float sensor_voltage;
    float sensor_resistance;
    uint32_t pwm_duty;
    float pwm_percent;
    uint32_t session_index;
    uint8_t mode;
} test_result_t;

typedef enum {
    TEST_MODE_STANDARD = 0,
    TEST_MODE_RAMP = 1,
} test_mode_t;

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_WAIT_ZERO,
    CAL_STATE_WAIT_POINT,
    CAL_STATE_READY_TO_SAVE,
} calibration_state_t;

typedef struct {
    uint8_t point_count;
    float mass_g[ELECTROMAGNET_MODEL_MAX_POINTS];
    uint16_t pwm_duty[ELECTROMAGNET_MODEL_MAX_POINTS];
} electromagnet_model_t;

typedef struct {
    bool valid;
    uint16_t threshold_duty;
    float full_scale_mass_g;
} electromagnet_premodel_t;

typedef struct {
    bool active;
    test_mode_t mode;
    uint16_t sample_count;
    uint16_t current_index;
    float start_mass_g;
    float end_mass_g;
    uint32_t start_pwm_duty;
    uint32_t end_pwm_duty;
} test_session_t;

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
static electromagnet_model_t g_electromagnet_model = {0};
static electromagnet_premodel_t g_electromagnet_premodel = {0};
static test_session_t g_test_session = {0};

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
    bool skip_first_valid = true;
    int valid_samples = 0;

    for (int attempt = 0; attempt < (AVG_SAMPLES + 1) * 2 && valid_samples < AVG_SAMPLES; ++attempt) {
        ads1219_startSync(ADS1219_ADRESS);
        const int32_t raw = ads1219_read(ADS1219_ADRESS);
        if (raw == 0x7FFFFFFF) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (skip_first_valid) {
            skip_first_valid = false;
            continue;
        }

        const float voltage = (raw / 8388608.0f) * vref;
        sum += voltage / gain;
        valid_samples++;
    }

    if (valid_samples < (AVG_SAMPLES / 2)) {
        return NAN;
    }

    return sum / (float)valid_samples;
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

static void electromagnet_apply_duty(uint32_t active_duty)
{
    if (active_duty > ELECTROMAGNET_PWM_MAX_DUTY) {
        active_duty = ELECTROMAGNET_PWM_MAX_DUTY;
    }

    const uint32_t hardware_duty = ELECTROMAGNET_PWM_MAX_DUTY - active_duty;
    ledc_set_duty(ELECTROMAGNET_PWM_MODE, ELECTROMAGNET_PWM_CHANNEL, hardware_duty);
    ledc_update_duty(ELECTROMAGNET_PWM_MODE, ELECTROMAGNET_PWM_CHANNEL);
}

static void electromagnet_off(void)
{
    electromagnet_apply_duty(0);
}

static void electromagnet_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = ELECTROMAGNET_PWM_MODE,
        .duty_resolution = ELECTROMAGNET_PWM_RESOLUTION,
        .timer_num = ELECTROMAGNET_PWM_TIMER,
        .freq_hz = ELECTROMAGNET_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channel_config = {
        .gpio_num = MOSFET_PIN,
        .speed_mode = ELECTROMAGNET_PWM_MODE,
        .channel = ELECTROMAGNET_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = ELECTROMAGNET_PWM_TIMER,
        .duty = ELECTROMAGNET_PWM_MAX_DUTY,
        .hpoint = 0,
        .flags = {
            .output_invert = 0,
        },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    electromagnet_off();
}

static void test_session_reset(void)
{
    memset(&g_test_session, 0, sizeof(g_test_session));
}

static void electromagnet_premodel_reset_defaults(void)
{
    g_electromagnet_premodel.valid = false;
    g_electromagnet_premodel.threshold_duty = 0;
    g_electromagnet_premodel.full_scale_mass_g = LOAD_CELL_CAPACITY_KG * 1000.0f;
}

static void electromagnet_model_sort(void)
{
    for (int i = 0; i < g_electromagnet_model.point_count; ++i) {
        for (int j = i + 1; j < g_electromagnet_model.point_count; ++j) {
            if (g_electromagnet_model.mass_g[j] < g_electromagnet_model.mass_g[i]) {
                const float mass_tmp = g_electromagnet_model.mass_g[i];
                g_electromagnet_model.mass_g[i] = g_electromagnet_model.mass_g[j];
                g_electromagnet_model.mass_g[j] = mass_tmp;

                const uint16_t duty_tmp = g_electromagnet_model.pwm_duty[i];
                g_electromagnet_model.pwm_duty[i] = g_electromagnet_model.pwm_duty[j];
                g_electromagnet_model.pwm_duty[j] = duty_tmp;
            }
        }
    }
}

static void electromagnet_model_reset_defaults(void)
{
    memset(&g_electromagnet_model, 0, sizeof(g_electromagnet_model));
}

static float electromagnet_premodel_full_scale_mass_g(void)
{
    if (
        g_electromagnet_premodel.full_scale_mass_g > 0.0f &&
        isfinite(g_electromagnet_premodel.full_scale_mass_g)
    ) {
        return g_electromagnet_premodel.full_scale_mass_g;
    }

    return LOAD_CELL_CAPACITY_KG * 1000.0f;
}

static uint16_t electromagnet_premodel_threshold_duty(void)
{
    if (!g_electromagnet_premodel.valid) {
        return 0;
    }

    if (g_electromagnet_premodel.threshold_duty >= ELECTROMAGNET_PWM_MAX_DUTY) {
        return (uint16_t)(ELECTROMAGNET_PWM_MAX_DUTY - 1U);
    }

    return g_electromagnet_premodel.threshold_duty;
}

static esp_err_t save_electromagnet_premodel_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, "mag_pre", &g_electromagnet_premodel, sizeof(g_electromagnet_premodel));
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, "mag_pre_ver", ELECTROMAGNET_PREMODEL_VERSION);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t load_electromagnet_premodel_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        electromagnet_premodel_reset_defaults();
        return err;
    }

    uint8_t version = 0;
    err = nvs_get_u8(handle, "mag_pre_ver", &version);
    if (err != ESP_OK || version != ELECTROMAGNET_PREMODEL_VERSION) {
        nvs_close(handle);
        electromagnet_premodel_reset_defaults();
        return ESP_ERR_NVS_NOT_FOUND;
    }

    size_t size = sizeof(g_electromagnet_premodel);
    err = nvs_get_blob(handle, "mag_pre", &g_electromagnet_premodel, &size);
    nvs_close(handle);
    if (
        err != ESP_OK ||
        size != sizeof(g_electromagnet_premodel) ||
        !isfinite(g_electromagnet_premodel.full_scale_mass_g) ||
        g_electromagnet_premodel.full_scale_mass_g <= 0.0f
    ) {
        electromagnet_premodel_reset_defaults();
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    if (g_electromagnet_premodel.threshold_duty >= ELECTROMAGNET_PWM_MAX_DUTY) {
        g_electromagnet_premodel.threshold_duty = (uint16_t)(ELECTROMAGNET_PWM_MAX_DUTY - 1U);
    }

    return ESP_OK;
}

static esp_err_t electromagnet_premodel_set(uint32_t threshold_duty, float full_scale_mass_g)
{
    if (
        threshold_duty >= ELECTROMAGNET_PWM_MAX_DUTY ||
        !(full_scale_mass_g > 0.0f) ||
        !isfinite(full_scale_mass_g)
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    g_electromagnet_premodel.valid = true;
    g_electromagnet_premodel.threshold_duty = (uint16_t)threshold_duty;
    g_electromagnet_premodel.full_scale_mass_g = full_scale_mass_g;
    return save_electromagnet_premodel_to_nvs();
}

static esp_err_t electromagnet_premodel_clear(void)
{
    electromagnet_premodel_reset_defaults();
    return save_electromagnet_premodel_to_nvs();
}

static void electromagnet_pulse(uint32_t active_duty, uint32_t hold_ms)
{
    if (hold_ms == 0U) {
        hold_ms = ELECTROMAGNET_PULSE_DEFAULT_MS;
    }

    electromagnet_apply_duty(active_duty);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    electromagnet_off();
    vTaskDelay(pdMS_TO_TICKS(ELECTROMAGNET_COOLDOWN_MS));
}

static esp_err_t save_electromagnet_model_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, "mag_model", &g_electromagnet_model, sizeof(g_electromagnet_model));
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, "mag_model_ver", ELECTROMAGNET_MODEL_VERSION);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t load_electromagnet_model_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("piezo", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        electromagnet_model_reset_defaults();
        return err;
    }

    uint8_t version = 0;
    err = nvs_get_u8(handle, "mag_model_ver", &version);
    if (err != ESP_OK || version != ELECTROMAGNET_MODEL_VERSION) {
        nvs_close(handle);
        electromagnet_model_reset_defaults();
        return ESP_ERR_NVS_NOT_FOUND;
    }

    size_t size = sizeof(g_electromagnet_model);
    err = nvs_get_blob(handle, "mag_model", &g_electromagnet_model, &size);
    nvs_close(handle);
    if (
        err != ESP_OK ||
        size != sizeof(g_electromagnet_model) ||
        g_electromagnet_model.point_count > ELECTROMAGNET_MODEL_MAX_POINTS
    ) {
        electromagnet_model_reset_defaults();
        return (err != ESP_OK) ? err : ESP_FAIL;
    }

    electromagnet_model_sort();
    return ESP_OK;
}

static void electromagnet_model_upsert_point(float mass_g, uint32_t pwm_duty)
{
    if (
        !(mass_g >= ELECTROMAGNET_MIN_LEARNED_MASS_G) ||
        !isfinite(mass_g) ||
        pwm_duty == 0 ||
        pwm_duty > ELECTROMAGNET_PWM_MAX_DUTY
    ) {
        return;
    }

    for (int i = 0; i < g_electromagnet_model.point_count; ++i) {
        if (fabsf(g_electromagnet_model.mass_g[i] - mass_g) <= ELECTROMAGNET_POINT_TOLERANCE_G) {
            g_electromagnet_model.mass_g[i] = (g_electromagnet_model.mass_g[i] + mass_g) * 0.5f;
            g_electromagnet_model.pwm_duty[i] = (uint16_t)((g_electromagnet_model.pwm_duty[i] + pwm_duty) / 2U);
            electromagnet_model_sort();
            save_electromagnet_model_to_nvs();
            return;
        }
    }

    if (g_electromagnet_model.point_count < ELECTROMAGNET_MODEL_MAX_POINTS) {
        const int idx = g_electromagnet_model.point_count++;
        g_electromagnet_model.mass_g[idx] = mass_g;
        g_electromagnet_model.pwm_duty[idx] = (uint16_t)pwm_duty;
    } else {
        int replace_idx = 0;
        float max_distance = -1.0f;
        for (int i = 0; i < g_electromagnet_model.point_count; ++i) {
            const float distance = fabsf(g_electromagnet_model.mass_g[i] - mass_g);
            if (distance > max_distance) {
                max_distance = distance;
                replace_idx = i;
            }
        }
        g_electromagnet_model.mass_g[replace_idx] = mass_g;
        g_electromagnet_model.pwm_duty[replace_idx] = (uint16_t)pwm_duty;
    }

    electromagnet_model_sort();
    save_electromagnet_model_to_nvs();
}

static float electromagnet_default_mass_per_duty(void)
{
    const uint32_t threshold_duty = electromagnet_premodel_threshold_duty();
    const uint32_t usable_span = (threshold_duty < ELECTROMAGNET_PWM_MAX_DUTY)
        ? (ELECTROMAGNET_PWM_MAX_DUTY - threshold_duty)
        : 1U;
    return electromagnet_premodel_full_scale_mass_g() / (float)usable_span;
}

static uint32_t electromagnet_default_pwm_for_mass(float target_mass_g)
{
    if (!(target_mass_g > 0.0f) || !isfinite(target_mass_g)) {
        return 0;
    }

    const float full_scale_mass_g = electromagnet_premodel_full_scale_mass_g();
    const uint32_t threshold_duty = electromagnet_premodel_threshold_duty();
    const uint32_t usable_span = (threshold_duty < ELECTROMAGNET_PWM_MAX_DUTY)
        ? (ELECTROMAGNET_PWM_MAX_DUTY - threshold_duty)
        : 1U;

    float estimated = (float)threshold_duty;
    if (target_mass_g >= full_scale_mass_g) {
        estimated = (float)ELECTROMAGNET_PWM_MAX_DUTY;
    } else {
        estimated += ((float)usable_span * target_mass_g) / full_scale_mass_g;
    }

    if (estimated <= 0.0f) {
        return 0;
    }
    if (estimated >= (float)ELECTROMAGNET_PWM_MAX_DUTY) {
        return ELECTROMAGNET_PWM_MAX_DUTY;
    }
    return (uint32_t)lrintf(estimated);
}

static uint32_t electromagnet_estimate_pwm_for_mass(float target_mass_g)
{
    if (!(target_mass_g > 0.0f) || !isfinite(target_mass_g)) {
        return 0;
    }

    if (g_electromagnet_model.point_count == 0) {
        return electromagnet_default_pwm_for_mass(target_mass_g);
    }

    if (g_electromagnet_model.point_count == 1) {
        const float ref_mass = g_electromagnet_model.mass_g[0];
        const float ref_duty = (float)g_electromagnet_model.pwm_duty[0];
        const float threshold_duty = (float)electromagnet_premodel_threshold_duty();
        float duty_per_mass = ((float)ELECTROMAGNET_PWM_MAX_DUTY - threshold_duty) / electromagnet_premodel_full_scale_mass_g();

        if (ref_mass > 0.0f && ref_duty > threshold_duty) {
            duty_per_mass = (ref_duty - threshold_duty) / ref_mass;
        }

        const float estimated = threshold_duty + target_mass_g * duty_per_mass;
        if (estimated <= 0.0f) {
            return 0;
        }
        if (estimated >= (float)ELECTROMAGNET_PWM_MAX_DUTY) {
            return ELECTROMAGNET_PWM_MAX_DUTY;
        }
        return (uint32_t)lrintf(estimated);
    }

    const float first_mass = g_electromagnet_model.mass_g[0];
    const float last_mass = g_electromagnet_model.mass_g[g_electromagnet_model.point_count - 1];

    for (int i = 0; i < g_electromagnet_model.point_count - 1; ++i) {
        const float mass_a = g_electromagnet_model.mass_g[i];
        const float mass_b = g_electromagnet_model.mass_g[i + 1];
        const float duty_a = (float)g_electromagnet_model.pwm_duty[i];
        const float duty_b = (float)g_electromagnet_model.pwm_duty[i + 1];

        if (target_mass_g >= mass_a && target_mass_g <= mass_b && mass_b > mass_a) {
            const float ratio = (target_mass_g - mass_a) / (mass_b - mass_a);
            return (uint32_t)lrintf(duty_a + (duty_b - duty_a) * ratio);
        }
    }

    float mass_a = 0.0f;
    float mass_b = first_mass;
    float duty_a = (float)electromagnet_premodel_threshold_duty();
    float duty_b = (float)g_electromagnet_model.pwm_duty[0];

    if (target_mass_g > last_mass) {
        mass_a = g_electromagnet_model.mass_g[g_electromagnet_model.point_count - 2];
        mass_b = g_electromagnet_model.mass_g[g_electromagnet_model.point_count - 1];
        duty_a = (float)g_electromagnet_model.pwm_duty[g_electromagnet_model.point_count - 2];
        duty_b = (float)g_electromagnet_model.pwm_duty[g_electromagnet_model.point_count - 1];
    }

    float estimated = 0.0f;
    if (mass_b > mass_a) {
        estimated = duty_a + (target_mass_g - mass_a) * (duty_b - duty_a) / (mass_b - mass_a);
    } else {
        estimated = target_mass_g / electromagnet_default_mass_per_duty();
    }

    if (estimated <= 0.0f) {
        return 0;
    }
    if (estimated >= (float)ELECTROMAGNET_PWM_MAX_DUTY) {
        return ELECTROMAGNET_PWM_MAX_DUTY;
    }
    return (uint32_t)lrintf(estimated);
}

static float electromagnet_estimate_mass_per_duty(float target_mass_g, uint32_t pwm_duty, float measured_mass_g)
{
    const uint32_t threshold_duty = electromagnet_premodel_threshold_duty();
    if (pwm_duty <= threshold_duty) {
        return electromagnet_default_mass_per_duty();
    }

    if (measured_mass_g > 0.0f && isfinite(measured_mass_g)) {
        return measured_mass_g / (float)(pwm_duty - threshold_duty);
    }

    const uint32_t estimated_pwm = electromagnet_estimate_pwm_for_mass(target_mass_g);
    if (estimated_pwm > threshold_duty && target_mass_g > 0.0f) {
        return target_mass_g / (float)(estimated_pwm - threshold_duty);
    }

    return electromagnet_default_mass_per_duty();
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

static bool parse_u32_argument(const char *line, const char *key, uint32_t *value_out)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "%s=", key);

    const char *start = strstr(line, pattern);
    if (start == NULL) {
        return false;
    }

    start += strlen(pattern);
    char *end = NULL;
    const unsigned long value = strtoul(start, &end, 10);
    if (end == start) {
        return false;
    }

    *value_out = (uint32_t)value;
    return true;
}

static bool parse_float_argument(const char *line, const char *key, float *value_out)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "%s=", key);

    const char *start = strstr(line, pattern);
    if (start == NULL) {
        return false;
    }

    start += strlen(pattern);
    char *end = NULL;
    const float value = strtof(start, &end);
    if (end == start || !isfinite(value)) {
        return false;
    }

    *value_out = value;
    return true;
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

static test_result_t measure_test_sample(uint32_t pwm_duty, float target_mass_g, test_mode_t mode, uint32_t session_index)
{
    test_result_t result = {0};

    result.mode = (uint8_t)mode;
    result.target_mass_g = target_mass_g;
    result.pwm_duty = pwm_duty;
    result.pwm_percent = ((float)pwm_duty * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY;
    result.session_index = session_index;

    result.load_zero = set_zero(ADS1219_MEAS_SINGLE_2, ADS1219_GAIN_4, VREF_EXT);
    vTaskDelay(pdMS_TO_TICKS(20));

    electromagnet_apply_duty(pwm_duty);
    vTaskDelay(pdMS_TO_TICKS(ELECTROMAGNET_SETTLE_MS));

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

    electromagnet_off();
    vTaskDelay(pdMS_TO_TICKS(ELECTROMAGNET_COOLDOWN_MS));

    return result;
}

static float test_session_target_mass_g(uint16_t index)
{
    if (!g_test_session.active || g_test_session.mode != TEST_MODE_RAMP) {
        return 0.0f;
    }

    if (g_test_session.sample_count <= 1) {
        return g_test_session.end_mass_g;
    }

    const float progress = (float)index / (float)(g_test_session.sample_count - 1U);
    return g_test_session.start_mass_g + (g_test_session.end_mass_g - g_test_session.start_mass_g) * progress;
}

static test_result_t run_targeted_ramp_sample(float target_mass_g, uint32_t initial_pwm_duty, uint32_t session_index)
{
    uint32_t pwm_duty = initial_pwm_duty;
    test_result_t best_result = {0};
    bool best_result_valid = false;
    float best_error = INFINITY;

    for (int attempt = 0; attempt < ELECTROMAGNET_TARGET_ATTEMPTS; ++attempt) {
        const test_result_t result = measure_test_sample(pwm_duty, target_mass_g, TEST_MODE_RAMP, session_index);
        electromagnet_model_upsert_point(result.mass_g, pwm_duty);

        const float error = fabsf(result.mass_g - target_mass_g);
        if (!best_result_valid || error < best_error) {
            best_result = result;
            best_error = error;
            best_result_valid = true;
        }

        const float tolerance = fmaxf(20.0f, target_mass_g * 0.03f);
        if (error <= tolerance) {
            break;
        }

        if (attempt >= ELECTROMAGNET_TARGET_ATTEMPTS - 1) {
            break;
        }

        float mass_per_duty = electromagnet_estimate_mass_per_duty(target_mass_g, pwm_duty, result.mass_g);
        if (!(mass_per_duty > 0.0001f) || !isfinite(mass_per_duty)) {
            mass_per_duty = electromagnet_default_mass_per_duty();
        }

        int32_t delta_duty = (int32_t)lrintf((target_mass_g - result.mass_g) / mass_per_duty);
        const int32_t max_step = (int32_t)(ELECTROMAGNET_PWM_MAX_DUTY / 4U);
        if (delta_duty > max_step) {
            delta_duty = max_step;
        }
        if (delta_duty < -max_step) {
            delta_duty = -max_step;
        }
        if (delta_duty == 0) {
            delta_duty = (target_mass_g > result.mass_g) ? 1 : -1;
        }

        int32_t next_pwm = (int32_t)pwm_duty + delta_duty;
        if (next_pwm < 0) {
            next_pwm = 0;
        }
        if (next_pwm > (int32_t)ELECTROMAGNET_PWM_MAX_DUTY) {
            next_pwm = (int32_t)ELECTROMAGNET_PWM_MAX_DUTY;
        }

        if ((uint32_t)next_pwm == pwm_duty) {
            break;
        }

        pwm_duty = (uint32_t)next_pwm;
    }

    return best_result;
}

static esp_err_t test_session_setup(test_mode_t mode, float start_mass_g, float end_mass_g, uint16_t sample_count)
{
    if (sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mode == TEST_MODE_RAMP) {
        if (
            start_mass_g < 0.0f ||
            end_mass_g < 0.0f ||
            !isfinite(start_mass_g) ||
            !isfinite(end_mass_g) ||
            end_mass_g < start_mass_g
        ) {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        start_mass_g = 0.0f;
        end_mass_g = 0.0f;
    }

    test_session_reset();
    g_test_session.active = true;
    g_test_session.mode = mode;
    g_test_session.sample_count = sample_count;
    g_test_session.start_mass_g = start_mass_g;
    g_test_session.end_mass_g = end_mass_g;
    g_test_session.start_pwm_duty = electromagnet_estimate_pwm_for_mass(start_mass_g);
    g_test_session.end_pwm_duty = electromagnet_estimate_pwm_for_mass(end_mass_g);
    return ESP_OK;
}

static void test_session_abort(void)
{
    electromagnet_off();
    test_session_reset();
}

static esp_err_t test_session_next(test_result_t *result_out)
{
    if (result_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_test_session.active || g_test_session.sample_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_test_session.current_index >= g_test_session.sample_count) {
        return ESP_ERR_NOT_FINISHED;
    }

    const uint32_t session_index = (uint32_t)g_test_session.current_index;
    test_result_t result = {0};

    if (g_test_session.mode == TEST_MODE_RAMP) {
        const float target_mass_g = test_session_target_mass_g(g_test_session.current_index);
        uint32_t estimated_pwm = electromagnet_estimate_pwm_for_mass(target_mass_g);

        if (g_test_session.sample_count > 1) {
            const float progress = (float)g_test_session.current_index / (float)(g_test_session.sample_count - 1U);
            const float endpoint_estimate =
                (float)g_test_session.start_pwm_duty +
                ((float)g_test_session.end_pwm_duty - (float)g_test_session.start_pwm_duty) * progress;
            const uint32_t blended_pwm = (uint32_t)lrintf((endpoint_estimate + (float)estimated_pwm) * 0.5f);
            estimated_pwm = blended_pwm;
        }

        result = run_targeted_ramp_sample(target_mass_g, estimated_pwm, session_index);
    } else {
        result = measure_test_sample(ELECTROMAGNET_PWM_MAX_DUTY, 0.0f, TEST_MODE_STANDARD, session_index);
        electromagnet_model_upsert_point(result.mass_g, result.pwm_duty);
    }

    g_test_session.current_index++;
    if (g_test_session.current_index >= g_test_session.sample_count) {
        g_test_session.active = false;
    }

    *result_out = result;
    return ESP_OK;
}

static test_result_t run_test_once(void)
{
    return measure_test_sample(ELECTROMAGNET_PWM_MAX_DUTY, 0.0f, TEST_MODE_STANDARD, g_test_counter);
}

static void send_status(void)
{
    const float nominal_kg_per_v = load_cell_nominal_kg_per_v();
    const float threshold_pct =
        ((float)electromagnet_premodel_threshold_duty() * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY;
    writef(
        "STATUS calibrated=%d nominal_kg_per_v=%.9f kg_per_v=%.9f correction=%.9f "
        "cal_mass=%.6f nominal_mass=%.6f cal_signal=%.9f cal_zero=%.9f "
        "sensor_calibrated=%d sensor_factor=%.9f sensor_ref=%.6f sensor_raw=%.6f sensor_voltage=%.9f "
        "mag_points=%u mag_pre_valid=%d mag_threshold=%u mag_threshold_pct=%.3f mag_full_scale_g=%.3f "
        "test_active=%d test_mode=%u test_samples=%u test_index=%u",
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
        g_sensor_calibration.measured_voltage,
        (unsigned)g_electromagnet_model.point_count,
        g_electromagnet_premodel.valid ? 1 : 0,
        (unsigned)electromagnet_premodel_threshold_duty(),
        threshold_pct,
        electromagnet_premodel_full_scale_mass_g(),
        g_test_session.active ? 1 : 0,
        (unsigned)g_test_session.mode,
        (unsigned)g_test_session.sample_count,
        (unsigned)g_test_session.current_index
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

    if (strncmp(line, "MAG_PULSE", 9) == 0) {
        uint32_t duty = 0;
        uint32_t hold_ms = ELECTROMAGNET_PULSE_DEFAULT_MS;
        float pct = 0.0f;
        const bool has_duty = parse_u32_argument(line, "duty", &duty);
        const bool has_pct = parse_float_argument(line, "pct", &pct);
        parse_u32_argument(line, "hold_ms", &hold_ms);

        if (!has_duty && !has_pct) {
            write_line("ERR mag_pulse_invalid_args");
            return;
        }

        if (!has_duty) {
            if (pct < 0.0f || pct > 100.0f) {
                write_line("ERR mag_pulse_invalid_pct");
                return;
            }
            duty = (uint32_t)lrintf((pct * (float)ELECTROMAGNET_PWM_MAX_DUTY) / 100.0f);
        }

        if (duty > ELECTROMAGNET_PWM_MAX_DUTY) {
            write_line("ERR mag_pulse_invalid_duty");
            return;
        }

        if (g_test_session.active) {
            test_session_abort();
        }

        electromagnet_pulse(duty, hold_ms);
        writef(
            "MAG_PULSE_OK duty=%u pct=%.3f hold_ms=%u threshold=%u threshold_pct=%.3f",
            (unsigned)duty,
            ((float)duty * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY,
            (unsigned)hold_ms,
            (unsigned)electromagnet_premodel_threshold_duty(),
            ((float)electromagnet_premodel_threshold_duty() * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY
        );
        return;
    }

    if (strncmp(line, "MAG_PREMODEL_SET", 16) == 0) {
        uint32_t threshold_duty = 0;
        float threshold_pct = 0.0f;
        float full_scale_mass_g = electromagnet_premodel_full_scale_mass_g();
        const bool has_threshold = parse_u32_argument(line, "threshold", &threshold_duty);
        const bool has_threshold_pct = parse_float_argument(line, "threshold_pct", &threshold_pct);
        parse_float_argument(line, "full_scale_g", &full_scale_mass_g);

        if (!has_threshold && !has_threshold_pct) {
            write_line("ERR mag_premodel_invalid_args");
            return;
        }

        if (!has_threshold) {
            if (threshold_pct < 0.0f || threshold_pct >= 100.0f) {
                write_line("ERR mag_premodel_invalid_pct");
                return;
            }
            threshold_duty = (uint32_t)lrintf((threshold_pct * (float)ELECTROMAGNET_PWM_MAX_DUTY) / 100.0f);
        }

        const esp_err_t err = electromagnet_premodel_set(threshold_duty, full_scale_mass_g);
        if (err != ESP_OK) {
            writef("ERR mag_premodel_set_failed code=%d", (int)err);
            return;
        }

        writef(
            "MAG_PREMODEL_SET_OK valid=1 threshold=%u threshold_pct=%.3f full_scale_g=%.3f",
            (unsigned)electromagnet_premodel_threshold_duty(),
            ((float)electromagnet_premodel_threshold_duty() * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY,
            electromagnet_premodel_full_scale_mass_g()
        );
        return;
    }

    if (strcmp(line, "MAG_PREMODEL_CLEAR") == 0) {
        const esp_err_t err = electromagnet_premodel_clear();
        if (err != ESP_OK) {
            writef("ERR mag_premodel_clear_failed code=%d", (int)err);
            return;
        }

        write_line("MAG_PREMODEL_CLEAR_OK");
        return;
    }

    if (strcmp(line, "MAG_PREMODEL_STATUS") == 0) {
        writef(
            "MAG_PREMODEL valid=%d threshold=%u threshold_pct=%.3f full_scale_g=%.3f points=%u",
            g_electromagnet_premodel.valid ? 1 : 0,
            (unsigned)electromagnet_premodel_threshold_duty(),
            ((float)electromagnet_premodel_threshold_duty() * 100.0f) / (float)ELECTROMAGNET_PWM_MAX_DUTY,
            electromagnet_premodel_full_scale_mass_g(),
            (unsigned)g_electromagnet_model.point_count
        );
        return;
    }

    if (strcmp(line, "MAG_MODEL_CLEAR") == 0) {
        electromagnet_model_reset_defaults();
        const esp_err_t err = save_electromagnet_model_to_nvs();
        if (err != ESP_OK) {
            writef("ERR mag_model_clear_failed code=%d", (int)err);
            return;
        }

        write_line("MAG_MODEL_CLEAR_OK");
        return;
    }

    if (strncmp(line, "TEST_SETUP ", 11) == 0) {
        char mode_text[16] = {0};
        float start_g = 0.0f;
        float end_g = 0.0f;
        unsigned sample_count = 0;

        if (sscanf(line, "TEST_SETUP mode=%15s start_g=%f end_g=%f samples=%u", mode_text, &start_g, &end_g, &sample_count) != 4) {
            write_line("ERR test_setup_invalid_args");
            return;
        }

        test_mode_t mode = TEST_MODE_STANDARD;
        if (strcmp(mode_text, "ramp") == 0) {
            mode = TEST_MODE_RAMP;
        } else if (strcmp(mode_text, "standard") != 0) {
            write_line("ERR test_setup_invalid_mode");
            return;
        }

        const esp_err_t err = test_session_setup(mode, start_g, end_g, (uint16_t)sample_count);
        if (err != ESP_OK) {
            writef("ERR test_setup_failed code=%d", (int)err);
            return;
        }

        writef(
            "TEST_SETUP_OK mode=%s samples=%u start_g=%.3f end_g=%.3f start_pwm=%u end_pwm=%u",
            (mode == TEST_MODE_RAMP) ? "ramp" : "standard",
            sample_count,
            g_test_session.start_mass_g,
            g_test_session.end_mass_g,
            (unsigned)g_test_session.start_pwm_duty,
            (unsigned)g_test_session.end_pwm_duty
        );
        return;
    }

    if (strcmp(line, "TEST_NEXT") == 0) {
        test_result_t result = {0};
        const esp_err_t err = test_session_next(&result);
        if (err == ESP_ERR_NOT_FINISHED) {
            write_line("TEST_DONE");
            return;
        }
        if (err != ESP_OK) {
            writef("ERR test_next_failed code=%d", (int)err);
            return;
        }

        writef(
            "TEST_RESULT idx=%lu mode=%s target_g=%.3f pwm=%u pwm_pct=%.3f load_v=%.9f zero_v=%.9f relative_v=%.9f mass_g=%.3f resistance=%.3f sensor_v=%.9f",
            (unsigned long)g_test_counter++,
            (result.mode == TEST_MODE_RAMP) ? "ramp" : "standard",
            result.target_mass_g,
            (unsigned)result.pwm_duty,
            result.pwm_percent,
            result.load_voltage,
            result.load_zero,
            result.load_relative,
            result.mass_g,
            result.sensor_resistance,
            result.sensor_voltage
        );
        return;
    }

    if (strcmp(line, "TEST_ABORT") == 0) {
        test_session_abort();
        write_line("TEST_ABORT_OK");
        return;
    }

    if (strcmp(line, "TEST") == 0) {
        const test_result_t result = run_test_once();
        writef(
            "TEST_RESULT idx=%lu mode=standard target_g=0.000 pwm=%u pwm_pct=%.3f load_v=%.9f zero_v=%.9f relative_v=%.9f mass_g=%.3f resistance=%.3f sensor_v=%.9f",
            (unsigned long)g_test_counter++,
            (unsigned)result.pwm_duty,
            result.pwm_percent,
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
        write_line("OK commands=PING,STATUS,CAL_START,CAL_ZERO,CAL_POINT <kg>,CAL_SAVE,LOAD_TARE,LOAD_PREVIEW,SENSOR_CAL <ohm>,MAG_PULSE duty=<n>|pct=<x> [hold_ms=<ms>],MAG_PREMODEL_SET threshold=<n>|threshold_pct=<x> [full_scale_g=<g>],MAG_PREMODEL_CLEAR,MAG_PREMODEL_STATUS,MAG_MODEL_CLEAR,TEST_SETUP mode=<standard|ramp> start_g=<g> end_g=<g> samples=<n>,TEST_NEXT,TEST_ABORT,TEST,HELP");
        return;
    }

    write_line("ERR unknown_command");
}

void app_main(void)
{
    initUART();
    initI2C();
    ADS1219_init(ADS1219_RST_PIN, ADS1219_DRDY_PIN);
    gpio_set_level(ADS1219_RST_PIN, 1);
    electromagnet_init();

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
    electromagnet_model_reset_defaults();
    load_electromagnet_model_from_nvs();
    electromagnet_premodel_reset_defaults();
    load_electromagnet_premodel_from_nvs();
    test_session_reset();
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
