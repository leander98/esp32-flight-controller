#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "esp32-ism330dlc.h"
#include "esp32-esc-xw30a.h"
#include "esp32-wifi-drone-remote.h"
#include "flight-controller.h"

#define IMU_I2C_PORT       I2C_NUM_0
#define IMU_I2C_SDA_GPIO   GPIO_NUM_9
#define IMU_I2C_SCL_GPIO   GPIO_NUM_8
#define IMU_I2C_FREQUENCY  400000U
#define IMU_I2C_ADDRESS    0x6AU
#define IMU_LOG_INTERVAL   256U
#define IMU_YIELD_INTERVAL 256U
#define IMU_NVS_NAMESPACE  "flight_imu"
#define IMU_NVS_CONFIG_KEY "config"
#define IMU_CONFIG_VERSION 1U
#define ESC_COUNT           ESP32_WIFI_DRONE_REMOTE_ESC_COUNT
#define ESC_NVS_NAMESPACE  "flight_esc"
#define ESC_NVS_CONFIG_KEY "channels"
#define ESC_CONFIG_VERSION 1U
#define PID_NVS_NAMESPACE  "flight_pid"
#define PID_NVS_CONFIG_KEY "gains"
#define PID_CONFIG_VERSION 3U
#define CONTROL_CORE        1
#define CONTROL_TASK_STACK  6144U
#define CONTROL_TASK_PRIORITY 10U

static const char *APP_TAG = "flight-controller";
static portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static esp32_wifi_drone_remote_telemetry_t s_telemetry;
static SemaphoreHandle_t s_imu_mutex;
static esp32_ism330dlc_t s_imu;
static SemaphoreHandle_t s_esc_mutex;
static SemaphoreHandle_t s_flight_mutex;
static xw30a_handle_t s_esc_handles[ESC_COUNT];
static xw30a_config_t s_esc_configs[ESC_COUNT];
static int s_programming_esc_index = -1;
static flight_controller_t s_flight_controller;
static flight_movement_command_t s_movement_command;

/** GPIO assignments used when no persisted ESC configuration exists. */
static const gpio_num_t s_default_esc_gpios[ESC_COUNT] = {
    GPIO_NUM_5, GPIO_NUM_7, GPIO_NUM_6, GPIO_NUM_4,
};

/** @brief Versioned representation of the ISM330 settings stored in NVS. */
typedef struct {
    uint8_t version;
    uint8_t accelerometer_odr;
    uint8_t gyroscope_odr;
    uint8_t accelerometer_full_scale;
    uint8_t gyroscope_full_scale;
} imu_persistent_config_t;

/** @brief Versioned persistent settings for all four ESC channels. */
typedef struct {
    uint8_t version;
    struct {
        uint8_t signal_gpio;
        uint16_t pwm_frequency_hz;
        uint16_t min_pulse_us;
        uint16_t max_pulse_us;
        uint16_t calibration_high_time_ms;
    } channel[ESC_COUNT];
} esc_persistent_config_t;

/** @brief Versioned flight-controller gains stored in NVS. */
typedef struct {
    uint8_t version;
    esp32_wifi_drone_remote_pid_config_t gains;
} pid_persistent_config_t;

/** @brief Return whether all submitted PID gains are finite and bounded. */
static bool is_valid_pid_config(
    const esp32_wifi_drone_remote_pid_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    const float values[] = {
        config->roll_kp, config->roll_ki, config->roll_kd,
        config->pitch_kp, config->pitch_ki, config->pitch_kd,
        config->yaw_kp, config->yaw_ki, config->yaw_kd,
        config->altitude_kp, config->altitude_ki, config->altitude_kd,
    };
    for (size_t index = 0; index < sizeof(values) / sizeof(values[0]);
         ++index) {
        if (!isfinite(values[index]) ||
            values[index] < 0.0f || values[index] > 10.0f) {
            return false;
        }
    }
    return isfinite(config->armed_idle_throttle) &&
           config->armed_idle_throttle >= 0.01f &&
           config->armed_idle_throttle <= 0.30f;
}

/** @brief Return whether one ESC configuration is safe and supported. */
static bool is_valid_esc_config(const xw30a_config_t *config)
{
    if (config == NULL ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->signal_gpio) ||
        config->signal_gpio == IMU_I2C_SDA_GPIO ||
        config->signal_gpio == IMU_I2C_SCL_GPIO ||
        config->pwm_frequency_hz == 0U ||
        config->min_pulse_us == 0U ||
        config->min_pulse_us >= config->max_pulse_us ||
        config->calibration_high_time_ms == 0U) {
        return false;
    }
    return (1000000U / config->pwm_frequency_hz) >
           config->max_pulse_us;
}

/** @brief Fill all ESC channels with safe receiver-style PWM defaults. */
static void set_default_esc_configs(void)
{
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        s_esc_configs[index] = (xw30a_config_t)
            XW30A_DEFAULT_CONFIG(s_default_esc_gpios[index]);
        s_esc_configs[index].mcpwm_group_id = index < 2U ? 0 : 1;
    }
}

/** @brief Store all ESC channel configurations as one atomic NVS blob. */
static esp_err_t save_esc_configs(void)
{
    esc_persistent_config_t stored = {
        .version = ESC_CONFIG_VERSION,
    };
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        stored.channel[index].signal_gpio =
            s_esc_configs[index].signal_gpio;
        stored.channel[index].pwm_frequency_hz =
            s_esc_configs[index].pwm_frequency_hz;
        stored.channel[index].min_pulse_us =
            s_esc_configs[index].min_pulse_us;
        stored.channel[index].max_pulse_us =
            s_esc_configs[index].max_pulse_us;
        stored.channel[index].calibration_high_time_ms =
            s_esc_configs[index].calibration_high_time_ms;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(ESC_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs, ESC_NVS_CONFIG_KEY, &stored, sizeof(stored));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

/** @brief Restore all valid ESC settings, or retain defaults. */
static esp_err_t load_esc_configs(void)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(ESC_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    esc_persistent_config_t stored = {0};
    size_t size = sizeof(stored);
    err = nvs_get_blob(nvs, ESC_NVS_CONFIG_KEY, &stored, &size);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    if (size != sizeof(stored) || stored.version != ESC_CONFIG_VERSION) {
        ESP_LOGW(APP_TAG, "Ignoring incompatible persisted ESC settings");
        return ESP_OK;
    }

    xw30a_config_t restored[ESC_COUNT];
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        restored[index] = s_esc_configs[index];
        restored[index].signal_gpio = stored.channel[index].signal_gpio;
        restored[index].pwm_frequency_hz =
            stored.channel[index].pwm_frequency_hz;
        restored[index].min_pulse_us =
            stored.channel[index].min_pulse_us;
        restored[index].max_pulse_us =
            stored.channel[index].max_pulse_us;
        restored[index].calibration_high_time_ms =
            stored.channel[index].calibration_high_time_ms;
        if (!is_valid_esc_config(&restored[index])) {
            ESP_LOGW(APP_TAG, "Ignoring invalid persisted ESC settings");
            return ESP_OK;
        }
        for (size_t previous = 0; previous < index; ++previous) {
            if (restored[previous].signal_gpio ==
                restored[index].signal_gpio) {
                ESP_LOGW(APP_TAG, "Ignoring duplicate persisted ESC GPIO");
                return ESP_OK;
            }
        }
    }
    memcpy(s_esc_configs, restored, sizeof(restored));
    ESP_LOGI(APP_TAG, "Restored persisted XW30A configurations");
    return ESP_OK;
}

/** @brief Create four XW30A outputs, each initially at minimum throttle. */
static esp_err_t initialize_escs(void)
{
    set_default_esc_configs();
    esp_err_t err = load_esc_configs();
    if (err != ESP_OK) {
        ESP_LOGW(APP_TAG, "Failed to load persisted ESC settings: %s",
                 esp_err_to_name(err));
    }
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        err = xw30a_new(&s_esc_configs[index], &s_esc_handles[index]);
        if (err != ESP_OK) {
            ESP_LOGE(APP_TAG, "Failed to initialize ESC %u: %s",
                     (unsigned)(index + 1U), esp_err_to_name(err));
            for (size_t created = 0; created < index; ++created) {
                xw30a_del(s_esc_handles[created]);
                s_esc_handles[created] = NULL;
            }
            return err;
        }
    }
    return ESP_OK;
}

/** @brief Return one XW30A channel's active settings to the web API. */
static esp_err_t get_remote_esc_config(
    esp32_wifi_drone_remote_esc_config_t *config,
    void *context)
{
    (void)context;
    if (config == NULL || config->index >= ESC_COUNT ||
        s_esc_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const xw30a_config_t *active = &s_esc_configs[config->index];
    config->signal_gpio = active->signal_gpio;
    config->pwm_frequency_hz = active->pwm_frequency_hz;
    config->min_pulse_us = active->min_pulse_us;
    config->max_pulse_us = active->max_pulse_us;
    config->calibration_high_time_ms = active->calibration_high_time_ms;
    xSemaphoreGive(s_esc_mutex);
    return ESP_OK;
}

/** @brief Recreate and persist one XW30A channel with submitted settings. */
static esp_err_t set_remote_esc_config(
    const esp32_wifi_drone_remote_esc_config_t *config,
    void *context)
{
    (void)context;
    if (config == NULL || config->index >= ESC_COUNT ||
        s_esc_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    xw30a_config_t replacement = {
        .signal_gpio = config->signal_gpio,
        .mcpwm_group_id = config->index < 2U ? 0 : 1,
        .pwm_frequency_hz = config->pwm_frequency_hz,
        .min_pulse_us = config->min_pulse_us,
        .max_pulse_us = config->max_pulse_us,
        .calibration_high_time_ms = config->calibration_high_time_ms,
    };
    if (!is_valid_esc_config(&replacement)) {
        ESP_LOGW(APP_TAG, "Rejected invalid ESC %u configuration (GPIO %u)",
                 (unsigned)(config->index + 1U), config->signal_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        if (index != config->index &&
            s_esc_configs[index].signal_gpio == replacement.signal_gpio) {
            ESP_LOGW(APP_TAG, "GPIO %u is already assigned to ESC %u",
                     replacement.signal_gpio, (unsigned)(index + 1U));
            xSemaphoreGive(s_esc_mutex);
            return ESP_ERR_INVALID_STATE;
        }
    }

    const uint8_t index = config->index;
    const xw30a_config_t previous = s_esc_configs[index];
    xw30a_del(s_esc_handles[index]);
    s_esc_handles[index] = NULL;
    esp_err_t err = xw30a_new(&replacement, &s_esc_handles[index]);
    if (err == ESP_OK) {
        s_esc_configs[index] = replacement;
        err = save_esc_configs();
    }
    if (err != ESP_OK) {
        if (s_esc_handles[index] != NULL) {
            xw30a_del(s_esc_handles[index]);
            s_esc_handles[index] = NULL;
        }
        s_esc_configs[index] = previous;
        esp_err_t restore_err =
            xw30a_new(&previous, &s_esc_handles[index]);
        if (restore_err != ESP_OK) {
            ESP_LOGE(APP_TAG, "Failed to restore ESC %u: %s",
                     (unsigned)(index + 1U),
                     esp_err_to_name(restore_err));
        }
    }
    xSemaphoreGive(s_esc_mutex);
    return err;
}

/** @brief Apply a manual browser throttle command to one XW30A channel. */
static esp_err_t set_remote_esc_throttle(
    uint8_t index,
    float throttle,
    void *context)
{
    (void)context;
    if (index >= ESC_COUNT || throttle < 0.0f || throttle > 1.0f ||
        s_esc_mutex == NULL || s_flight_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    bool flight_armed = s_flight_controller.armed;
    xSemaphoreGive(s_flight_mutex);
    if (flight_armed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = s_programming_esc_index >= 0
        ? ESP_ERR_INVALID_STATE
        : s_esc_handles[index] == NULL
        ? ESP_ERR_INVALID_STATE
        : xw30a_control_set_throttle(s_esc_handles[index], throttle);
    xSemaphoreGive(s_esc_mutex);
    return err;
}

/** @brief Dispatch one guided audible-programming operation to an ESC. */
static esp_err_t program_remote_esc(
    uint8_t index,
    esp32_wifi_drone_remote_esc_program_action_t action,
    uint8_t selection,
    void *context)
{
    (void)context;
    if (index >= ESC_COUNT || s_esc_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (action == ESP32_WIFI_ESC_PROGRAM_BEGIN) {
        if (s_flight_mutex == NULL ||
            xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        bool flight_armed = s_flight_controller.armed;
        xSemaphoreGive(s_flight_mutex);
        if (flight_armed) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    xw30a_handle_t handle = s_esc_handles[index];
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (handle != NULL) {
        if (action == ESP32_WIFI_ESC_PROGRAM_BEGIN &&
            s_programming_esc_index >= 0) {
            xSemaphoreGive(s_esc_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        if (action != ESP32_WIFI_ESC_PROGRAM_BEGIN &&
            s_programming_esc_index != index) {
            xSemaphoreGive(s_esc_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        switch (action) {
        case ESP32_WIFI_ESC_PROGRAM_BEGIN:
            for (size_t channel = 0; channel < ESC_COUNT; ++channel) {
                err = xw30a_control_stop(s_esc_handles[channel]);
                if (err != ESP_OK) {
                    break;
                }
            }
            if (err != ESP_OK) {
                break;
            }
            err = xw30a_setup_begin_programming(handle);
            if (err == ESP_OK) {
                s_programming_esc_index = index;
            }
            break;
        case ESP32_WIFI_ESC_PROGRAM_SELECT_ITEM:
            err = xw30a_setup_select_program_item(
                handle, (xw30a_program_item_t)selection);
            if (err == ESP_OK &&
                selection == XW30A_PROGRAM_ITEM_EXIT) {
                s_programming_esc_index = -1;
            }
            break;
        case ESP32_WIFI_ESC_PROGRAM_STORE_VALUE:
            err = xw30a_setup_store_program_value(
                handle, (xw30a_program_value_t)selection);
            break;
        case ESP32_WIFI_ESC_PROGRAM_CONTINUE:
            err = xw30a_setup_finish_programming_value(handle, true);
            break;
        case ESP32_WIFI_ESC_PROGRAM_EXIT:
            err = xw30a_setup_finish_programming_value(handle, false);
            if (err == ESP_OK) {
                s_programming_esc_index = -1;
            }
            break;
        case ESP32_WIFI_ESC_PROGRAM_CANCEL:
            err = xw30a_setup_cancel(handle);
            if (err == ESP_OK) {
                s_programming_esc_index = -1;
            }
            break;
        default:
            err = ESP_ERR_INVALID_ARG;
            break;
        }
    }
    xSemaphoreGive(s_esc_mutex);
    return err;
}

/** @brief Execute the manual's guided throttle-range setting sequence. */
static esp_err_t set_remote_esc_throttle_range(
    uint8_t index,
    esp32_wifi_drone_remote_esc_throttle_range_action_t action,
    void *context)
{
    (void)context;
    if (index >= ESC_COUNT || s_esc_mutex == NULL ||
        action > ESP32_WIFI_ESC_THROTTLE_RANGE_CANCEL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (action == ESP32_WIFI_ESC_THROTTLE_RANGE_BEGIN) {
        if (s_flight_mutex == NULL ||
            xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        const bool flight_armed = s_flight_controller.armed;
        xSemaphoreGive(s_flight_mutex);
        if (flight_armed) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(3500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    xw30a_handle_t handle = s_esc_handles[index];
    if (handle != NULL) {
        if (action == ESP32_WIFI_ESC_THROTTLE_RANGE_BEGIN) {
            if (s_programming_esc_index >= 0) {
                err = ESP_ERR_INVALID_STATE;
            } else {
                err = ESP_OK;
                for (size_t channel = 0; channel < ESC_COUNT; ++channel) {
                    if (channel != index) {
                        err = xw30a_control_stop(s_esc_handles[channel]);
                        if (err != ESP_OK) {
                            break;
                        }
                    }
                }
                if (err == ESP_OK) {
                    err = xw30a_setup_begin_calibration(handle);
                }
                if (err == ESP_OK) {
                    s_programming_esc_index = index;
                }
            }
        } else if (s_programming_esc_index != index) {
            err = ESP_ERR_INVALID_STATE;
        } else if (action ==
                   ESP32_WIFI_ESC_THROTTLE_RANGE_LATCH_MINIMUM) {
            err = xw30a_setup_finish_calibration(handle);
            if (err == ESP_OK) {
                s_programming_esc_index = -1;
            }
        } else {
            err = xw30a_setup_cancel(handle);
            if (err == ESP_OK) {
                s_programming_esc_index = -1;
            }
        }
    }
    xSemaphoreGive(s_esc_mutex);
    return err;
}

/** @brief Clamp a normalized controller value to its supported interval. */
static float clamp_normalized(float value)
{
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

/**
 * @brief Convert browser stick and arming requests into flight setpoints.
 *
 * The left stick controls yaw and collective throttle. Its center position is
 * the configured open-loop hover estimate (50 percent). The right stick
 * controls desired roll and pitch angles.
 */
static esp_err_t handle_flight_request(
    const char *uri,
    const uint8_t *body,
    size_t body_length,
    void *context)
{
    (void)context;
    if (uri == NULL || body == NULL || body_length >= 96U ||
        s_flight_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char json[96];
    memcpy(json, body, body_length);
    json[body_length] = '\0';
    if (xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    esp_err_t err = ESP_OK;
    if (strcmp(uri, "/api/left-stick") == 0) {
        float x;
        float y;
        if (sscanf(json, "{\"x\":%f,\"y\":%f}", &x, &y) != 2) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            s_movement_command.yaw = clamp_normalized(x);
            s_movement_command.throttle =
                (clamp_normalized(y) + 1.0f) * 0.5f;
            err = flight_controller_set_movement(
                &s_flight_controller, &s_movement_command, now_us);
        }
    } else if (strcmp(uri, "/api/right-stick") == 0) {
        float x;
        float y;
        if (sscanf(json, "{\"x\":%f,\"y\":%f}", &x, &y) != 2) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            s_movement_command.roll = clamp_normalized(x);
            s_movement_command.pitch = clamp_normalized(y);
            err = flight_controller_set_movement(
                &s_flight_controller, &s_movement_command, now_us);
        }
    } else if (strcmp(uri, "/api/settings") == 0) {
        if (strstr(json, "\"heartbeat\":true") != NULL) {
            err = flight_controller_heartbeat(&s_flight_controller, now_us);
        } else if (strstr(json, "\"align_imu\":true") != NULL) {
            err = flight_controller_align_level(&s_flight_controller);
        } else if (strstr(json, "\"armed\":true") != NULL) {
            err = s_programming_esc_index < 0 &&
                    s_movement_command.throttle <= 0.05f
                ? flight_controller_set_armed(
                    &s_flight_controller, true, now_us)
                : ESP_ERR_INVALID_STATE;
        } else if (strstr(json, "\"armed\":false") != NULL) {
            err = flight_controller_set_armed(
                &s_flight_controller, false, now_us);
            memset(&s_movement_command, 0, sizeof(s_movement_command));
        }
    }
    xSemaphoreGive(s_flight_mutex);
    return err;
}

/** @brief Return whether a persisted ISM330 configuration is supported. */
static bool is_valid_persistent_imu_config(
    const imu_persistent_config_t *config)
{
    return config != NULL &&
           config->version == IMU_CONFIG_VERSION &&
           config->accelerometer_odr <= ISM330DLC_ODR_6660_HZ &&
           config->gyroscope_odr <= ISM330DLC_ODR_6660_HZ &&
           config->accelerometer_full_scale <= ISM330DLC_ACCEL_FS_8G &&
           config->gyroscope_full_scale <= ISM330DLC_GYRO_FS_2000_DPS;
}

/** @brief Save the active ISM330 configuration to non-volatile storage. */
static esp_err_t save_imu_config(const esp32_ism330dlc_t *imu)
{
    if (imu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const imu_persistent_config_t stored = {
        .version = IMU_CONFIG_VERSION,
        .accelerometer_odr = imu->accel_odr,
        .gyroscope_odr = imu->gyro_odr,
        .accelerometer_full_scale = imu->accel_full_scale,
        .gyroscope_full_scale = imu->gyro_full_scale,
    };
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(IMU_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs, IMU_NVS_CONFIG_KEY, &stored, sizeof(stored));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

/**
 * @brief Load persisted settings into an ISM330 instance.
 *
 * Missing, obsolete, or invalid data leaves the driver's defaults unchanged.
 */
static esp_err_t load_imu_config(esp32_ism330dlc_t *imu)
{
    if (imu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(IMU_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    imu_persistent_config_t stored = {0};
    size_t stored_size = sizeof(stored);
    err = nvs_get_blob(nvs, IMU_NVS_CONFIG_KEY, &stored, &stored_size);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    if (stored_size != sizeof(stored) ||
        !is_valid_persistent_imu_config(&stored)) {
        ESP_LOGW(APP_TAG, "Ignoring invalid persisted ISM330 configuration");
        return ESP_OK;
    }

    imu->accel_odr = stored.accelerometer_odr;
    imu->gyro_odr = stored.gyroscope_odr;
    imu->accel_full_scale = stored.accelerometer_full_scale;
    imu->gyro_full_scale = stored.gyroscope_full_scale;
    ESP_LOGI(APP_TAG, "Restored persisted ISM330 configuration");
    return ESP_OK;
}

/**
 * @brief Copy the latest IMU sample into a webserver telemetry response.
 */
static esp_err_t provide_remote_telemetry(
    esp32_wifi_drone_remote_telemetry_t *telemetry,
    void *context)
{
    (void)context;
    if (telemetry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_telemetry_lock);
    *telemetry = s_telemetry;
    portEXIT_CRITICAL(&s_telemetry_lock);
    return ESP_OK;
}

/** @brief Return the active ISM330DLC acquisition settings. */
static esp_err_t get_remote_imu_config(
    esp32_wifi_drone_remote_imu_config_t *config,
    void *context)
{
    esp32_ism330dlc_t *imu = context;
    if (config == NULL || imu == NULL || s_imu_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *config = (esp32_wifi_drone_remote_imu_config_t) {
        .accelerometer_odr = imu->accel_odr,
        .gyroscope_odr = imu->gyro_odr,
        .accelerometer_full_scale = imu->accel_full_scale,
        .gyroscope_full_scale = imu->gyro_full_scale,
    };
    xSemaphoreGive(s_imu_mutex);
    return ESP_OK;
}

/** @brief Apply web-submitted acquisition settings to the ISM330DLC. */
static esp_err_t set_remote_imu_config(
    const esp32_wifi_drone_remote_imu_config_t *config,
    void *context)
{
    esp32_ism330dlc_t *imu = context;
    if (config == NULL || imu == NULL || s_imu_mutex == NULL ||
        config->accelerometer_odr > ISM330DLC_ODR_6660_HZ ||
        config->gyroscope_odr > ISM330DLC_ODR_6660_HZ ||
        config->accelerometer_full_scale > ISM330DLC_ACCEL_FS_8G ||
        config->gyroscope_full_scale > ISM330DLC_GYRO_FS_2000_DPS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp32_ism330dlc_odr_t old_accel_odr = imu->accel_odr;
    esp32_ism330dlc_odr_t old_gyro_odr = imu->gyro_odr;
    esp32_ism330dlc_accel_fs_t old_accel_scale = imu->accel_full_scale;
    esp32_ism330dlc_gyro_fs_t old_gyro_scale = imu->gyro_full_scale;
    imu->accel_odr = config->accelerometer_odr;
    imu->gyro_odr = config->gyroscope_odr;
    imu->accel_full_scale = config->accelerometer_full_scale;
    imu->gyro_full_scale = config->gyroscope_full_scale;

    esp_err_t err = esp32_ism330dlc_init(imu);
    if (err == ESP_OK) {
        err = save_imu_config(imu);
    }
    if (err != ESP_OK) {
        imu->accel_odr = old_accel_odr;
        imu->gyro_odr = old_gyro_odr;
        imu->accel_full_scale = old_accel_scale;
        imu->gyro_full_scale = old_gyro_scale;
        esp32_ism330dlc_init(imu);
    }
    xSemaphoreGive(s_imu_mutex);
    return err;
}

/** @brief Copy web-facing gains into the internal controller configuration. */
static void apply_pid_config(
    flight_controller_config_t *destination,
    const esp32_wifi_drone_remote_pid_config_t *source)
{
    destination->roll.kp = source->roll_kp;
    destination->roll.ki = source->roll_ki;
    destination->roll.kd = source->roll_kd;
    destination->pitch.kp = source->pitch_kp;
    destination->pitch.ki = source->pitch_ki;
    destination->pitch.kd = source->pitch_kd;
    destination->yaw_rate.kp = source->yaw_kp;
    destination->yaw_rate.ki = source->yaw_ki;
    destination->yaw_rate.kd = source->yaw_kd;
    destination->vertical_velocity.kp = source->altitude_kp;
    destination->vertical_velocity.ki = source->altitude_ki;
    destination->vertical_velocity.kd = source->altitude_kd;
    destination->armed_idle_throttle = source->armed_idle_throttle;
    destination->stabilize_at_minimum_throttle =
        source->stabilize_at_minimum_throttle;
}

/** @brief Convert internal controller gains to their web-facing structure. */
static esp32_wifi_drone_remote_pid_config_t export_pid_config(
    const flight_controller_config_t *source)
{
    return (esp32_wifi_drone_remote_pid_config_t) {
        .roll_kp = source->roll.kp,
        .roll_ki = source->roll.ki,
        .roll_kd = source->roll.kd,
        .pitch_kp = source->pitch.kp,
        .pitch_ki = source->pitch.ki,
        .pitch_kd = source->pitch.kd,
        .yaw_kp = source->yaw_rate.kp,
        .yaw_ki = source->yaw_rate.ki,
        .yaw_kd = source->yaw_rate.kd,
        .altitude_kp = source->vertical_velocity.kp,
        .altitude_ki = source->vertical_velocity.ki,
        .altitude_kd = source->vertical_velocity.kd,
        .armed_idle_throttle = source->armed_idle_throttle,
        .stabilize_at_minimum_throttle =
            source->stabilize_at_minimum_throttle,
    };
}

/** @brief Restore valid PID gains from NVS into a default configuration. */
static esp_err_t load_pid_config(flight_controller_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PID_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    pid_persistent_config_t stored = {0};
    size_t size = sizeof(stored);
    err = nvs_get_blob(nvs, PID_NVS_CONFIG_KEY, &stored, &size);
    nvs_close(nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    if (size != sizeof(stored) || stored.version != PID_CONFIG_VERSION ||
        !is_valid_pid_config(&stored.gains)) {
        ESP_LOGW(APP_TAG, "Ignoring invalid persisted PID gains");
        return ESP_OK;
    }
    apply_pid_config(config, &stored.gains);
    ESP_LOGI(APP_TAG, "Restored persisted flight-controller PID gains");
    return ESP_OK;
}

/** @brief Load control-timeout settings saved by the Wi-Fi AP page. */
static void load_timeout_config(flight_controller_config_t *config)
{
    nvs_handle_t nvs = 0;
    if (nvs_open("drone_remote", NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    uint32_t timeout_ms;
    uint16_t throttle_permille;
    if (nvs_get_u32(nvs, "ctrl_timeout", &timeout_ms) == ESP_OK &&
        timeout_ms > 0U) {
        config->command_timeout_us = timeout_ms * 1000U;
    }
    if (nvs_get_u16(nvs, "timeout_thr", &throttle_permille) == ESP_OK &&
        throttle_permille <= 1000U) {
        config->timeout_throttle = (float)throttle_permille / 1000.0f;
    }
    nvs_close(nvs);
}

/** @brief Return active flight-controller gains to the settings API. */
static esp_err_t get_remote_pid_config(
    esp32_wifi_drone_remote_pid_config_t *config,
    void *context)
{
    (void)context;
    if (config == NULL || s_flight_mutex == NULL ||
        xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ESP_ERR_INVALID_ARG;
    }
    *config = export_pid_config(&s_flight_controller.config);
    xSemaphoreGive(s_flight_mutex);
    return ESP_OK;
}

/** @brief Apply and persist web-submitted PID gains while disarmed. */
static esp_err_t set_remote_pid_config(
    const esp32_wifi_drone_remote_pid_config_t *config,
    void *context)
{
    (void)context;
    if (!is_valid_pid_config(config) || s_flight_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_flight_controller.armed) {
        xSemaphoreGive(s_flight_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    const flight_controller_config_t previous = s_flight_controller.config;
    apply_pid_config(&s_flight_controller.config, config);
    const pid_persistent_config_t stored = {
        .version = PID_CONFIG_VERSION,
        .gains = *config,
    };
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(PID_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, PID_NVS_CONFIG_KEY,
                           &stored, sizeof(stored));
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err == ESP_OK) {
        s_flight_controller.roll_integral = 0.0f;
        s_flight_controller.pitch_integral = 0.0f;
        s_flight_controller.yaw_integral = 0.0f;
        s_flight_controller.previous_roll_error = 0.0f;
        s_flight_controller.previous_pitch_error = 0.0f;
        s_flight_controller.previous_yaw_error = 0.0f;
    } else {
        s_flight_controller.config = previous;
    }
    if (nvs != 0) {
        nvs_close(nvs);
    }
    xSemaphoreGive(s_flight_mutex);
    return err;
}

static esp_err_t initialize_imu(esp32_ism330dlc_t *imu)
{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_GPIO,
        .scl_io_num = IMU_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Failed to initialize I2C bus: %s",
                 esp_err_to_name(err));
        return err;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDRESS,
        .scl_speed_hz = IMU_I2C_FREQUENCY,
    };
    err = i2c_master_bus_add_device(bus, &device_config, &device);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Failed to add ISM330DLC to I2C bus: %s",
                 esp_err_to_name(err));
        return err;
    }

    *imu = (esp32_ism330dlc_t)ESP32_ISM330DLC_DEFAULT_CONFIG(
        esp32_ism330dlc_i2c_read, esp32_ism330dlc_i2c_write, device);

    err = load_imu_config(imu);
    if (err != ESP_OK) {
        ESP_LOGW(APP_TAG, "Failed to load persisted ISM330 configuration: %s",
                 esp_err_to_name(err));
    }
    for (unsigned int attempt = 1;
         attempt <= CONFIG_FLIGHT_IMU_INIT_ATTEMPTS; ++attempt) {
        err = esp32_ism330dlc_init(imu);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(APP_TAG, "ISM330DLC initialization attempt %u/%u failed: %s",
                 attempt, CONFIG_FLIGHT_IMU_INIT_ATTEMPTS,
                 esp_err_to_name(err));
        if (attempt < CONFIG_FLIGHT_IMU_INIT_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_FLIGHT_IMU_RETRY_DELAY_MS));
        }
    }
    return err;
}

/**
 * @brief Convert the board-mounted sensor axes to the aircraft body frame.
 *
 * Body X points forward, body Y points right, and body Z points up. The IMU
 * is mounted with its X axis reversed, so accelerometer and gyro X values
 * receive the same sign correction while Y remains native.
 */
static flight_imu_sample_t imu_sample_to_body_frame(
    const esp32_ism330dlc_sample_t *sample)
{
    return (flight_imu_sample_t) {
        .acceleration_g = {
            -sample->acceleration_g.x,
             sample->acceleration_g.y,
             sample->acceleration_g.z,
        },
        .angular_rate_dps = {
            -sample->angular_rate_dps.x,
             sample->angular_rate_dps.y,
             sample->angular_rate_dps.z,
        },
    };
}

/** @brief Run stabilization and update PWM on the non-Wi-Fi core. */
static void flight_control_task(void *context)
{
    (void)context;
    esp_err_t err;
    uint32_t sample_count = 0;
    int64_t previous_sample_us = esp_timer_get_time();
    bool flight_was_armed = false;

    while (true) {
        esp32_ism330dlc_sample_t sample;

        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        err = esp32_ism330dlc_read(&s_imu, &sample);
        xSemaphoreGive(s_imu_mutex);
        if (err == ESP_OK) {
            const int64_t now_us = esp_timer_get_time();
            const flight_imu_sample_t flight_sample =
                imu_sample_to_body_frame(&sample);
            float dt_seconds =
                (float)(now_us - previous_sample_us) / 1000000.0f;
            previous_sample_us = now_us;
            portENTER_CRITICAL(&s_telemetry_lock);
            s_telemetry = (esp32_wifi_drone_remote_telemetry_t) {
                .acceleration_x = flight_sample.acceleration_g[0],
                .acceleration_y = flight_sample.acceleration_g[1],
                .acceleration_z = flight_sample.acceleration_g[2],
                .gyroscope_x = flight_sample.angular_rate_dps[0],
                .gyroscope_y = flight_sample.angular_rate_dps[1],
                .gyroscope_z = flight_sample.angular_rate_dps[2],
            };
            portEXIT_CRITICAL(&s_telemetry_lock);
            flight_controller_output_t output;
            if (dt_seconds > 0.0f && dt_seconds <= 0.1f &&
                xSemaphoreTake(s_flight_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                err = flight_controller_update(
                    &s_flight_controller, &flight_sample, dt_seconds,
                    (uint64_t)now_us, &output);
                xSemaphoreGive(s_flight_mutex);
                if (err == ESP_OK &&
                    (output.armed || flight_was_armed) &&
                    xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    for (size_t index = 0; index < ESC_COUNT; ++index) {
                        esp_err_t motor_err = xw30a_control_set_throttle(
                            s_esc_handles[index], output.motor[index]);
                        if (motor_err != ESP_OK) {
                            ESP_LOGE(APP_TAG, "ESC %u control failed: %s",
                                     (unsigned)(index + 1U),
                                     esp_err_to_name(motor_err));
                        }
                    }
                    xSemaphoreGive(s_esc_mutex);
                }
                if (err == ESP_OK) {
                    flight_was_armed = output.armed;
                }
            }
            ++sample_count;
        } else {
            /* Retain armed state and the most recent PWM output. */
            ESP_LOGE(APP_TAG, "Failed to read ISM330DLC: %s",
                     esp_err_to_name(err));
            vTaskDelay(1);
        }

        if ((sample_count % IMU_YIELD_INTERVAL) == 0U) {
            vTaskDelay(1);
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "NVS initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    s_imu_mutex = xSemaphoreCreateMutex();
    s_esc_mutex = xSemaphoreCreateMutex();
    s_flight_mutex = xSemaphoreCreateMutex();
    if (s_imu_mutex == NULL || s_esc_mutex == NULL ||
        s_flight_mutex == NULL) {
        ESP_LOGE(APP_TAG, "Failed to create sensor/control mutexes");
        return;
    }
    flight_controller_config_t flight_config =
        FLIGHT_CONTROLLER_DEFAULT_CONFIG();
    load_timeout_config(&flight_config);
    err = load_pid_config(&flight_config);
    if (err != ESP_OK) {
        ESP_LOGW(APP_TAG, "Failed to load persisted PID gains: %s",
                 esp_err_to_name(err));
    }
    err = flight_controller_init(&s_flight_controller, &flight_config);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Flight controller initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }
    err = initialize_imu(&s_imu);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "ISM330DLC initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }
    err = initialize_escs();
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "XW30A initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    esp32_wifi_drone_remote_config_t remote_config =
        ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG();
    remote_config.telemetry_handler = provide_remote_telemetry;
    remote_config.imu_get_handler = get_remote_imu_config;
    remote_config.imu_set_handler = set_remote_imu_config;
    remote_config.imu_context = &s_imu;
    remote_config.pid_get_handler = get_remote_pid_config;
    remote_config.pid_set_handler = set_remote_pid_config;
    remote_config.esc_get_handler = get_remote_esc_config;
    remote_config.esc_set_handler = set_remote_esc_config;
    remote_config.esc_throttle_handler = set_remote_esc_throttle;
    remote_config.esc_program_handler = program_remote_esc;
    remote_config.esc_throttle_range_handler =
        set_remote_esc_throttle_range;
    remote_config.api_handler = handle_flight_request;
    err = esp32_wifi_drone_remote_start(&remote_config);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Wi-Fi drone remote initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    if (xTaskCreatePinnedToCore(flight_control_task, "flight-control",
                                CONTROL_TASK_STACK, NULL,
                                CONTROL_TASK_PRIORITY, NULL,
                                CONTROL_CORE) != pdPASS) {
        ESP_LOGE(APP_TAG, "Failed to create core-%d flight-control task",
                 CONTROL_CORE);
    }
}
