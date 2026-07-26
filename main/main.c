#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "esp32-ism330dlc.h"
#include "esp32-esc-xw30a.h"
#include "esp32-wifi-drone-remote.h"

#define IMU_I2C_PORT       I2C_NUM_0
#define IMU_I2C_SDA_GPIO   GPIO_NUM_8
#define IMU_I2C_SCL_GPIO   GPIO_NUM_9
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

static const char *APP_TAG = "flight-controller";
static portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static esp32_wifi_drone_remote_telemetry_t s_telemetry;
static SemaphoreHandle_t s_imu_mutex;
static esp32_ism330dlc_t s_imu;
static SemaphoreHandle_t s_esc_mutex;
static xw30a_handle_t s_esc_handles[ESC_COUNT];
static xw30a_config_t s_esc_configs[ESC_COUNT];
static int s_programming_esc_index = -1;

/** GPIO assignments used when no persisted ESC configuration exists. */
static const gpio_num_t s_default_esc_gpios[ESC_COUNT] = {
    GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7,
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

    nvs_handle_t nvs;
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
    nvs_handle_t nvs;
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
    if (!is_valid_esc_config(&replacement) ||
        xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t index = 0; index < ESC_COUNT; ++index) {
        if (index != config->index &&
            s_esc_configs[index].signal_gpio == replacement.signal_gpio) {
            xSemaphoreGive(s_esc_mutex);
            return ESP_ERR_INVALID_ARG;
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
        s_esc_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_esc_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = s_esc_handles[index] == NULL
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
    return esp32_ism330dlc_init(imu);
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
    if (s_imu_mutex == NULL || s_esc_mutex == NULL) {
        ESP_LOGE(APP_TAG, "Failed to create sensor/control mutexes");
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
    remote_config.esc_get_handler = get_remote_esc_config;
    remote_config.esc_set_handler = set_remote_esc_config;
    remote_config.esc_throttle_handler = set_remote_esc_throttle;
    remote_config.esc_program_handler = program_remote_esc;
    err = esp32_wifi_drone_remote_start(&remote_config);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Wi-Fi drone remote initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    uint32_t sample_count = 0;

    while (true) {
        esp32_ism330dlc_sample_t sample;

        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        err = esp32_ism330dlc_read(&s_imu, &sample);
        xSemaphoreGive(s_imu_mutex);
        if (err == ESP_OK) {
            portENTER_CRITICAL(&s_telemetry_lock);
            s_telemetry = (esp32_wifi_drone_remote_telemetry_t) {
                .acceleration_x = sample.acceleration_g.x,
                .acceleration_y = sample.acceleration_g.y,
                .acceleration_z = sample.acceleration_g.z,
                .gyroscope_x = sample.angular_rate_dps.x,
                .gyroscope_y = sample.angular_rate_dps.y,
                .gyroscope_z = sample.angular_rate_dps.z,
            };
            portEXIT_CRITICAL(&s_telemetry_lock);
            ++sample_count;
        } else {
            ESP_LOGE(APP_TAG, "Failed to read ISM330DLC: %s",
                     esp_err_to_name(err));
            vTaskDelay(1);
        }

        /* Give the idle task occasional CPU time without pacing every read. */
        if ((sample_count % IMU_YIELD_INTERVAL) == 0U) {
            vTaskDelay(1);
        }
    }
}
