#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp32-ism330dlc.h"

#define IMU_I2C_PORT       I2C_NUM_0
#define IMU_I2C_SDA_GPIO   GPIO_NUM_8
#define IMU_I2C_SCL_GPIO   GPIO_NUM_9
#define IMU_I2C_FREQUENCY  400000U
#define IMU_I2C_ADDRESS    0x6AU
#define IMU_SAMPLE_PERIOD  pdMS_TO_TICKS(1000U)

static const char *APP_TAG = "flight-controller";

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

    return esp32_ism330dlc_init(imu);
}

void app_main(void)
{
    esp32_ism330dlc_t imu;
    esp_err_t err = initialize_imu(&imu);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "ISM330DLC initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    while (true) {
        esp32_ism330dlc_sample_t sample;

        err = esp32_ism330dlc_read(&imu, &sample);
        if (err == ESP_OK) {
            ESP_LOGI(APP_TAG,
                     "accel [g]: %.3f, %.3f, %.3f | "
                     "gyro [dps]: %.3f, %.3f, %.3f | temp: %.2f C",
                     sample.acceleration_g.x,
                     sample.acceleration_g.y,
                     sample.acceleration_g.z,
                     sample.angular_rate_dps.x,
                     sample.angular_rate_dps.y,
                     sample.angular_rate_dps.z,
                     sample.temperature_c);
        } else {
            ESP_LOGE(APP_TAG, "Failed to read ISM330DLC: %s",
                     esp_err_to_name(err));
        }

        vTaskDelay(IMU_SAMPLE_PERIOD);
    }
}
