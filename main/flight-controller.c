/**
 * @file flight-controller.c
 * @brief Complementary-filter stabilization and quad-X control implementation.
 */
#include "flight-controller.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.2957795f

static float clampf(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

static float pid_step(const flight_pid_config_t *config, float error,
                      float *integral, float *previous_error, float dt)
{
    *integral = clampf(*integral + error * dt,
                       -config->integral_limit, config->integral_limit);
    float derivative = dt > 0.0f ? (error - *previous_error) / dt : 0.0f;
    *previous_error = error;
    return clampf(config->kp * error + config->ki * *integral +
                      config->kd * derivative,
                  -config->output_limit, config->output_limit);
}

static void reset_pid(flight_controller_t *controller)
{
    controller->roll_integral = 0.0f;
    controller->pitch_integral = 0.0f;
    controller->yaw_integral = 0.0f;
    controller->previous_roll_error = 0.0f;
    controller->previous_pitch_error = 0.0f;
    controller->previous_yaw_error = 0.0f;
}

esp_err_t flight_controller_init(
    flight_controller_t *controller,
    const flight_controller_config_t *config)
{
    if (controller == NULL || config == NULL ||
        config->maximum_tilt_degrees <= 0.0f ||
        config->maximum_yaw_rate_dps <= 0.0f ||
        config->command_timeout_us == 0U ||
        config->complementary_accelerometer_weight < 0.0f ||
        config->complementary_accelerometer_weight > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    return ESP_OK;
}

esp_err_t flight_controller_set_armed(
    flight_controller_t *controller, bool armed, uint64_t now_us)
{
    if (controller == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    controller->armed = armed;
    controller->last_command_us = now_us;
    if (!armed) {
        controller->command.throttle = 0.0f;
        reset_pid(controller);
    }
    return ESP_OK;
}

esp_err_t flight_controller_set_movement(
    flight_controller_t *controller,
    const flight_movement_command_t *command,
    uint64_t now_us)
{
    if (controller == NULL || command == NULL ||
        command->throttle < 0.0f || command->throttle > 1.0f ||
        fabsf(command->roll) > 1.0f || fabsf(command->pitch) > 1.0f ||
        fabsf(command->yaw) > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    controller->command = *command;
    controller->last_command_us = now_us;
    return ESP_OK;
}

esp_err_t flight_controller_heartbeat(
    flight_controller_t *controller, uint64_t now_us)
{
    if (controller == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    controller->last_command_us = now_us;
    return ESP_OK;
}

esp_err_t flight_controller_update(
    flight_controller_t *controller,
    const flight_imu_sample_t *sample,
    float dt_seconds,
    uint64_t now_us,
    flight_controller_output_t *output)
{
    if (controller == NULL || sample == NULL || output == NULL ||
        !isfinite(dt_seconds) || dt_seconds <= 0.0f ||
        dt_seconds > 0.1f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float ax = sample->acceleration_g[0];
    const float ay = sample->acceleration_g[1];
    const float az = sample->acceleration_g[2];
    const float accel_roll = atan2f(ay, az) * RAD_TO_DEG;
    const float accel_pitch =
        atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    if (!controller->attitude_initialized) {
        controller->roll_degrees = accel_roll;
        controller->pitch_degrees = accel_pitch;
        controller->attitude_initialized = true;
    } else {
        const float weight =
            controller->config.complementary_accelerometer_weight;
        controller->roll_degrees =
            (1.0f - weight) *
                (controller->roll_degrees +
                 sample->angular_rate_dps[0] * dt_seconds) +
            weight * accel_roll;
        controller->pitch_degrees =
            (1.0f - weight) *
                (controller->pitch_degrees +
                 sample->angular_rate_dps[1] * dt_seconds) +
            weight * accel_pitch;
    }

    if (controller->armed &&
        now_us - controller->last_command_us >
            controller->config.command_timeout_us) {
        controller->armed = false;
        controller->command.throttle = 0.0f;
        reset_pid(controller);
    }

    memset(output, 0, sizeof(*output));
    output->roll_degrees = controller->roll_degrees;
    output->pitch_degrees = controller->pitch_degrees;
    output->armed = controller->armed;
    if (!controller->armed) {
        return ESP_OK;
    }

    const float desired_roll =
        controller->command.roll * controller->config.maximum_tilt_degrees;
    const float desired_pitch =
        controller->command.pitch * controller->config.maximum_tilt_degrees;
    const float desired_yaw_rate =
        controller->command.yaw * controller->config.maximum_yaw_rate_dps;
    const float roll_correction = pid_step(
        &controller->config.roll, desired_roll - controller->roll_degrees,
        &controller->roll_integral, &controller->previous_roll_error,
        dt_seconds);
    const float pitch_correction = pid_step(
        &controller->config.pitch, desired_pitch - controller->pitch_degrees,
        &controller->pitch_integral, &controller->previous_pitch_error,
        dt_seconds);
    const float yaw_correction = pid_step(
        &controller->config.yaw_rate,
        desired_yaw_rate - sample->angular_rate_dps[2],
        &controller->yaw_integral, &controller->previous_yaw_error,
        dt_seconds);
    const float throttle = controller->command.throttle;

    /* Quad-X order: front-left, front-right, rear-right, rear-left. */
    output->motor[0] = clampf(
        throttle + pitch_correction + roll_correction - yaw_correction,
        0.0f, 1.0f);
    output->motor[1] = clampf(
        throttle + pitch_correction - roll_correction + yaw_correction,
        0.0f, 1.0f);
    output->motor[2] = clampf(
        throttle - pitch_correction - roll_correction - yaw_correction,
        0.0f, 1.0f);
    output->motor[3] = clampf(
        throttle - pitch_correction + roll_correction + yaw_correction,
        0.0f, 1.0f);
    return ESP_OK;
}
