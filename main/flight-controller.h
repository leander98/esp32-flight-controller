/**
 * @file flight-controller.h
 * @brief Attitude stabilization and quad-X motor mixing.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} flight_pid_config_t;

typedef struct {
    flight_pid_config_t roll;
    flight_pid_config_t pitch;
    flight_pid_config_t yaw_rate;
    float complementary_accelerometer_weight;
    float maximum_tilt_degrees;
    float maximum_yaw_rate_dps;
    uint32_t command_timeout_us;
} flight_controller_config_t;

typedef struct {
    float acceleration_g[3];
    float angular_rate_dps[3];
} flight_imu_sample_t;

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} flight_movement_command_t;

typedef struct {
    float roll_degrees;
    float pitch_degrees;
    float motor[4];
    bool armed;
} flight_controller_output_t;

typedef struct {
    flight_controller_config_t config;
    flight_movement_command_t command;
    float roll_degrees;
    float pitch_degrees;
    float roll_integral;
    float pitch_integral;
    float yaw_integral;
    float previous_roll_error;
    float previous_pitch_error;
    float previous_yaw_error;
    uint64_t last_command_us;
    bool attitude_initialized;
    bool armed;
} flight_controller_t;

#define FLIGHT_CONTROLLER_DEFAULT_CONFIG() {                              \
    .roll = { 0.018f, 0.004f, 0.0025f, 20.0f, 0.30f },                    \
    .pitch = { 0.018f, 0.004f, 0.0025f, 20.0f, 0.30f },                   \
    .yaw_rate = { 0.003f, 0.001f, 0.0f, 50.0f, 0.20f },                   \
    .complementary_accelerometer_weight = 0.02f,                           \
    .maximum_tilt_degrees = 20.0f,                                        \
    .maximum_yaw_rate_dps = 120.0f,                                       \
    .command_timeout_us = 750000U,                                        \
}

esp_err_t flight_controller_init(
    flight_controller_t *controller,
    const flight_controller_config_t *config);

esp_err_t flight_controller_set_armed(
    flight_controller_t *controller, bool armed, uint64_t now_us);

esp_err_t flight_controller_set_movement(
    flight_controller_t *controller,
    const flight_movement_command_t *command,
    uint64_t now_us);

esp_err_t flight_controller_heartbeat(
    flight_controller_t *controller, uint64_t now_us);

esp_err_t flight_controller_update(
    flight_controller_t *controller,
    const flight_imu_sample_t *sample,
    float dt_seconds,
    uint64_t now_us,
    flight_controller_output_t *output);
