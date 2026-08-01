/**
 * @file flight-controller.c
 * @brief Complementary-filter stabilization and quad-X control implementation.
 */
#include "flight-controller.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f
#define STANDARD_GRAVITY_MPS2 9.80665f
#define LEVEL_ALIGNMENT_DWELL_SECONDS 3.0f

static float clampf(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

/** @brief Wrap an angle to the half-open interval [-180, 180). */
static float wrap_degrees(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float pid_step(const flight_pid_config_t *config, float error,
                      float *integral, float *previous_error, float dt)
{
    const float candidate_integral = clampf(
        *integral + error * dt,
        -config->integral_limit, config->integral_limit);
    float derivative = dt > 0.0f ? (error - *previous_error) / dt : 0.0f;
    *previous_error = error;
    float unconstrained =
        config->kp * error + config->ki * candidate_integral +
        config->kd * derivative;
    const bool saturating_high =
        unconstrained > config->output_limit && error > 0.0f;
    const bool saturating_low =
        unconstrained < -config->output_limit && error < 0.0f;
    if (!saturating_high && !saturating_low) {
        *integral = candidate_integral;
    } else {
        unconstrained =
            config->kp * error + config->ki * *integral +
            config->kd * derivative;
    }
    return clampf(unconstrained,
                  -config->output_limit, config->output_limit);
}

static void reset_pid(flight_controller_t *controller)
{
    controller->roll_integral = 0.0f;
    controller->pitch_integral = 0.0f;
    controller->yaw_integral = 0.0f;
    controller->vertical_velocity_integral = 0.0f;
    controller->previous_roll_error = 0.0f;
    controller->previous_pitch_error = 0.0f;
    controller->previous_yaw_error = 0.0f;
    controller->previous_vertical_velocity_error = 0.0f;
}

/** @brief Integrate body acceleration into a deliberately leaky climb rate. */
static float update_vertical_velocity(flight_controller_t *controller,
                                      float ax, float ay, float az,
                                      float dt_seconds)
{
    const float roll = controller->roll_degrees * DEG_TO_RAD;
    const float pitch = controller->pitch_degrees * DEG_TO_RAD;
    const float vertical_specific_force_g =
        -sinf(pitch) * ax + sinf(roll) * cosf(pitch) * ay +
        cosf(roll) * cosf(pitch) * az;
    const float acceleration =
        (vertical_specific_force_g - 1.0f) * STANDARD_GRAVITY_MPS2;
    const float leak = clampf(
        1.0f - controller->config.vertical_velocity_leak_per_second *
                   dt_seconds,
        0.0f, 1.0f);
    controller->vertical_velocity_mps =
        controller->vertical_velocity_mps * leak + acceleration * dt_seconds;
    return controller->vertical_velocity_mps;
}

/**
 * @brief Mix collective and body corrections into Quad-X motor outputs.
 *
 * Order is front-left, front-right, rear-right, rear-left. A positive roll
 * correction increases the left pair; a positive pitch correction increases
 * the front pair. These signs oppose positive measured attitude error because
 * the PID error is desired minus measured.
 */
static void mix_quad_x(float throttle, float roll, float pitch, float yaw,
                       float minimum, flight_controller_output_t *output)
{
    output->motor[0] = clampf(throttle + pitch + roll - yaw, minimum, 1.0f);
    output->motor[1] = clampf(throttle + pitch - roll + yaw, minimum, 1.0f);
    output->motor[2] = clampf(throttle - pitch - roll - yaw, minimum, 1.0f);
    output->motor[3] = clampf(throttle - pitch + roll + yaw, minimum, 1.0f);
}

esp_err_t flight_controller_init(
    flight_controller_t *controller,
    const flight_controller_config_t *config)
{
    if (controller == NULL || config == NULL ||
        config->maximum_tilt_degrees <= 0.0f ||
        config->maximum_yaw_rate_dps <= 0.0f ||
        config->maximum_vertical_speed_mps <= 0.0f ||
        config->hover_throttle <= 0.0f || config->hover_throttle >= 1.0f ||
        config->armed_idle_throttle <= 0.0f ||
        config->armed_idle_throttle >= config->hover_throttle ||
        config->vertical_velocity_leak_per_second < 0.0f ||
        config->command_timeout_us == 0U ||
        config->timeout_throttle < 0.0f ||
        config->timeout_throttle > 1.0f ||
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
        controller->vertical_velocity_mps = 0.0f;
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

esp_err_t flight_controller_align_level(flight_controller_t *controller)
{
    if (controller == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (controller->armed || !controller->attitude_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    controller->level_roll_offset_degrees =
        controller->last_accelerometer_roll_degrees;
    controller->level_pitch_offset_degrees =
        controller->last_accelerometer_pitch_degrees;
    controller->roll_degrees = 0.0f;
    controller->pitch_degrees = 0.0f;
    reset_pid(controller);
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
    const float acceleration_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    const float raw_gyro_magnitude = sqrtf(
        sample->angular_rate_dps[0] * sample->angular_rate_dps[0] +
        sample->angular_rate_dps[1] * sample->angular_rate_dps[1] +
        sample->angular_rate_dps[2] * sample->angular_rate_dps[2]);
    const bool level_candidate = !controller->armed &&
        fabsf(ax) < 0.02f && fabsf(ay) < 0.02f &&
        fabsf(az - 1.0f) < 0.03f && raw_gyro_magnitude < 0.5f;
    controller->level_candidate_seconds = level_candidate
        ? fminf(controller->level_candidate_seconds + dt_seconds,
                LEVEL_ALIGNMENT_DWELL_SECONDS)
        : 0.0f;
    const bool settled_level = controller->level_candidate_seconds >=
        LEVEL_ALIGNMENT_DWELL_SECONDS;
    if (settled_level) {
        const float bias_weight = controller->gyroscope_bias_initialized
            ? clampf(dt_seconds / 5.0f, 0.0f, 1.0f)
            : 1.0f;
        for (size_t axis = 0; axis < 3U; ++axis) {
            controller->gyroscope_bias_dps[axis] += bias_weight *
                (sample->angular_rate_dps[axis] -
                 controller->gyroscope_bias_dps[axis]);
        }
        controller->gyroscope_bias_initialized = true;
    }
    const float gyro_x = sample->angular_rate_dps[0] -
        controller->gyroscope_bias_dps[0];
    const float gyro_y = sample->angular_rate_dps[1] -
        controller->gyroscope_bias_dps[1];
    const float gyro_z = sample->angular_rate_dps[2] -
        controller->gyroscope_bias_dps[2];
    float accel_roll = atan2f(ay, az) * RAD_TO_DEG;
    float accel_pitch =
        atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    controller->last_accelerometer_roll_degrees = accel_roll;
    controller->last_accelerometer_pitch_degrees = accel_pitch;
    if (settled_level) {
        const float weight = clampf(dt_seconds / 20.0f, 0.0f, 1.0f);
        controller->level_roll_offset_degrees += weight *
            (accel_roll - controller->level_roll_offset_degrees);
        controller->level_pitch_offset_degrees += weight *
            (accel_pitch - controller->level_pitch_offset_degrees);
    }
    accel_roll = wrap_degrees(
        accel_roll - controller->level_roll_offset_degrees);
    accel_pitch = wrap_degrees(
        accel_pitch - controller->level_pitch_offset_degrees);

    if (!controller->attitude_initialized) {
        controller->roll_degrees = accel_roll;
        controller->pitch_degrees = accel_pitch;
        controller->attitude_initialized = true;
    } else {
        const float accelerometer_trust = clampf(
            1.0f - fabsf(acceleration_magnitude - 1.0f) / 0.35f,
            0.0f, 1.0f);
        const float weight =
            controller->config.complementary_accelerometer_weight *
            accelerometer_trust;
        const float predicted_roll = wrap_degrees(
            controller->roll_degrees +
            gyro_x * dt_seconds);
        const float predicted_pitch = wrap_degrees(
            controller->pitch_degrees +
            gyro_y * dt_seconds);

        /*
         * Accelerometer Euler angles have two equivalent representations.
         * Select the one nearest the gyro prediction so crossing ±90 degrees
         * of pitch does not suddenly introduce a 180-degree roll jump.
         */
        const float alternate_roll = wrap_degrees(accel_roll + 180.0f);
        const float alternate_pitch = accel_pitch >= 0.0f
            ? 180.0f - accel_pitch
            : -180.0f - accel_pitch;
        const float primary_roll_error =
            wrap_degrees(accel_roll - predicted_roll);
        const float primary_pitch_error =
            wrap_degrees(accel_pitch - predicted_pitch);
        const float alternate_roll_error =
            wrap_degrees(alternate_roll - predicted_roll);
        const float alternate_pitch_error =
            wrap_degrees(alternate_pitch - predicted_pitch);
        const float primary_distance =
            primary_roll_error * primary_roll_error +
            primary_pitch_error * primary_pitch_error;
        const float alternate_distance =
            alternate_roll_error * alternate_roll_error +
            alternate_pitch_error * alternate_pitch_error;
        if (alternate_distance < primary_distance) {
            accel_roll = alternate_roll;
            accel_pitch = alternate_pitch;
        }

        controller->roll_degrees = wrap_degrees(
            predicted_roll +
            weight * wrap_degrees(accel_roll - predicted_roll));
        controller->pitch_degrees = wrap_degrees(
            predicted_pitch +
            weight * wrap_degrees(accel_pitch - predicted_pitch));
    }

    const bool command_timed_out = controller->armed &&
        now_us - controller->last_command_us >
            controller->config.command_timeout_us;
    const float command_throttle = command_timed_out
        ? controller->config.timeout_throttle
        : controller->command.throttle;

    memset(output, 0, sizeof(*output));
    output->roll_degrees = controller->roll_degrees;
    output->pitch_degrees = controller->pitch_degrees;
    output->vertical_velocity_mps = controller->vertical_velocity_mps;
    output->armed = controller->armed;
    if (!controller->armed) {
        return ESP_OK;
    }

    /*
     * Estimate world-up acceleration from body acceleration and the filtered
     * attitude. Yaw is irrelevant for the vertical projection. The leaky
     * integrator limits, but cannot eliminate, accelerometer bias drift.
     */
    output->vertical_velocity_mps = update_vertical_velocity(
        controller, ax, ay, az, dt_seconds);

    /*
     * At minimum stick, keep only the vertical estimator/controller reset.
     * Attitude control remains active at armed idle so the motors can react to
     * a tilted body. Explicit disarm still outputs zero.
     */
    const bool throttle_low = command_throttle <= 0.05f;
    if (throttle_low) {
        controller->vertical_velocity_mps = 0.0f;
        controller->vertical_velocity_integral = 0.0f;
        controller->previous_vertical_velocity_error = 0.0f;
        if (!controller->config.stabilize_at_minimum_throttle) {
            reset_pid(controller);
            return ESP_OK;
        }
    }

    const float desired_roll =
        controller->command.roll * controller->config.maximum_tilt_degrees;
    const float desired_pitch =
        controller->command.pitch * controller->config.maximum_tilt_degrees;
    const float desired_yaw_rate =
        controller->command.yaw * controller->config.maximum_yaw_rate_dps;
    const float roll_correction = pid_step(
        &controller->config.roll,
        wrap_degrees(desired_roll - controller->roll_degrees),
        &controller->roll_integral, &controller->previous_roll_error,
        dt_seconds);
    const float pitch_correction = pid_step(
        &controller->config.pitch,
        wrap_degrees(desired_pitch - controller->pitch_degrees),
        &controller->pitch_integral, &controller->previous_pitch_error,
        dt_seconds);
    const float yaw_correction = pid_step(
        &controller->config.yaw_rate,
        desired_yaw_rate - gyro_z,
        &controller->yaw_integral, &controller->previous_yaw_error,
        dt_seconds);
    float throttle = controller->config.armed_idle_throttle;
    if (!throttle_low) {
        const float desired_vertical_velocity =
            (command_throttle - 0.5f) * 2.0f *
            controller->config.maximum_vertical_speed_mps;
        const float vertical_correction = pid_step(
            &controller->config.vertical_velocity,
            desired_vertical_velocity - controller->vertical_velocity_mps,
            &controller->vertical_velocity_integral,
            &controller->previous_vertical_velocity_error, dt_seconds);
        throttle = clampf(
            controller->config.hover_throttle + vertical_correction,
            controller->config.armed_idle_throttle, 1.0f);
    }

    mix_quad_x(throttle, roll_correction, pitch_correction, yaw_correction,
               controller->config.armed_idle_throttle, output);
    return ESP_OK;
}
