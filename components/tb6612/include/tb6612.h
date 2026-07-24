#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One TB6612 motor channel instance.
 *
 * Fill this structure before calling tb6612_init(). Do not modify its
 * configuration fields while the instance is initialized.
 *
 * Each simultaneously active motor must use a different pwm_channel. Motors
 * may share a timer only when their frequency and resolution are identical.
 */
typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t pwm_gpio;
    uint32_t pwm_frequency_hz;
    ledc_mode_t pwm_speed_mode;
    ledc_timer_t pwm_timer;
    ledc_channel_t pwm_channel;
    ledc_timer_bit_t pwm_resolution;

    /*
     * Optional quadrature encoder configuration. Set both pins to GPIO_NUM_NC
     * to disable encoder support for this instance.
     */
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;
    uint32_t encoder_glitch_filter_ns;
    uint32_t encoder_counts_per_revolution;
    bool encoder_invert_direction;

    /* Private runtime state. Initialize these fields to zero. */
    bool _initialized;
    int8_t _direction;
    void *_encoder_context;
} tb6612_t;

/**
 * @brief Speed result returned by tb6612_read_speed().
 */
typedef struct {
    int32_t delta_counts;
    int64_t sample_time_us;
    float counts_per_second;
    float rpm;
    bool rpm_valid;
} tb6612_speed_t;

/**
 * @brief Default configuration for one TB6612 channel.
 *
 * Produces 20 kHz, 10-bit PWM on LEDC low-speed timer 0/channel 0.
 */
#define TB6612_CONFIG_DEFAULT(in1, in2, pwm) { \
    .in1_gpio = (in1),                       \
    .in2_gpio = (in2),                       \
    .pwm_gpio = (pwm),                       \
    .pwm_frequency_hz = 20000,               \
    .pwm_speed_mode = LEDC_LOW_SPEED_MODE,   \
    .pwm_timer = LEDC_TIMER_0,               \
    .pwm_channel = LEDC_CHANNEL_0,           \
    .pwm_resolution = LEDC_TIMER_10_BIT,     \
    .encoder_a_gpio = GPIO_NUM_NC,           \
    .encoder_b_gpio = GPIO_NUM_NC,           \
    .encoder_glitch_filter_ns = 1000,        \
    .encoder_counts_per_revolution = 0,      \
    .encoder_invert_direction = false,       \
    ._initialized = false,                   \
    ._direction = 0,                         \
    ._encoder_context = 0,                   \
}

/**
 * @brief Default motor configuration with a quadrature encoder.
 *
 * counts_per_revolution may be zero when only counts/s is required. Set it to
 * the quadrature counts per output-shaft revolution to also obtain RPM.
 */
#define TB6612_CONFIG_DEFAULT_WITH_ENCODER(                         \
    in1, in2, pwm, encoder_a, encoder_b, counts_per_revolution) {  \
    .in1_gpio = (in1),                                             \
    .in2_gpio = (in2),                                             \
    .pwm_gpio = (pwm),                                             \
    .pwm_frequency_hz = 20000,                                     \
    .pwm_speed_mode = LEDC_LOW_SPEED_MODE,                         \
    .pwm_timer = LEDC_TIMER_0,                                     \
    .pwm_channel = LEDC_CHANNEL_0,                                 \
    .pwm_resolution = LEDC_TIMER_10_BIT,                           \
    .encoder_a_gpio = (encoder_a),                                 \
    .encoder_b_gpio = (encoder_b),                                 \
    .encoder_glitch_filter_ns = 1000,                              \
    .encoder_counts_per_revolution = (counts_per_revolution),      \
    .encoder_invert_direction = false,                             \
    ._initialized = false,                                         \
    ._direction = 0,                                               \
    ._encoder_context = 0,                                         \
}

/**
 * @brief Initialize a configured TB6612 motor instance.
 *
 * The TB6612 STBY pin must be tied high externally.
 */
esp_err_t tb6612_init(tb6612_t *motor);

/**
 * @brief Set signed motor output in permille.
 *
 * Positive values run forward, negative values run in reverse, and zero
 * coasts. The accepted range is -1000 to 1000.
 */
esp_err_t tb6612_set_speed(tb6612_t *motor, int16_t speed_permille);

/**
 * @brief Stop driving the motor outputs and allow the motor to coast.
 */
esp_err_t tb6612_coast(tb6612_t *motor);

/**
 * @brief Actively brake the motor through the TB6612.
 */
esp_err_t tb6612_brake(tb6612_t *motor);

/**
 * @brief Read encoder speed accumulated since the previous call.
 *
 * The encoder counter is cleared after each read. counts_per_second is always
 * available. rpm_valid is true only when encoder_counts_per_revolution is
 * greater than zero.
 */
esp_err_t tb6612_read_speed(tb6612_t *motor, tb6612_speed_t *speed);

/**
 * @brief Stop one motor instance and release its GPIO/LEDC channel.
 */
esp_err_t tb6612_deinit(tb6612_t *motor);

#ifdef __cplusplus
}
#endif
