#include "tb6612.h"

#include <stdbool.h>
#include <stdlib.h>

#include "driver/pulse_cnt.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define ENCODER_PCNT_HIGH_LIMIT  32767
#define ENCODER_PCNT_LOW_LIMIT  -32767

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel_a;
    pcnt_channel_handle_t channel_b;
    int64_t last_sample_time_us;
} tb6612_encoder_context_t;

static bool encoder_is_configured(const tb6612_t *motor)
{
    return motor->encoder_a_gpio != GPIO_NUM_NC &&
           motor->encoder_b_gpio != GPIO_NUM_NC;
}

static esp_err_t validate_config(const tb6612_t *motor)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(motor->in1_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(motor->in2_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(motor->pwm_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motor->in1_gpio == motor->in2_gpio ||
        motor->in1_gpio == motor->pwm_gpio ||
        motor->in2_gpio == motor->pwm_gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motor->pwm_frequency_hz == 0 ||
        motor->pwm_resolution <= 0 ||
        motor->pwm_resolution >= 31 ||
        motor->pwm_timer >= LEDC_TIMER_MAX ||
        motor->pwm_channel >= LEDC_CHANNEL_MAX ||
        motor->pwm_speed_mode >= LEDC_SPEED_MODE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool encoder_disabled =
        motor->encoder_a_gpio == GPIO_NUM_NC &&
        motor->encoder_b_gpio == GPIO_NUM_NC;
    if (!encoder_disabled) {
        if (!GPIO_IS_VALID_GPIO(motor->encoder_a_gpio) ||
            !GPIO_IS_VALID_GPIO(motor->encoder_b_gpio) ||
            motor->encoder_a_gpio == motor->encoder_b_gpio ||
            motor->encoder_a_gpio == motor->in1_gpio ||
            motor->encoder_a_gpio == motor->in2_gpio ||
            motor->encoder_a_gpio == motor->pwm_gpio ||
            motor->encoder_b_gpio == motor->in1_gpio ||
            motor->encoder_b_gpio == motor->in2_gpio ||
            motor->encoder_b_gpio == motor->pwm_gpio) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

static void cleanup_partial_encoder(tb6612_encoder_context_t *context,
                                    bool enabled, bool started)
{
    if (context == NULL) {
        return;
    }
    if (started) {
        pcnt_unit_stop(context->unit);
    }
    if (enabled) {
        pcnt_unit_disable(context->unit);
    }
    if (context->channel_b != NULL) {
        pcnt_del_channel(context->channel_b);
    }
    if (context->channel_a != NULL) {
        pcnt_del_channel(context->channel_a);
    }
    if (context->unit != NULL) {
        pcnt_del_unit(context->unit);
    }
    free(context);
}

static esp_err_t init_encoder(tb6612_t *motor)
{
    if (!encoder_is_configured(motor)) {
        return ESP_OK;
    }

    tb6612_encoder_context_t *context =
        calloc(1, sizeof(tb6612_encoder_context_t));
    if (context == NULL) {
        return ESP_ERR_NO_MEM;
    }

    bool enabled = false;
    bool started = false;
    const pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &context->unit);
    if (err != ESP_OK) {
        goto fail;
    }

    if (motor->encoder_glitch_filter_ns > 0) {
        const pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = motor->encoder_glitch_filter_ns,
        };
        err = pcnt_unit_set_glitch_filter(context->unit, &filter_config);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = motor->encoder_a_gpio,
        .level_gpio_num = motor->encoder_b_gpio,
    };
    err = pcnt_new_channel(
        context->unit, &channel_a_config, &context->channel_a);
    if (err != ESP_OK) {
        goto fail;
    }

    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = motor->encoder_b_gpio,
        .level_gpio_num = motor->encoder_a_gpio,
    };
    err = pcnt_new_channel(
        context->unit, &channel_b_config, &context->channel_b);
    if (err != ESP_OK) {
        goto fail;
    }

    gpio_pullup_en(motor->encoder_a_gpio);
    gpio_pulldown_dis(motor->encoder_a_gpio);
    gpio_pullup_en(motor->encoder_b_gpio);
    gpio_pulldown_dis(motor->encoder_b_gpio);

    err = pcnt_channel_set_edge_action(
        context->channel_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_channel_set_level_action(
        context->channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_channel_set_edge_action(
        context->channel_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_channel_set_level_action(
        context->channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) {
        goto fail;
    }

    err = pcnt_unit_enable(context->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    enabled = true;
    err = pcnt_unit_clear_count(context->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    err = pcnt_unit_start(context->unit);
    if (err != ESP_OK) {
        goto fail;
    }
    started = true;

    context->last_sample_time_us = esp_timer_get_time();
    motor->_encoder_context = context;
    return ESP_OK;

fail:
    cleanup_partial_encoder(context, enabled, started);
    return err;
}

static esp_err_t deinit_encoder(tb6612_t *motor)
{
    tb6612_encoder_context_t *context = motor->_encoder_context;
    if (context == NULL) {
        return ESP_OK;
    }

    esp_err_t err = pcnt_unit_stop(context->unit);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_disable(context->unit);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_del_channel(context->channel_b);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_del_channel(context->channel_a);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_del_unit(context->unit);
    if (err != ESP_OK) {
        return err;
    }

    free(context);
    motor->_encoder_context = NULL;
    return ESP_OK;
}

static uint32_t max_duty(const tb6612_t *motor)
{
    return (1UL << motor->pwm_resolution) - 1UL;
}

static esp_err_t set_pwm_duty(tb6612_t *motor, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(
        motor->pwm_speed_mode, motor->pwm_channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(motor->pwm_speed_mode, motor->pwm_channel);
}

static esp_err_t set_direction_levels(
    const tb6612_t *motor, int in1_level, int in2_level)
{
    esp_err_t err = gpio_set_level(motor->in1_gpio, in1_level);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_set_level(motor->in2_gpio, in2_level);
}

esp_err_t tb6612_init(tb6612_t *motor)
{
    esp_err_t err = validate_config(motor);
    if (err != ESP_OK) {
        return err;
    }
    if (motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const gpio_config_t direction_gpio_config = {
        .pin_bit_mask = BIT64(motor->in1_gpio) | BIT64(motor->in2_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&direction_gpio_config);
    if (err != ESP_OK) {
        return err;
    }
    err = set_direction_levels(motor, 0, 0);
    if (err != ESP_OK) {
        goto fail_gpio;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = motor->pwm_speed_mode,
        .duty_resolution = motor->pwm_resolution,
        .timer_num = motor->pwm_timer,
        .freq_hz = motor->pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        goto fail_gpio;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = motor->pwm_gpio,
        .speed_mode = motor->pwm_speed_mode,
        .channel = motor->pwm_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = motor->pwm_timer,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) {
        goto fail_gpio;
    }

    err = init_encoder(motor);
    if (err != ESP_OK) {
        ledc_stop(motor->pwm_speed_mode, motor->pwm_channel, 0);
        gpio_reset_pin(motor->pwm_gpio);
        goto fail_gpio;
    }

    motor->_direction = 0;
    motor->_initialized = true;
    return ESP_OK;

fail_gpio:
    gpio_reset_pin(motor->in1_gpio);
    gpio_reset_pin(motor->in2_gpio);
    return err;
}

esp_err_t tb6612_set_speed(tb6612_t *motor, int16_t speed_permille)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_permille < -1000 || speed_permille > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (speed_permille == 0) {
        return tb6612_coast(motor);
    }

    const int8_t new_direction = speed_permille > 0 ? 1 : -1;
    if (motor->_direction != 0 && motor->_direction != new_direction) {
        esp_err_t err = set_pwm_duty(motor, 0);
        if (err != ESP_OK) {
            return err;
        }
        err = set_direction_levels(motor, 0, 0);
        if (err != ESP_OK) {
            return err;
        }
        esp_rom_delay_us(1000);
    }

    esp_err_t err = new_direction > 0
                        ? set_direction_levels(motor, 1, 0)
                        : set_direction_levels(motor, 0, 1);
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t magnitude = (uint32_t)abs((int)speed_permille);
    const uint32_t duty = (magnitude * max_duty(motor)) / 1000U;
    err = set_pwm_duty(motor, duty);
    if (err == ESP_OK) {
        motor->_direction = new_direction;
    }
    return err;
}

esp_err_t tb6612_coast(tb6612_t *motor)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = set_pwm_duty(motor, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = set_direction_levels(motor, 0, 0);
    if (err == ESP_OK) {
        motor->_direction = 0;
    }
    return err;
}

esp_err_t tb6612_brake(tb6612_t *motor)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = set_pwm_duty(motor, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = set_direction_levels(motor, 1, 1);
    if (err == ESP_OK) {
        motor->_direction = 0;
    }
    return err;
}

esp_err_t tb6612_read_speed(tb6612_t *motor, tb6612_speed_t *speed)
{
    if (motor == NULL || speed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    tb6612_encoder_context_t *context = motor->_encoder_context;
    if (context == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int count = 0;
    esp_err_t err = pcnt_unit_get_count(context->unit, &count);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_clear_count(context->unit);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - context->last_sample_time_us;
    if (elapsed_us <= 0) {
        return ESP_ERR_INVALID_STATE;
    }
    context->last_sample_time_us = now_us;

    if (motor->encoder_invert_direction) {
        count = -count;
    }

    speed->delta_counts = count;
    speed->sample_time_us = elapsed_us;
    speed->counts_per_second =
        ((float)count * 1000000.0f) / (float)elapsed_us;
    speed->rpm_valid = motor->encoder_counts_per_revolution > 0;
    speed->rpm = speed->rpm_valid
                     ? speed->counts_per_second * 60.0f /
                           (float)motor->encoder_counts_per_revolution
                     : 0.0f;
    return ESP_OK;
}

esp_err_t tb6612_deinit(tb6612_t *motor)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_OK;
    }

    esp_err_t err = tb6612_coast(motor);
    if (err != ESP_OK) {
        return err;
    }
    err = deinit_encoder(motor);
    if (err != ESP_OK) {
        return err;
    }
    err = ledc_stop(motor->pwm_speed_mode, motor->pwm_channel, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_reset_pin(motor->pwm_gpio);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_reset_pin(motor->in1_gpio);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_reset_pin(motor->in2_gpio);
    if (err == ESP_OK) {
        motor->_initialized = false;
        motor->_direction = 0;
    }
    return err;
}