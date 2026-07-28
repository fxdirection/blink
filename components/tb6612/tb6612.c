/**
 * @file tb6612.c
 * @brief TB6612 电机驱动的内部实现。
 *
 * 初学者可以先只看本文件中的 6 个公开函数：
 *   tb6612_init()
 *   tb6612_set_speed()
 *   tb6612_coast()
 *   tb6612_brake()
 *   tb6612_read_speed()
 *   tb6612_deinit()
 *
 * 前面的 static 函数都是模块内部使用的“小助手”，类似 Python 类中
 * 以下划线开头的方法。static 表示它们不会暴露给其他 .c 文件。
 */
#include "tb6612.h"

#include <stdbool.h>
#include <stdlib.h>

/*
 * PCNT（Pulse Counter）是 ESP32 的硬件脉冲计数器，用来数编码器脉冲；
 * esp_timer 提供微秒时间，用来把“脉冲数”换算成“每秒脉冲数/RPM”；
 * esp_rom_delay_us 用于电机换向前的短暂停顿。
 */
#include "driver/pulse_cnt.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

/*
 * PCNT 的硬件计数上下限。这里每次测速都会读取并清零，因此正常情况下
 * 不会接近上下限；上下限主要防止很久不读取时无限累计。
 */
#define ENCODER_PCNT_HIGH_LIMIT  32767
#define ENCODER_PCNT_LOW_LIMIT  -32767

/*
 * 编码器运行时上下文，只在本文件内部使用。
 *
 * unit      ：一个 PCNT 计数单元；
 * channel_a ：负责观察 A 相边沿，并用 B 相电平判断方向；
 * channel_b ：负责观察 B 相边沿，并用 A 相电平判断方向；
 * last_sample_time_us：上次测速时间。
 *
 * 两个通道配合后，A/B 的上升沿和下降沿都能计数，也就是常说的
 * “四倍频正交解码”。
 */
typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel_a;
    pcnt_channel_handle_t channel_b;
    int64_t last_sample_time_us;
} tb6612_encoder_context_t;

/* 只有 A、B 两个引脚都有效时，才认为该电机启用了编码器。 */
static bool encoder_is_configured(const tb6612_t *motor)
{
    return motor->encoder_a_gpio != GPIO_NUM_NC &&
           motor->encoder_b_gpio != GPIO_NUM_NC;
}

/**
 * @brief 在碰硬件之前检查用户填写的配置。
 *
 * 为什么先检查：GPIO 或 LEDC 配置错误时，尽早返回比初始化到一半才失败
 * 更容易排查，也避免占用一部分硬件资源后忘记释放。
 */
static esp_err_t validate_config(const tb6612_t *motor)
{
    /* C 中传入 NULL 类似 Python 中传入 None，这里不能继续解引用。 */
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* IN1、IN2 和 PWM 都必须是支持输出的 GPIO。 */
    if (!GPIO_IS_VALID_OUTPUT_GPIO(motor->in1_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(motor->in2_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(motor->pwm_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 同一路电机的三个输出功能不能复用同一个引脚。 */
    if (motor->in1_gpio == motor->in2_gpio ||
        motor->in1_gpio == motor->pwm_gpio ||
        motor->in2_gpio == motor->pwm_gpio) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 检查 PWM 频率、分辨率以及 LEDC 枚举值是否在合法范围内。 */
    if (motor->pwm_frequency_hz == 0 ||
        motor->pwm_resolution <= 0 ||
        motor->pwm_resolution >= 31 ||
        motor->pwm_timer >= LEDC_TIMER_MAX ||
        motor->pwm_channel >= LEDC_CHANNEL_MAX ||
        motor->pwm_speed_mode >= LEDC_SPEED_MODE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 编码器只能有两种完整状态：
     *   1. A/B 都是 GPIO_NUM_NC：不使用编码器；
     *   2. A/B 都是有效且互不冲突的 GPIO：使用编码器。
     * 只配置一根线会在下面被判为无效。
     */
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

/**
 * @brief 编码器初始化中途失败时，释放已经申请成功的资源。
 *
 * 硬件资源按“申请的相反顺序”释放。enabled/started 用来记录初始化走到了
 * 哪一步，避免对尚未启动的资源执行停止操作。
 */
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

/**
 * @brief 根据 motor 中的 A/B 引脚配置创建 PCNT 正交编码器。
 *
 * 如果调用者把 A/B 都设成 GPIO_NUM_NC，本函数什么也不做并返回成功，
 * 这样同一套电机驱动既能支持带编码器电机，也能支持普通直流电机。
 */
static esp_err_t init_encoder(tb6612_t *motor)
{
    if (!encoder_is_configured(motor)) {
        return ESP_OK;
    }

    /*
     * calloc 类似“创建一个对象并把所有字段初始化为 0”。
     * 上下文存在堆内存里，地址最后保存到 motor->_encoder_context。
     */
    tb6612_encoder_context_t *context =
        calloc(1, sizeof(tb6612_encoder_context_t));
    if (context == NULL) {
        return ESP_ERR_NO_MEM;
    }

    bool enabled = false;
    bool started = false;

    /* 创建一个硬件计数单元，并规定允许的计数范围。 */
    const pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &context->unit);
    if (err != ESP_OK) {
        goto fail;
    }

    /* 可选毛刺滤波：过短的电平跳变通常是电气噪声，不计入脉冲。 */
    if (motor->encoder_glitch_filter_ns > 0) {
        const pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = motor->encoder_glitch_filter_ns,
        };
        err = pcnt_unit_set_glitch_filter(context->unit, &filter_config);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    /*
     * A 通道：A 是“边沿信号”，B 是“方向判断信号”。
     * 当 A 发生变化时，PCNT 会结合 B 当前的高低电平决定加一还是减一。
     */
    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = motor->encoder_a_gpio,
        .level_gpio_num = motor->encoder_b_gpio,
    };
    err = pcnt_new_channel(
        context->unit, &channel_a_config, &context->channel_a);
    if (err != ESP_OK) {
        goto fail;
    }

    /*
     * B 通道反过来：B 是边沿信号，A 是方向判断信号。
     * 两个通道都建立后，上升沿和下降沿都不会浪费。
     */
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = motor->encoder_b_gpio,
        .level_gpio_num = motor->encoder_a_gpio,
    };
    err = pcnt_new_channel(
        context->unit, &channel_b_config, &context->channel_b);
    if (err != ESP_OK) {
        goto fail;
    }

    /*
     * 打开内部上拉、关闭下拉，避免编码器输出悬空时随机跳变。
     * 如果你的编码器模块已经有合适的外部上拉，这些内部上拉通常也可共存。
     */
    gpio_pullup_en(motor->encoder_a_gpio);
    gpio_pulldown_dis(motor->encoder_a_gpio);
    gpio_pullup_en(motor->encoder_b_gpio);
    gpio_pulldown_dis(motor->encoder_b_gpio);

    /*
     * 下面四个调用定义“看到哪个边沿时加一/减一，以及另一相为高时是否反转”。
     * 这是正交编码器能够同时测速度和方向的关键：
     *   - 正转时得到正计数；
     *   - 反转时得到负计数。
     */
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

    /* PCNT 的标准启动顺序：enable → 清零 → start。 */
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

    /*
     * 记录第一次测速的时间起点，并把内部上下文挂到公开电机对象上。
     * 后续 tb6612_read_speed() 会从这里取回它。
     */
    context->last_sample_time_us = esp_timer_get_time();
    motor->_encoder_context = context;
    return ESP_OK;

fail:
    /* goto fail 是 C 中常见的统一错误清理写法，类似 Python 的 finally。 */
    cleanup_partial_encoder(context, enabled, started);
    return err;
}

/**
 * @brief 按安全顺序停止并删除编码器资源。
 */
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

/*
 * 把 PWM 分辨率换算成最大占空比。
 * 例如 10 bit 时：(1 << 10) - 1 = 1023。
 */
static uint32_t max_duty(const tb6612_t *motor)
{
    return (1UL << motor->pwm_resolution) - 1UL;
}

/* LEDC 修改占空比需要“先设置，再更新”两个步骤，这里统一封装。 */
static esp_err_t set_pwm_duty(tb6612_t *motor, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(
        motor->pwm_speed_mode, motor->pwm_channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(motor->pwm_speed_mode, motor->pwm_channel);
}

/* 同时设置 TB6612 的两个方向引脚，避免公开函数重复相同代码。 */
static esp_err_t set_direction_levels(
    const tb6612_t *motor, int in1_level, int in2_level)
{
    esp_err_t err = gpio_set_level(motor->in1_gpio, in1_level);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_set_level(motor->in2_gpio, in2_level);
}

/**
 * @brief 初始化电机方向 GPIO、LEDC PWM 和可选编码器。
 *
 * 可以把它理解成 Python 类的 __init__ 之后再打开硬件资源：
 * 只有本函数返回 ESP_OK，后续设速和测速函数才可以使用。
 */
esp_err_t tb6612_init(tb6612_t *motor)
{
    /* 第一步只检查参数，不触碰硬件。 */
    esp_err_t err = validate_config(motor);
    if (err != ESP_OK) {
        return err;
    }
    if (motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 把 IN1/IN2 配成输出，默认带下拉。
     * 紧接着写 0/0，确保初始化期间电机处于滑行停止状态。
     */
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

    /*
     * LEDC timer 决定一组 PWM 的频率和分辨率。
     * 多路电机可以共用 timer，但共用时必须使用相同参数。
     */
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

    /*
     * LEDC channel 把某个 PWM 通道连接到实际 pwm_gpio。
     * 初始 duty=0，防止初始化完成前电机突然转动。
     */
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

    /* 没配置编码器时 init_encoder() 也会返回 ESP_OK。 */
    err = init_encoder(motor);
    if (err != ESP_OK) {
        /* 编码器失败时，撤销前面已经完成的 PWM/GPIO 配置。 */
        ledc_stop(motor->pwm_speed_mode, motor->pwm_channel, 0);
        gpio_reset_pin(motor->pwm_gpio);
        goto fail_gpio;
    }

    /* 最后才标记成功，避免半初始化对象被误用。 */
    motor->_direction = 0;
    motor->_initialized = true;
    return ESP_OK;

fail_gpio:
    /* 初始化失败的统一出口：恢复方向引脚的默认状态。 */
    gpio_reset_pin(motor->in1_gpio);
    gpio_reset_pin(motor->in2_gpio);
    return err;
}

/**
 * @brief 把带符号的千分比转换成方向引脚和 PWM 占空比。
 */
esp_err_t tb6612_set_speed(tb6612_t *motor, int16_t speed_permille)
{
    /* 公开函数都先检查对象和生命周期，避免访问无效硬件句柄。 */
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_permille < -1000 || speed_permille > 1000) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 约定速度 0 使用滑行停止，而不是主动刹车。 */
    if (speed_permille == 0) {
        return tb6612_coast(motor);
    }

    const int8_t new_direction = speed_permille > 0 ? 1 : -1;

    /*
     * 正转和反转之间不能直接切换：
     * 先把 PWM 降为 0，再让 IN1/IN2 回到 0/0，等待 1 ms 后再反向。
     * 这样可以减小瞬时大电流和机械冲击。
     */
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

    /* 正数对应 IN1/IN2=1/0，负数对应 0/1。 */
    esp_err_t err = new_direction > 0
                        ? set_direction_levels(motor, 1, 0)
                        : set_direction_levels(motor, 0, 1);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * 把 0~1000 千分比线性映射到 LEDC 的 0~max_duty。
     * 10 bit 时，例如 500 会得到约 511，也就是约 50% 占空比。
     */
    const uint32_t magnitude = (uint32_t)abs((int)speed_permille);
    const uint32_t duty = (magnitude * max_duty(motor)) / 1000U;
    err = set_pwm_duty(motor, duty);
    if (err == ESP_OK) {
        /* 只有硬件设置成功，才更新软件记录的方向。 */
        motor->_direction = new_direction;
    }
    return err;
}

/**
 * @brief 滑行停止：PWM=0，IN1/IN2=0/0。
 */
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

/**
 * @brief 主动刹车：PWM=0，IN1/IN2=1/1。
 *
 * brake 和 coast 都会让软件记录的方向归零，但电机停下的物理方式不同。
 */
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

/**
 * @brief 读取本采样窗口内的编码器计数，并换算成速度。
 *
 * 速度公式：
 *   counts_per_second = 本次计数 × 1,000,000 / 经过微秒数
 *   rpm = counts_per_second × 60 / 每圈计数
 */
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
        /* 电机可不带编码器；此时驱动能设速，但不支持测速。 */
        return ESP_ERR_NOT_SUPPORTED;
    }

    /*
     * 先读取当前累计值，再清零。
     * 因此 delta_counts 只覆盖“上次调用到本次调用”这一段时间。
     */
    int count = 0;
    esp_err_t err = pcnt_unit_get_count(context->unit, &count);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_clear_count(context->unit);
    if (err != ESP_OK) {
        return err;
    }

    /* 用真实经过时间计算速度，而不是假设任务周期永远精确。 */
    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - context->last_sample_time_us;
    if (elapsed_us <= 0) {
        return ESP_ERR_INVALID_STATE;
    }
    context->last_sample_time_us = now_us;

    /* 用配置开关修正接线或安装方向导致的正负号相反。 */
    if (motor->encoder_invert_direction) {
        count = -count;
    }

    /* 保存原始计数、采样时间和换算后的 counts/s。 */
    speed->delta_counts = count;
    speed->sample_time_us = elapsed_us;
    speed->counts_per_second =
        ((float)count * 1000000.0f) / (float)elapsed_us;

    /*
     * 只有知道每圈计数才能计算 RPM。
     * rpm_valid 单独存在，是为了区分“真实 0 RPM”和“根本无法计算 RPM”。
     */
    speed->rpm_valid = motor->encoder_counts_per_revolution > 0;
    speed->rpm = speed->rpm_valid
                     ? speed->counts_per_second * 60.0f /
                           (float)motor->encoder_counts_per_revolution
                     : 0.0f;
    return ESP_OK;
}

/**
 * @brief 停止电机，并释放 init 时申请的全部硬件资源。
 *
 * 释放顺序大致与初始化相反：
 *   滑行停止 → 编码器 → LEDC → PWM GPIO → 方向 GPIO。
 */
esp_err_t tb6612_deinit(tb6612_t *motor)
{
    if (motor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!motor->_initialized) {
        /* 让重复清理变得安全，类似 Python close() 的幂等用法。 */
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
        /* 全部释放成功后，才把对象恢复成“未初始化”状态。 */
        motor->_initialized = false;
        motor->_direction = 0;
    }
    return err;
}
