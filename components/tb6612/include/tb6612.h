#pragma once

/*
 * 这个头文件是 tb6612 模块的“公开说明书”。
 *
 * 如果用 Python 类比：
 *   - #include "tb6612.h" 类似 import tb6612
 *   - tb6612_t 类似一个保存配置和状态的对象
 *   - tb6612_init()/tb6612_set_speed() 类似这个对象对外提供的方法
 *
 * 使用者通常只需要阅读本文件；tb6612.c 中的细节由模块自己处理。
 */

#include <stdbool.h>
#include <stdint.h>

/* ESP-IDF 提供的 GPIO、PWM(LEDC) 类型和统一错误码。 */
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

/*
 * 让这个 C 头文件也能被 C++ 调用。
 * 当前项目使用 C，可以先把这几行理解成兼容性模板。
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一路 TB6612 电机的配置和运行状态。
 *
 * 一个 tb6612_t 只代表“一路电机”，不是整块双路 TB6612 芯片。
 * 如果有 3 个电机，就创建 3 个 tb6612_t 变量。
 *
 * 调用顺序：
 *   1. 先填写结构体配置；
 *   2. 调用 tb6612_init()；
 *   3. 再调用设速、测速、刹车等函数；
 *   4. 不再使用时调用 tb6612_deinit()。
 *
 * 初始化成功后不要再修改配置字段，否则软件保存的状态可能与硬件不一致。
 *
 * 多电机注意：
 *   - 每路电机必须使用不同的 pwm_channel；
 *   - 只有 PWM 频率和分辨率相同时，才可以共用 pwm_timer。
 */
typedef struct {
    /* ---------- TB6612 方向与 PWM 输出配置 ---------- */

    /*
     * IN1/IN2 决定转动方向：
     *   IN1=1, IN2=0：正转
     *   IN1=0, IN2=1：反转
     *   IN1=0, IN2=0：滑行停止
     *   IN1=1, IN2=1：主动刹车
     */
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;

    /* 接 TB6612 PWMA/PWMB 的 GPIO，由 PWM 占空比控制输出强弱。 */
    gpio_num_t pwm_gpio;

    /* PWM 频率，当前项目通常使用 20000 Hz（20 kHz）。 */
    uint32_t pwm_frequency_hz;

    /*
     * ESP32 的 LEDC 是 PWM 外设。下面三个字段选择 LEDC 的：
     *   - 工作模式；
     *   - 定时器；
     *   - 通道；
     *   - 分辨率（例如 10 bit 对应 0~1023）。
     */
    ledc_mode_t pwm_speed_mode;
    ledc_timer_t pwm_timer;
    ledc_channel_t pwm_channel;
    ledc_timer_bit_t pwm_resolution;

    /* ---------- 可选的 AB 相正交编码器配置 ---------- */

    /*
     * 编码器 A/B 相 GPIO。
     * 如果电机没有编码器，把这两个字段都设为 GPIO_NUM_NC。
     */
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;

    /* 忽略短于该时间的毛刺脉冲，单位为纳秒；0 表示不启用滤波。 */
    uint32_t encoder_glitch_filter_ns;

    /*
     * 输出轴转一圈对应多少个“四倍频计数”。
     * 大于 0 时可以计算 RPM；等于 0 时只能得到 counts_per_second。
     */
    uint32_t encoder_counts_per_revolution;

    /* 如果实测 RPM 正负方向与期望相反，设为 true 即可软件翻转。 */
    bool encoder_invert_direction;

    /* ---------- 模块内部运行状态 ---------- */

    /*
     * 以下字段相当于 Python 对象中以下划线开头的“内部属性”。
     * 调用者不要直接读写；创建结构体时必须初始化为 0/false。
     */
    bool _initialized;
    int8_t _direction;
    void *_encoder_context;
} tb6612_t;

/**
 * @brief tb6612_read_speed() 返回的一次测速结果。
 */
typedef struct {
    /* 从上一次测速到这一次测速之间，编码器累计的有符号计数。 */
    int32_t delta_counts;

    /* 两次测速之间真实经过的时间，单位为微秒。 */
    int64_t sample_time_us;

    /* 每秒编码器计数，可在不知道每圈计数时使用。 */
    float counts_per_second;

    /* 每分钟转数；只有 rpm_valid=true 时才有效。 */
    float rpm;
    bool rpm_valid;
} tb6612_speed_t;

/**
 * @brief 创建“不带编码器”的默认配置。
 *
 * 默认值：20 kHz、10 bit PWM、LEDC 低速模式、timer 0、channel 0。
 *
 * 这是 C 的宏，可以把它理解成 Python 中返回默认配置对象的函数：
 *
 * @code{.c}
 * static tb6612_t motor = TB6612_CONFIG_DEFAULT(
 *     GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18);
 * @endcode
 *
 * 如果同时创建多路电机，必须为后续电机修改 pwm_channel，不能都用 channel 0。
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
 * @brief 创建“带 AB 相编码器”的默认配置。
 *
 * counts_per_revolution 可以设成 0，此时仍能读取 counts/s，但不能计算 RPM。
 * 若要得到 RPM，应填写输出轴转一圈的实际四倍频计数。
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
 * @brief 初始化一路电机。
 *
 * 该函数会检查配置、初始化方向 GPIO、LEDC PWM，并在配置了编码器时
 * 初始化 PCNT 计数器。
 *
 * 本模块没有控制 STBY 引脚，因此 TB6612 STBY 必须在硬件上拉高。
 *
 * @param motor 已填写配置的电机对象，不能为 NULL。
 * @return ESP_OK 表示成功；其他值是 ESP-IDF 错误码。
 */
esp_err_t tb6612_init(tb6612_t *motor);

/**
 * @brief 设置带方向的电机输出，单位为千分比。
 *
 * - 正数：正转；
 * - 负数：反转；
 * - 0：滑行停止；
 * - 有效范围：-1000 到 1000。
 *
 * 例如 500 表示正向 50% PWM，-300 表示反向 30% PWM。
 * 这里设置的是 PWM 输出，不是 RPM；闭环转速由上层 motor/PID 模块负责。
 *
 * @param motor 已成功初始化的电机对象。
 * @param speed_permille 带符号的 PWM 千分比。
 */
esp_err_t tb6612_set_speed(tb6612_t *motor, int16_t speed_permille);

/**
 * @brief 滑行停止：撤掉驱动力，让电机依靠惯性慢慢停下。
 *
 * 对应 PWM=0、IN1=0、IN2=0。
 */
esp_err_t tb6612_coast(tb6612_t *motor);

/**
 * @brief 主动刹车：让 TB6612 短接电机两端，使电机更快停下。
 *
 * 对应 PWM=0、IN1=1、IN2=1。它与 coast 的电气行为不同。
 */
esp_err_t tb6612_brake(tb6612_t *motor);

/**
 * @brief 读取“从上一次调用到现在”的编码器速度。
 *
 * 读取后硬件计数器会被清零，因此应由一个固定任务周期性调用本函数，
 * 不要让多个任务同时读取同一电机。
 *
 * counts_per_second 始终可用；只有 encoder_counts_per_revolution > 0 时，
 * rpm_valid 才为 true，rpm 才有意义。
 *
 * @param motor 已成功初始化且配置了编码器的电机对象。
 * @param speed 用来接收结果的结构体。
 */
esp_err_t tb6612_read_speed(tb6612_t *motor, tb6612_speed_t *speed);

/**
 * @brief 停止电机并释放本模块申请的 GPIO、LEDC 和 PCNT 资源。
 *
 * 可重复调用；如果本来就没有初始化，会直接返回 ESP_OK。
 */
esp_err_t tb6612_deinit(tb6612_t *motor);

#ifdef __cplusplus
}
#endif
