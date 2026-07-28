#include "motor.h"
#include "tb6612.h"
#include "pid.h"

#define SPEED_SAMPLE_PERIOD_MS  100//锟劫度诧拷锟斤拷锟斤拷锟斤拷
#define SPEED_LOG_PERIOD_MS     500//锟劫讹拷锟斤拷志锟斤拷锟斤拷
#define SPEED_STOP_THRESHOLD    5//锟劫讹拷止停锟斤拷值

tb6612_t s_motor1 = {
    .in1_gpio = 16,
    .in2_gpio = 17,
    .pwm_gpio = 18,
    .pwm_frequency_hz = 20000,
    .pwm_speed_mode = LEDC_LOW_SPEED_MODE,
    .pwm_timer = LEDC_TIMER_0,
    .pwm_channel = LEDC_CHANNEL_0,
    .pwm_resolution = LEDC_TIMER_10_BIT,
    .encoder_a_gpio = 41,
    .encoder_b_gpio = 42,
    .encoder_glitch_filter_ns = 1000,
    .encoder_counts_per_revolution = 1320,
    .encoder_invert_direction = true,
};
tb6612_t s_motor2 = {
    .in1_gpio = 4,
    .in2_gpio = 5,
    .pwm_gpio = 6,
    .pwm_frequency_hz = 20000,
    .pwm_speed_mode = LEDC_LOW_SPEED_MODE,
    .pwm_timer = LEDC_TIMER_0,
    .pwm_channel = LEDC_CHANNEL_1,
    .pwm_resolution = LEDC_TIMER_10_BIT,
    .encoder_a_gpio = 7,
    .encoder_b_gpio = 15,
    .encoder_glitch_filter_ns = 1000,
    .encoder_counts_per_revolution = 1320,
    .encoder_invert_direction = true,
};
tb6612_t s_motor3 = {
    .in1_gpio = 8,
    .in2_gpio = 9,
    .pwm_gpio = 10,
    .pwm_frequency_hz = 20000,
    .pwm_speed_mode = LEDC_LOW_SPEED_MODE,
    .pwm_timer = LEDC_TIMER_0,
    .pwm_channel = LEDC_CHANNEL_2,
    .pwm_resolution = LEDC_TIMER_10_BIT,
    .encoder_a_gpio = 11,
    .encoder_b_gpio = 12,
    .encoder_glitch_filter_ns = 1000,
    .encoder_counts_per_revolution = 1320,
    .encoder_invert_direction = true,
};

MOTOR_REAL_INFO chassisMotorRealInfo[3] = {0};

//pid锟结构锟斤拷锟绞硷拷锟�
PID_T pid_chassis_rpm[3] = {0};

//锟斤拷锟斤拷栈锟斤拷锟斤拷锟�
// index: 0/1/2 指锟斤拷 pid_chassis_rpm[index]
void set_motor_speed(tb6612_t *motor, MOTOR_REAL_INFO *REAL_MOTOR, uint8_t index)
{
    int16_t pwm = 0;//pid锟斤拷锟�
    tb6612_speed_t speed = {0};
    //锟斤拷取锟斤拷锟斤拷锟斤拷俣锟�
    ESP_ERROR_CHECK(tb6612_read_speed(motor, &speed));
    REAL_MOTOR->REAL_RPM =  speed.rpm;
    if (index >= 3) index = 0;
    //pid锟斤拷锟斤拷
    pwm = (int16_t)pid_calc(&pid_chassis_rpm[index], REAL_MOTOR->TARGET_RPM, REAL_MOTOR->REAL_RPM);
    //锟斤拷锟矫碉拷锟斤拷俣锟�
    ESP_ERROR_CHECK(tb6612_set_speed(motor, pwm));
}

//tb6612锟斤拷始锟斤拷
void motor_init(void)
{
    //锟斤拷锟斤拷锟绞硷拷锟�
    ESP_ERROR_CHECK(tb6612_init(&s_motor1));
    ESP_ERROR_CHECK(tb6612_init(&s_motor2));
    ESP_ERROR_CHECK(tb6612_init(&s_motor3));
    //锟斤拷锟斤拷锟斤拷锟�
    ESP_ERROR_CHECK(tb6612_coast(&s_motor1));
    ESP_ERROR_CHECK(tb6612_coast(&s_motor2));
    ESP_ERROR_CHECK(tb6612_coast(&s_motor3));
}
