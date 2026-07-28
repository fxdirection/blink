/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "chassis.h"
#include "motor.h"
#include "pid.h"
#include "config.h"
#include "debug_comm.h"
#include "tb6612.h"
#include "wifi_debug.h"

static const char *TAG = "app";

/* ============== debug_comm 的强实现（覆盖 weak） ============== */
void debug_comm_apply_velocity(float vx, float vy, float vw)
{
    chassis_set_velocity(vx, vy, vw);
}

void debug_comm_apply_max_velocity(float vx, float vy, float vw)
{
    chassis_set_max_velocity(vx, vy, vw);
}

void debug_comm_apply_pid(uint8_t idx, float kp, float ki, float kd)
{
    if (idx >= 3) return;
    pid_reset(&pid_chassis_rpm[idx], kp, ki, kd);
}

void debug_comm_apply_emergency_stop(void)
{
    chassis_stop();
}

/* ============== 任务 ============== */
static void motor_task(void *param)
{
    (void)param;
    while (1) {
        // ESP_ERROR_CHECK(tb6612_set_speed(&s_motor1, 100));
        // chassisMotorRealInfo[2].TARGET_RPM = 100;
        // printf("REAL_RPM: %.2f\n", (double)chassisMotorRealInfo[2].TARGET_RPM);
        set_motor_speed(&s_motor1, &chassisMotorRealInfo[0], 0);
        set_motor_speed(&s_motor2, &chassisMotorRealInfo[1], 1);
        set_motor_speed(&s_motor3, &chassisMotorRealInfo[2], 2);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void chassis_task(void *param)
{
    (void)param;
    while (1) {
        Robot_Wheels_RPM_calculate();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* 周期性向 PC 推一帧状态 */
static void state_report_task(void *param)
{
    (void)param;
    const TickType_t period = pdMS_TO_TICKS(100);   /* 10 Hz */
    TickType_t last = xTaskGetTickCount();
    while (1) {
        debug_comm_send_state();
        vTaskDelayUntil(&last, period);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot");
    motor_init();          // tb6612 初始化
    config();              // pid 初始化
    chassis_init();        // 底盘结构初始化 + 设置默认最大速度
    // debug_comm_init();     // 启动串口调试通信 (USB-UART)
    wifi_debug_start();    // 启动 SoftAP + TCP server (无线调试)
    xTaskCreate(motor_task,        "motor_task",  4096, NULL, 5, NULL);
    xTaskCreate(chassis_task,      "chassis_task",4096, NULL, 5, NULL);
    // xTaskCreate(state_report_task, "state_rep",   2048, NULL, 3, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
