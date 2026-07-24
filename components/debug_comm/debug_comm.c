/**
 * @file debug_comm.c
 * @brief ESP32-S3 串口调试协议实现（UART0，与日志共用）
 */
#include "debug_comm.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "chassis.h"
#include "motor.h"
#include "pid.h"

#define TAG "dbg_comm"

#define UART_NUM            UART_NUM_0
#define RX_BUF_SIZE         256
#define TX_BUF_SIZE         1024
#define READ_TIMEOUT_MS     10
#define MAX_FRAME_PAYLOAD   64

/* 协议常量 */
#define FRAME_HEAD          0xAAu

/* 应用层 hook：指向真正的“执行”函数 */
__attribute__((weak)) void debug_comm_apply_velocity(float vx, float vy, float vw)    { (void)vx;(void)vy;(void)vw; }
__attribute__((weak)) void debug_comm_apply_max_velocity(float vx, float vy, float vw) { (void)vx;(void)vy;(void)vw; }
__attribute__((weak)) void debug_comm_apply_pid(uint8_t idx, float kp, float ki, float kd) { (void)idx;(void)kp;(void)ki;(void)kd; }
__attribute__((weak)) void debug_comm_apply_emergency_stop(void)                       {}

/* ---- 浮点小端读写 ---- */
static inline void put_f32_le(uint8_t *p, float v)
{
    union { float f; uint32_t u; } conv = { .f = v };
    p[0] = (uint8_t)(conv.u      );
    p[1] = (uint8_t)(conv.u >>  8);
    p[2] = (uint8_t)(conv.u >> 16);
    p[3] = (uint8_t)(conv.u >> 24);
}
static inline float get_f32_le(const uint8_t *p)
{
    union { float f; uint32_t u; } conv = {
        .u = ((uint32_t)p[0])
           | ((uint32_t)p[1] << 8)
           | ((uint32_t)p[2] << 16)
           | ((uint32_t)p[3] << 24)
    };
    return conv.f;
}

/* ---- 发送 ---- */
static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[3] = { FRAME_HEAD, cmd, len };
    uint8_t chk = 0;
    for (int i = 0; i < 3; ++i) chk += hdr[i];
    for (int i = 0; i < len; ++i) chk += payload[i];

    /* 直接走 UART0 TX，不与 log 抢资源 */
    uart_write_bytes(UART_NUM, (const char *)hdr, sizeof(hdr));
    if (payload && len) {
        uart_write_bytes(UART_NUM, (const char *)payload, len);
    }
    uart_write_bytes(UART_NUM, (const char *)&chk, 1);
}

/* ---- 应用层：解析一条命令帧 ---- */
static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
    case DC_CMD_SET_VELOCITY: {
        if (len != 12) { ESP_LOGW(TAG, "SET_VELOCITY bad len %u", len); return; }
        float vx = get_f32_le(payload + 0);
        float vy = get_f32_le(payload + 4);
        float vw = get_f32_le(payload + 8);
        debug_comm_apply_velocity(vx, vy, vw);
        ESP_LOGI(TAG, "VEL %.2f %.2f %.2f", vx, vy, vw);
        break;
    }
    case DC_CMD_SET_PID: {
        if (len != 13) { ESP_LOGW(TAG, "SET_PID bad len %u", len); return; }
        uint8_t idx = payload[0];
        if (idx >= 3) { ESP_LOGW(TAG, "SET_PID idx OOB %u", idx); return; }
        float kp = get_f32_le(payload + 1);
        float ki = get_f32_le(payload + 5);
        float kd = get_f32_le(payload + 9);
        debug_comm_apply_pid(idx, kp, ki, kd);
        ESP_LOGI(TAG, "PID[%u] kp=%.3f ki=%.3f kd=%.3f", idx, kp, ki, kd);
        break;
    }
    case DC_CMD_SET_MAX_VEL: {
        if (len != 12) { ESP_LOGW(TAG, "SET_MAX_VEL bad len %u", len); return; }
        float vx = get_f32_le(payload + 0);
        float vy = get_f32_le(payload + 4);
        float vw = get_f32_le(payload + 8);
        debug_comm_apply_max_velocity(vx, vy, vw);
        ESP_LOGI(TAG, "MAX %.2f %.2f %.2f", vx, vy, vw);
        break;
    }
    case DC_CMD_EMERGENCY_STOP: {
        debug_comm_apply_emergency_stop();
        ESP_LOGW(TAG, "EMERGENCY STOP");
        break;
    }
    case DC_CMD_PING: {
        /* 回一帧 PING，长度 0 也可；这里用 STATE_REPORT 主动推一帧 */
        debug_comm_send_state();
        break;
    }
    default:
        ESP_LOGW(TAG, "unknown cmd 0x%02X", cmd);
        break;
    }
}

/* ---- 解析字节流：状态机 ---- */
typedef struct {
    enum { ST_HEAD, ST_CMD, ST_LEN, ST_PAYLOAD, ST_CHK } state;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[MAX_FRAME_PAYLOAD];
    uint8_t idx;
    uint8_t chk;
} parser_t;

static void parser_reset(parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ST_HEAD;
}

static void parser_feed(parser_t *p, uint8_t b)
{
    switch (p->state) {
    case ST_HEAD:
        if (b == FRAME_HEAD) { p->state = ST_CMD; p->chk = FRAME_HEAD; }
        break;
    case ST_CMD:
        p->cmd = b;
        p->chk += b;
        p->state = ST_LEN;
        break;
    case ST_LEN:
        p->len = b;
        p->chk += b;
        if (p->len > MAX_FRAME_PAYLOAD) {
            ESP_LOGW(TAG, "payload too big %u, reset", p->len);
            parser_reset(p);
        } else if (p->len == 0) {
            p->state = ST_CHK;
        } else {
            p->state = ST_PAYLOAD;
            p->idx = 0;
        }
        break;
    case ST_PAYLOAD:
        p->payload[p->idx++] = b;
        p->chk += b;
        if (p->idx >= p->len) p->state = ST_CHK;
        break;
    case ST_CHK:
        if (p->chk == b) {
            handle_frame(p->cmd, p->payload, p->len);
        } else {
            ESP_LOGW(TAG, "chk fail cmd=0x%02X got=0x%02X want=0x%02X",
                     p->cmd, b, p->chk);
        }
        parser_reset(p);
        break;
    }
}

/* ---- 状态上报（主动推） ---- */
void debug_comm_send_state(void)
{
    /* 外部定义：ROBOT_CHASSI, chassisMotorRealInfo, pid_chassis_rpm
       因为我们 include 了 chassis.h / motor.h, 链接时由 main 提供 */
    extern ROBOT_CHASSIS ROBOT_CHASSI;
    extern MOTOR_REAL_INFO chassisMotorRealInfo[3];
    extern PID_T pid_chassis_rpm[3];

    /* payload 长度 = 12(target) + 12(real) + 36(pid*3) + 12(vel) = 72 */
    uint8_t payload[12 + 12 + 36 + 12];
    uint8_t *p = payload;
    for (int i = 0; i < 3; ++i) put_f32_le(p + i * 4, ROBOT_CHASSI.Motor_Target_RPM[i]);
    p += 12;
    for (int i = 0; i < 3; ++i) put_f32_le(p + i * 4, chassisMotorRealInfo[i].REAL_RPM);
    p += 12;
    for (int i = 0; i < 3; ++i) {
        put_f32_le(p + 0, pid_chassis_rpm[i].kp);
        put_f32_le(p + 4, pid_chassis_rpm[i].ki);
        put_f32_le(p + 8, pid_chassis_rpm[i].kd);
        p += 12;
    }
    put_f32_le(p + 0, ROBOT_CHASSI.Vx);
    put_f32_le(p + 4, ROBOT_CHASSI.Vy);
    put_f32_le(p + 8, ROBOT_CHASSI.Vw);
    p += 12;

    send_frame(DC_CMD_STATE_REPORT, payload, (uint8_t)(p - payload));
}

/* ---- RX 任务 ---- */
static void rx_task(void *arg)
{
    (void)arg;
    parser_t parser;
    parser_reset(&parser);

    uint8_t buf[64];
    while (1) {
        int n = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(READ_TIMEOUT_MS));
        for (int i = 0; i < n; ++i) {
            parser_feed(&parser, buf[i]);
        }
    }
}

void debug_comm_init(void)
{
    /* UART0 已被 console 占用，不要重复 install；
       这里只确认 RX FIFO 阈值 & 必要时增加 RX buffer */
    ESP_LOGI(TAG, "debug_comm ready on UART0 (shared with console)");
    xTaskCreate(rx_task, "dbg_rx", 2048, NULL, 4, NULL);
}