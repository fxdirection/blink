/**
 * @file debug_comm.h
 * @brief 串口调试通信协议：PC <-> ESP32S3
 *
 *  帧格式 (小端)
 *  +------+-----+-------+---------+-----+
 *  | 0xAA | CMD | LEN   | PAYLOAD | CHK |
 *  +------+-----+-------+---------+-----+
 *  CHK = (0xAA + CMD + LEN + payload[i]...) & 0xFF
 *
 *  命令字
 *  0x01 SET_VELOCITY  PC->ESP  payload: Vx,Vy,Vw (3 float)
 *  0x02 SET_PID       PC->ESP  payload: idx,u8 + kp,ki,kd (3 float)
 *  0x03 SET_MAX_VEL   PC->ESP  payload: Vx_MAX,Vy_MAX,Vw_MAX (3 float)
 *  0x04 EMERGENCY_STOP PC->ESP payload: empty
 *  0x05 PING / REQ_STATE PC->ESP payload: empty
 *  0x10 STATE_REPORT  ESP->PC  payload: 3 target_rpm + 3 real_rpm + 9 pid + 3 vel
 */
#ifndef DEBUG_COMM_H
#define DEBUG_COMM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DC_CMD_SET_VELOCITY     = 0x01,
    DC_CMD_SET_PID          = 0x02,
    DC_CMD_SET_MAX_VEL      = 0x03,
    DC_CMD_EMERGENCY_STOP   = 0x04,
    DC_CMD_PING             = 0x05,
    DC_CMD_STATE_REPORT     = 0x10,
} dc_cmd_t;

/** 初始化调试通信（默认使用 UART0，与日志共用） */
void debug_comm_init(void);

/** 由其他任务周期性调用，把当前状态打包发给 PC */
void debug_comm_send_state(void);

/** 处理在串口 RX 任务中收到的命令（应用层统一入口） */
void debug_comm_apply_velocity(float vx, float vy, float vw);
void debug_comm_apply_max_velocity(float vx, float vy, float vw);
void debug_comm_apply_pid(uint8_t idx, float kp, float ki, float kd);
void debug_comm_apply_emergency_stop(void);

#ifdef __cplusplus
}
#endif
#endif /* DEBUG_COMM_H */