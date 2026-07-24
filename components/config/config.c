#include "config.h"
#include "pid.h"
#include "motor.h"

//pid参数配置结构体

//pid初始化
void config(void)
{
    pid_param_init(&pid_chassis_rpm[0], PID_Position, 1000, 100, 0, 2.0, 0, 2.0f, 0.03f, 0.1f);
    pid_param_init(&pid_chassis_rpm[1], PID_Position, 1000, 100, 0, 2.0, 0, 2.0f, 0.03f, 0.1f);
    pid_param_init(&pid_chassis_rpm[2], PID_Position, 1000, 100, 0, 2.0, 0, 2.0f, 0.03f, 0.1f);
}
