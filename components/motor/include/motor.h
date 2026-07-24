#include "pid.h"
#include "tb6612.h"

typedef struct
{
    float REAL_RPM;
    float TARGET_RPM;      
} MOTOR_REAL_INFO;

void motor_init(void);
void set_motor_speed(tb6612_t *motor, MOTOR_REAL_INFO *REAL_MOTOR, uint8_t index);

extern tb6612_t s_motor1;
extern tb6612_t s_motor2;
extern tb6612_t s_motor3;

extern MOTOR_REAL_INFO chassisMotorRealInfo[3];
extern PID_T pid_chassis_rpm[3];