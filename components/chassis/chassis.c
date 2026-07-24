#include "chassis.h"
#include "motor.h"

ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void)
{
	/* 默认最大速度 (m/s, rad/s)，可被 debug_comm 运行时覆盖 */
	ROBOT_CHASSI.Vx_MAX = 1.0f;
	ROBOT_CHASSI.Vy_MAX = 1.0f;
	ROBOT_CHASSI.Vw_MAX = 3.0f;

	ROBOT_CHASSI.Vx = 0;
	ROBOT_CHASSI.Vy = 0;
	ROBOT_CHASSI.Vw = 0;

	ROBOT_CHASSI.world_x = 0;
	ROBOT_CHASSI.world_y = 0;
	ROBOT_CHASSI.world_w = 0;

	ROBOT_CHASSI.remote_x = 0;
	ROBOT_CHASSI.remote_y = 0;
	ROBOT_CHASSI.remote_w = 0;

	ROBOT_CHASSI.plan_x = 0;
	ROBOT_CHASSI.plan_y = 0;
	ROBOT_CHASSI.plan_w = 0;

	ROBOT_CHASSI.Motor_Target_RPM[0] = 0;
	ROBOT_CHASSI.Motor_Target_RPM[1] = 0;
	ROBOT_CHASSI.Motor_Target_RPM[2] = 0;
}

void Robot_Wheels_RPM_calculate(void)
{
	ROBOT_CHASSI.Motor_Target_RPM[0] = (ROBOT_CHASSI.Vy * COS30 - ROBOT_CHASSI.Vx * COS60 + ROBOT_CHASSI.Vw * MS_transition_RM) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[1] = (-ROBOT_CHASSI.Vy * COS60 + ROBOT_CHASSI.Vx * COS30 + ROBOT_CHASSI.Vw * MS_transition_RM) * MS_transition_RM;
	ROBOT_CHASSI.Motor_Target_RPM[2] = (ROBOT_CHASSI.Vx + ROBOT_CHASSI.Vw * MS_transition_RM) * MS_transition_RM;
    for(int i = 0;i<=2;i++)
    {
		printf("REAL_RPM: %.2f\n", (double)ROBOT_CHASSI.Motor_Target_RPM[i]);
        chassisMotorRealInfo[i].TARGET_RPM = ROBOT_CHASSI.Motor_Target_RPM[i];
    }
}

void chassis_stop(void)
{
	ROBOT_CHASSI.Vx = 0;
	ROBOT_CHASSI.Vy = 0;
	ROBOT_CHASSI.Vw = 0;
}

/* ============== 远程调试使用的运行时 setter ============== */
void chassis_set_velocity(float vx, float vy, float vw)
{
	/* 简易限幅：如果设了上限则按上限裁剪 */
	if (ROBOT_CHASSI.Vx_MAX > 0.0f) {
		if (vx >  ROBOT_CHASSI.Vx_MAX) vx =  ROBOT_CHASSI.Vx_MAX;
		if (vx < -ROBOT_CHASSI.Vx_MAX) vx = -ROBOT_CHASSI.Vx_MAX;
	}
	if (ROBOT_CHASSI.Vy_MAX > 0.0f) {
		if (vy >  ROBOT_CHASSI.Vy_MAX) vy =  ROBOT_CHASSI.Vy_MAX;
		if (vy < -ROBOT_CHASSI.Vy_MAX) vy = -ROBOT_CHASSI.Vy_MAX;
	}
	if (ROBOT_CHASSI.Vw_MAX > 0.0f) {
		if (vw >  ROBOT_CHASSI.Vw_MAX) vw =  ROBOT_CHASSI.Vw_MAX;
		if (vw < -ROBOT_CHASSI.Vw_MAX) vw = -ROBOT_CHASSI.Vw_MAX;
	}
	ROBOT_CHASSI.Vx = vx;
	ROBOT_CHASSI.Vy = vy;
	ROBOT_CHASSI.Vw = vw;
}

void chassis_set_max_velocity(float vx_max, float vy_max, float vw_max)
{
	if (vx_max < 0) vx_max = -vx_max;
	if (vy_max < 0) vy_max = -vy_max;
	if (vw_max < 0) vw_max = -vw_max;
	ROBOT_CHASSI.Vx_MAX = vx_max;
	ROBOT_CHASSI.Vy_MAX = vy_max;
	ROBOT_CHASSI.Vw_MAX = vw_max;
}