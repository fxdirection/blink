#ifndef INC_2024RC_B_R1_CHASSIS_H
#define INC_2024RC_B_R1_CHASSIS_H

#define COS60 0.500000f
#define COS30 0.866025f
#define PI 3.14159265358979f
#define WHEEL_R 0.030f							 // 锟斤拷锟接半径(锟斤拷位锟斤拷m)
#define CHASSIS_R 0.10f							 // 锟斤拷锟教半径(锟斤拷位锟斤拷m)
#define RM_transition_MS 1.0f // 转锟斤拷锟斤拷锟劫度碉拷转锟斤拷 转锟斤拷一圈锟斤拷路锟斤拷 / 每锟斤拷转锟斤拷锟斤拷圈锟斤拷 锟姐法锟斤拷2*pi*R / 19锟斤拷3508锟斤拷锟斤拷锟戒）* 60锟斤拷锟斤拷转锟斤拷锟诫） rpm 2 m (锟斤拷位锟斤拷m/s)  rpm锟斤拷 转/锟斤拷锟斤拷
#define MS_transition_RM 1.0f  // 锟劫讹拷锟斤拷转锟劫碉拷转锟斤拷 m 2 rpm (锟斤拷位锟斤拷m/s)

#define Mecanum_Rx 0.5
#define Mecanum_Ry 0.5

typedef struct ROBOT_CHASSIS_T
{
	float Vx; // 锟斤拷锟秸碉拷锟斤拷锟劫讹拷
	float Vy;
	float Vw;

	float world_x; // 锟斤拷锟斤拷action锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟疥（锟斤拷锟斤拷锟剿碉拷锟斤拷实位锟矫ｏ拷
	float world_y;
	float world_w;

	float remote_x; // 锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟截碉拷锟劫讹拷锟斤拷锟斤拷值
	float remote_y;
	float remote_w;

	float plan_x; // 锟斤拷锟斤拷路锟斤拷锟芥划锟斤拷锟截碉拷锟劫讹拷锟斤拷锟斤拷值
	float plan_y;
	float plan_w;

	float Vy_MAX; // 锟斤拷锟斤拷俣锟斤拷锟斤拷锟�
	float Vx_MAX;
	float Vw_MAX;

	float Motor_Target_RPM[3]; // 3锟斤拷锟斤拷锟接碉拷目锟斤拷转锟斤拷

} ROBOT_CHASSIS;

extern ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void);
void Robot_Wheels_RPM_calculate(void);
void chassis_stop(void);
void Free_Control(void);

/* 杩滅▼璋冭瘯杩愯鏃朵娇鐢� */
void chassis_set_velocity(float vx, float vy, float vw);
void chassis_set_max_velocity(float vx_max, float vy_max, float vw_max);

#endif
