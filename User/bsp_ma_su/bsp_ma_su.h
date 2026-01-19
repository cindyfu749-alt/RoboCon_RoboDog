#ifndef _MA_SU
#define	_MA_SU

#include "stm32f4xx.h"
#include "struct_typedef.h"
#include  "GO-M8010-6.h"
#include "pid.h"
#include "system_data.h"
/* ==================== PID 参数表 ==================== */

/** PID 参数表：[速度Kp, 速度Ki, 速度Kd] */
extern const fp32 motor_speed_pid_params[10][3];

/** PID 参数表：[位置Kp, 位置Ki, 位置Kd] */
extern const fp32 motor_angle_pid_params[10][3];

/** 电机控制和反馈数据数组 */
extern MOTOR_send motor_ctrl[10];
extern MOTOR_recv motor_fdbk[10];

/** PID 输出数组 */
extern float motor_speed_pid_out[10];
extern float motor_angle_pid_out[10];

/* ==================== 函数声明 ==================== */

int Motor_pid_count(int Motor_feedback_ID,float set_pos);
int Motor_Mode_Init(void);
int Motor_data_update(int Motor_feedback_ID);
int Motor_date_send(int*Motor_send_ID);
void Motor_Key (void);
void PID_Init(void);
void Pos_speed_control(float feedback_pos,float set_pos,float pos_speed);
void counter_motion(float X,float Y,float *range1, float *range2);
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode);
void jump_backflip(int backflip_time);
////////////////////////////////////////int Dog_Init(int motor_id);
extern float rang__2, rang__3, rang__4, rang__5, rang__6, rang__7, rang__8, rang__9;

#endif /* _MA_SU */

