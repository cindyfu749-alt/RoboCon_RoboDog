#ifndef _MA_SU
#define	_MA_SU

#include "stm32f4xx.h"
#include "struct_typedef.h"
#include  "GO-M8010-6.h"
#include "pid.h"
#include "system_data.h"
/* ==================== 结构体定义==================== */
typedef struct
{
	float T;
	float W;
	float Pos;
}Motor_Feedback;

/* ==================== 函数声明 ==================== */

int Motor_pid_count(int Motor_feedback_ID,float set_pos);
int Motor_Mode_Init(void);
int Motor_data_update(int Motor_feedback_ID);
int Motor_date_send(int*Motor_send_ID);
void Motor_Key (void);
void PID_Init(void);
void motor_pid_init_all(void);
void motor_pos_init(void);
void Pos_speed_control(float feedback_pos,float set_pos,float pos_speed);
void counter_motion(float X,float Y,float *range1, float *range2);
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode);
void jump_backflip(int backflip_time);
void motor_key_control(void);
void motor_set_control_interval(uint32_t interval_ms);
uint8_t motor_get_running_state(void);
static inline int _clamp_motor_id(int id){ if (id < 1) return 1; if (id > Mo_Count) return Mo_Count; return id; };
////////////////////////////////////////int Dog_Init(int motor_id);
#endif /* _MA_SU */

