#ifndef _MA_SU
#define	_MA_SU

#include "stm32f4xx.h"
#include "struct_typedef.h"
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
//以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
extern	MOTOR_send motor_control_clear; //电机停止
extern	MOTOR_send motor_control_data[Mo_Count+1];    //电机控制数据
extern	MOTOR_recv motor_feedback_data[Mo_Count+1];	 //电机反馈数据
//电机pid计算输出
extern float Motor_speed_PID_OUT[Mo_Count+1];
extern float Motor_rang_PID_OUT[Mo_Count+1];
//电机pid参数数据
extern const fp32 Motor_speed_PID_data[Mo_Count+1][3];
extern const fp32 Motor_rang_PID_data[Mo_Count+1][3];
//电机目标位置变量
extern float rang__[Mo_Count+1];
//足端轨迹
	//--1号腿足端轨迹坐标
	extern float foot_track_x1 ;
	extern float foot_track_y1 ;
	//--2号腿足端轨迹坐标
	extern float foot_track_x2 ;
	extern float foot_track_y2 ;
	//--3号腿足端轨迹坐标
	extern float foot_track_x3 ;
	extern float foot_track_y3 ;
	//--4号腿足端轨迹坐标
	extern float foot_track_x4 ;
	extern float foot_track_y4 ;
	extern int time_currently;//步态当前时间
	extern int time_turn ;//转向当前时间
    //跳------------------------------------------------------------------------------------
		//跳开始计时标志
		extern int dog_jump_flag;
		//--跳--时间------------
		extern int dog_jump_time;
		//--跳--时间------------
		extern int dog_jump_time_2 ;
		//跳开始计时标志
		extern int dog_jump_flag_2 ;
		//空翻开始计时标志
		extern int backflip_flag ;
		//空翻计时
		extern int backflip_time ;
//-IMC-陀螺仪使用-----------------------------------------------------------------
extern float imu_Rx_data[4][4];
//---步幅转向标志--------------------------------------------------------
extern int step_turn_flag;
#endif /* _MA_SU */

