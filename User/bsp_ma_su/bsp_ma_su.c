/*
 * Optimized/adaptive implementation. This file preserves the legacy
 * external symbols used across the project while implementing a
 * clearer, parameter-driven PID initialization and update flow using
 * data from `motor_pid_params.h`.
 */
#include "bsp_ma_su.h"
#include "pid.h"
#include "GO-M8010-6.h"
#include "bsp_usart_dma.h"
#include "bsp_rc.h"
#include "math.h"
#include "./usart/bsp_debug_usart.h"
#include "bsp_key.h" 
#include <string.h>
#include "system_data.h"
#include "motor_pid_params.h"

/* Legacy globals (definitions) kept for compatibility */
MOTOR_send motor_control_clear;
MOTOR_send motor_control_data[10]= {0}; /* individual legacy structs may still be used */
MOTOR_recv motor_feedback_data[10] = {0}; /* individual legacy structs may still be used */

float Motor_speed_PID_OUT[10] = {0};
float Motor_rang_PID_OUT[10] = {0};

/* gait/tracking globals (keep original names) */
float rang__2 = 0, rang__3 = 0, rang__4 = 0, rang__5 = 0, rang__6 = 0, rang__7 = 0, rang__8 = 0, rang__9 = 0;
float foot_track_x1 = 0, foot_track_y1 = 0;
float foot_track_x2 = 0, foot_track_y2 = 0;
float foot_track_x3 = 0, foot_track_y3 = 0;
float foot_track_x4 = 0, foot_track_y4 = 0;
float Motor_set_Pos = 0.0f; 
int Motor_send_ID = 1; 
int time_currently = 0;
int time_turn = 0;
float ttl = 0.0f;
int step = 1;
float step_turn_k = 0.0004f;
int dog_rest_flag = 0;

//--�ȸ߱���//
	int leg_high = 200;
	int leg_high_tai_tui = 40;

/* small helper: clamp motor id to valid range */
static inline int _clamp_motor_id(int id) { if (id < 1) return 1; if (id > Mo_Count) return Mo_Count; return id; }

/* Initialize PID structs for all motors using MOTOR_* parameters */
void motor_pid_init_all(void)
{ 
	for (int id = 1; id <= Mo_Count; ++id) {
		PID_init(&Motor_speed_PID[id], PID_POSITION, Motor_speed_PID_data[id], MOTOR_SPEED_PID_MAX_OUT[id], MOTOR_SPEED_PID_MAX_I_OUT[id]);
		PID_init(&Motor_rang_PID[id], PID_POSITION, Motor_angle_PID_data[id], MOTOR_ANGLE_PID_MAX_OUT[id], MOTOR_ANGLE_PID_MAX_I_OUT[id]);
	}
	/* IMU Z axis if present */
	PID_init(&imu_Z_PID, PID_POSITION, IMU_Z_AXIS_PID_data, 0.0f, 0.0f);
}

int Motor_Mode_init(void)
{
	/* prepare a default stop packet */
	memset(&motor_control_clear, 0, sizeof(motor_control_clear));//���
	motor_control_clear.id = 15;
	motor_control_clear.mode = 1;
	modify_data(&motor_control_clear);

	/* initialize per-motor control frames to safe defaults */
	for (int id = 1; id <= Mo_Count; ++id) {
		motor_control_data[id].id = id;
		motor_control_data[id].mode = 1;
		motor_control_data[id].T = 0;
		motor_control_data[id].W = 0.0f;
		motor_control_data[id].Pos = 0.0f;
		motor_control_data[id].K_P = 0;
		motor_control_data[id].K_W = 0;
		modify_data(&motor_control_data[id]);
	}
	return 0;
}

/* Compute PID outputs for one motor (legacy wrapper) */
int Motor_pid_compute(int Motor_feedback_ID, float set_pos)
{
	int id = _clamp_motor_id(Motor_feedback_ID);
    Motor_Pos *motor_pos=&motor_pos_offset[id]; //�������л��ƫ������Ŀ��Ƕ�
	//�Ƕ�PID//
	Motor_rang_PID_OUT[id] = PID_calc(&Motor_rang_PID[id], motor_feedback_data[id].Pos+motor_pos->pos_offset,motor_pos->target_rang);
	//�ٶ�PID//
	Motor_speed_PID_OUT[id] = PID_calc(&Motor_speed_PID[id], motor_feedback_data[id].w, Motor_rang_PID_OUT[id]);
	return 0;
}

int Motor_data_update(int Motor_feedback_ID)
{
	int id = _clamp_motor_id(Motor_feedback_ID);
    if((dog_jump_time>500)&&(dog_jump_time<700)&&(rc_rc.s2 == 1))
	{
		Motor_speed_PID_OUT[id] = Motor_speed_PID_OUT[id]*1.50f;
	}
	motor_control_data[id].T = (int16_t)Motor_speed_PID_OUT[id];
	modify_data(&motor_control_data[id]);
	return 0;
}

int Motor_date_send(int *Motor_send_ID)
{
	if(Motor_send_ID == 0) return -1;
	int id = _clamp_motor_id(*Motor_send_ID);
	Usart_SendStr_length( USART6, ((uint8_t *)(&motor_control_data[id])),motor_control_data[id].hex_len );
	*Motor_send_ID = id % Mo_Count + 1; /* �߽����ã�����ѭ���� */
	return 0;
}

void motor_key_control(void)
{
	/* ��ѯ����״̬�Դ������ֹͣ������ */    
	if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
	{
		//pid����
		for (int id = 1; id <= Mo_Count; ++id) {
			PID_clear(&Motor_speed_PID[id]);
			PID_clear(&Motor_rang_PID[id]);
		}
		while(1)
		{
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor_control_clear)),motor_control_clear.hex_len );
		}
	}
}

//�ȹؽ��˶����?
void counter_motion(float X,float Y,float *range1, float *range2)
{
	float L = 0.0f;//���ȳ�
//	float L1 = 0.0f;//���ȳ�
//	float L2 = 0.0f;//���ȳ�
	float range_Leg = 0.0f;//�Ƚ�
	float range_separate = 0.0f;//�����?
//	X = X*0.333f;
//	Y = Y*0.333f;
	L = sqrt(X*X + Y*Y);
	range_Leg = asin(X/L);
//                             136mm   279mm
//range_separate = acos((L*L + L1*L1 - L2*L2)/(2*L1*L));
	range_separate = acos((L*L - 59345)/(279*L));

	*range1 = (3.1415f - (range_separate - range_Leg))*6.33f;
	*range2 = (3.1415f - (range_separate + range_Leg))*6.33f;
}

//��˹�?--------------------------̧��ʱ��----��ǰʱ��------ת�������?--ת��ǰʱ��-----������֧��-ģʽ------
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode)
{
	//--��Ƶѡ��----------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.s2 == 1))
		{
			leg_high = 220;
		}
		else
		{
			leg_high = 200;
		}
  //����ģʽʱ����λ��
	int flag = 1;
	
	//����ʼ��վ������2s---------------------------------------------------------
	if(Dog_Iinit <2000)
	{
		//���ֱ������?
		*X = 160 - 160*((float)Dog_Iinit/2000);
		*Y = 100 + 100*((float)Dog_Iinit/2000);
	}
	else
	{
	//-1-//�����Ⱥ��?------��֧��-------------------------------------------------------------------------------
	if(move_mode == 1)
	{
		//��ǰ��
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//̧��ʱ��-----̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//���ֱ������?
				*X = 70 - 140*((float)time_currently/Tm) ;
				*Y = leg_high;
			}
			else//����̧��ʱ������֧��ֱ�����?
			{
				//���Ľ��ŵ�ʱ��
				//��˰�����ǰ��?
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   -70  +   140*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//��˸�?     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));
////				//���Ľ��ŵ�ʱ��
////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////				{
////					*X = -70;
////					*Y = 200;
////				}
////				else
////				{
////					//��˰�����ǰ��?
////					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
////					*X =   -70  +   140*((float)time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.5f)));
////					//��˸�?     ̧�Ÿ߶�
////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.5f)));
////				}
			}
//---����ת��-----------------------------------------------------------
			if((rc_rc.ch0 > 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��2��-4-5-----------����ת����?--------------------------------------------------------
				if(((direction_mode ==3))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
				//-------��4��-8-9---------����ת����?--------------------------------------------------------
				if(((direction_mode ==4))&&step_turn_flag)
				{
					*X = (*X)*1.09f ;
				}
			}
			if((rc_rc.ch0 < 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��-4-5-----------����ת����?--------------------------------------------------------
				if(((direction_mode ==3))&&step_turn_flag)
				{
					*X = *X ;
				}
				//-------��-8-9-----------����ת����?--------------------------------------------------------
				if(((direction_mode ==4))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*( - rc_rc.ch0));
				}
			}
//��̬��λ----------------
			dog_rest_flag = 0;
		}
		//�����?----------------------------------------------------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//̧��ʱ��̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//���ֱ������?
				*X = -70 + 140*((float)time_currently/Tm);
				*Y = leg_high;
			}
			else//����̧��ʱ������֧��ֱ�����?
			{
				//���Ľ��ŵ�ʱ��
				//��˰�����ǰ��?
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   70 - 140*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//��˸�?     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));

//////////				//�Ľ��ŵ�ʱ��
//////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
//////////				{
//////////					*X = 70;
//////////					*Y = 200;
//////////				}
//////////				else
//////////				{
//////////					//��˰�����ǰ��?
//////////					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
//////////					*X =   70 - 140*((float)time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.5f)));
//////////					//��˸�?     ̧�Ÿ߶�
//////////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.5f)));
//////////				}
			}
			//��̬��λ----------------
			dog_rest_flag = 0;
		}
	}
//-1-//�����Ⱥ��?--��̧��-------------------------------------------------------------------------------
	if(move_mode == 2)
	{
		//��ǰ��
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//̧��ʱ��-----̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//��˰�����ǰ��?
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   -70 + 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//��˸�?     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//����̧��ʱ������֧��ֱ�����?
			{
				//���Ľ��ŵ�ʱ��
				//���ֱ������?
				*X = 70 - 140*((float)time_currently/Tm -1.0f);
				*Y = leg_high;
////////				//���Ľ��ŵ�ʱ��
////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////////				{
////////					*X = 70;
////////					*Y = 200;
////////				}
////////				else
////////				{
////////					//���ֱ������?
////////					*X = 70 - 140*((float)time_currently/Tm -1.5f);
////////					*Y = 200;
////////				}
			}
//---����ת��-----------------------------------------------------------
			//��ת--
			if((rc_rc.ch0 > 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��1��-2-3-----------����ת����?--------------------------------------------------------
				if(((direction_mode ==1))&&step_turn_flag)
				{
					*X = (*X)*1.09f  ;
				}
				//-------��2��-6-7----------����ת����?--------------------------------------------------------
				if(((direction_mode ==2))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
			}
			//��ת--
			if((rc_rc.ch0 < 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��-2-3-----------����ת����?--------------------------------------------------------
				if(((direction_mode ==1))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*(-rc_rc.ch0));
				}
				//-------��-4-5--------����ת����?--------------------------------------------------------
				if(((direction_mode ==2))&&step_turn_flag)
				{
					* X = *X;
				}
			}
//��̬��λ----------------
			dog_rest_flag = 0;
		}
		//�����?
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//̧��ʱ��̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//��˰�����ǰ��?
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   70 - 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//��˸�?     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//����̧��ʱ������֧��ֱ�����?
			{
				//���Ľ��ŵ�ʱ��
				//���ֱ������?
				*X = -70 + 140*((float)time_currently/Tm - 1.0f);
				*Y = leg_high;
////////				//�Ľ��ŵ�ʱ��
////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////////				{
////////					*X = -70;
////////					*Y = 200;
////////				}
////////				else
////////				{
////////					//���ֱ������?
////////					*X = -70 + 140*((float)time_currently/Tm - 1.5f);
////////					*Y = 200;
////////				}
			}
			//��̬��λ----------------
			dog_rest_flag = 0;
		}
	}
//-2-//ת��-----------------------------------------------------------------------------
	if(rc_rc.s1 == 2)
	{
		//����ǰ----- -X -----2,3
		if(direction_mode == 1)
		{
			//����ת----X-��ֵ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//���ֱ������?
					*X = 70-140*((float)time_turn/turn_Tm);
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
////////����ת----X-��ֵ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70-140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ������?
					*X = -70 + 140*((float)time_turn/turn_Tm - 1);
					*Y = 200;
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
		}
//����ǰ----- +X    6-7
		if(direction_mode == 2)
		{
////////����ת----X-��ֵ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//���ֱ������?
					*X = -70 + 140*((float)time_turn/turn_Tm);
					*Y = leg_high;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�?     
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
			//��̬��λ----------------
			dog_rest_flag = 0;
			}
////////����ת----X-��ֵ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ������?
					*X = 70-140*((float)time_turn/turn_Tm - 1);
					*Y = 200;
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
		}
		//��̧----- -X     4-5
		if(direction_mode == 3)
		{
////////����ת----X-��ֵ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ������?
					*X = -70 + 140*((float)time_turn/turn_Tm -1 );
					*Y = 200;
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
////////����ת----X-��ֵ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_currently > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//���ֱ������?
					*X = 70-140*((float)time_turn/turn_Tm  );
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
		}
		//��̧----- +X    8-9
		if(direction_mode == 4)
		{
/////////����ת----X-��ֵ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ������?
					*X = 70 - 140*((float)time_turn/turn_Tm -1);
					*Y = 200;
				}
			//��̬��λ----------------
			dog_rest_flag = 0;
			}
////////����ת----X-��ֵ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ħ�����ؽ׶�
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<Tm)||(Tm == time_turn)))
				{
					//���ֱ������?
					*X = -70 + 140*((float)time_turn/Tm  );
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ��?
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/Tm - 1)));
					//��˸�?     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/Tm - 1)));
				}
				//��̬��λ----------------
				dog_rest_flag = 0;
			}
		}
	}
//-1-rc_rc.s1 == 1--������ģʽ---/////////////////////////////////////////////////////////
	if(rc_rc.s1 == 1)
	{
		flag = 0;
		//��---------------------------
		if(rc_rc.s2 == 1)
		{
			dog_jump_flag = 1;//����ʼ��ʱ��־
				if(500>dog_jump_time)
				{
					//�¶�
					*X = -50;
					//�¶׸߶�
					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time/1000)));
				}
				if((500<dog_jump_time)&&(700>dog_jump_time))
				{
					//����
					*X = -220;
					*Y = 350;
				//dog_jump_flag = 0;//����ʼ��ʱ��־
				}
//�������?---------------------------------------------------------------------------
				if((700<dog_jump_time)&&(900>dog_jump_time))
				{
					//ǰ��
					if((direction_mode == 1)||(direction_mode == 3))
					{
						*X = -220 + 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
						//�����¶׸߶�
						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					}
					//����
					if((direction_mode == 2)||(direction_mode == 4))
					{
						*X = -220 + 260*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
						//�����¶׸߶�
						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					}
				}
				//�����?��λ
				if((1400<dog_jump_time)&&(1900>dog_jump_time))
				{
					*X = -50 + 50*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));

					*Y = 160 + 40*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
				}

		}
		else//--��������ģʽ����ʱ������
		{
			dog_jump_time = 0;
			dog_jump_flag = 0;
		}
//////¥����,��ǰ��---------------------------
		if(rc_rc.s2 == 2)
		{
			dog_jump_flag_2 = 1;//����ʼ��ʱ��־
				if(500>dog_jump_time_2)
				{
					//�¶�
					*X = -50;
					//�¶׸߶�
					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_2/1000)));
				}
				if((500<dog_jump_time_2)&&(700>dog_jump_time_2))
				{
					//����
					*X = -200;
					*Y = 250;
				//dog_jump_flag = 0;//����ʼ��ʱ��־
				}
//�������?---------------------------------------------------------------------------
				if((700<dog_jump_time_2)&&(900>dog_jump_time_2))
				{
					//ǰ��
					if((direction_mode == 1)||(direction_mode == 3))
					{
						*X = -200 + 210*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
						//�����¶׸߶�
						*Y = 210 - 50*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					}
					//����
					if((direction_mode == 2)||(direction_mode == 4))
					{
						*X = -200 + 200*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
						//�����¶׸߶�
						*Y = 250 - 90*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					}
				}
				//�����?��λ
				if((1400<dog_jump_time_2)&&(1900>dog_jump_time_2))
				{
					*X = 10 - 10*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));

					*Y = 160 + 40*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
				}

		}
		else//--��������ģʽ����ʱ������
		{
			dog_jump_time_2 = 0;
			dog_jump_flag_2 = 0;
		}
//̤��--------------------------------------------------------------------------------------
////////		if(rc_rc.s2 == 2)
////////		{
////////			if(move_mode == 1)
////////			{
////////				if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
////////				{
////////					*Y = 250;
////////				}
////////				else
////////				{
////////					if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.1f))
////////					{
////////						//�Ľ��ŵ�ʱ��
////////						*Y = 250;
////////					}
////////					else
////////					{
////////						*Y = 250 - 50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.1f)));
////////					}
////////				}
////////			}
////////			if(move_mode == 2)
////////			{
////////				if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
////////				{
////////					*Y = 250 - 50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm)));
////////				}
////////				else
////////				{
////////					if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.1f))
////////					{//�Ľ��ŵ�ʱ��
////////						*Y = 250;
////////					}
////////					else
////////					{
////////						*Y = 250;
////////					}
////////				}
////////			}
////////		}
	}
//////////-1-rc_rc.s1 == 1--������ģʽ---/////////////////////////////////////////////////////////
////////	if(rc_rc.s1 == 1)
////////	{
////////		flag = 0;
////////		//��---------------------------
////////		if(rc_rc.s2 == 1)
////////		{
////////			dog_jump_flag = 1;//����ʼ��ʱ��־
////////				if(500>dog_jump_time)
////////				{
////////					//�¶�
////////					*X = -50;
////////					//�¶׸߶�
////////					*Y = 200 - 40*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time/1000)));
////////				}
////////				if((500<dog_jump_time)&&(700>dog_jump_time))
////////				{
////////					//����
////////					*X = -220;
////////					*Y = 350;
////////				//dog_jump_flag = 0;//����ʼ��ʱ��־
////////				}
//////////�������?---------------------------------------------------------------------------
////////				if((700<dog_jump_time)&&(900>dog_jump_time))
////////				{
////////					//ǰ��
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = -150 + 170*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////						//�����¶׸߶�
////////						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////					//����
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = -150 + 210*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////						//�����¶׸߶�
////////						*Y = 350 - 100*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////				}
////////				//�����?��λ
////////				if((1400<dog_jump_time)&&(1900>dog_jump_time))
////////				{
////////					//ǰ��
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = 20 - 40*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////						//�����¶׸߶�
////////						*Y = 160 ;//- 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////					//����
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = 60 ;//+ 50*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////						//�����¶׸߶�
////////						*Y = 250;//350 - 100*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////					}

////////				}

////////		}
////////		else//--��������ģʽ����ʱ������
////////		{
////////			dog_jump_time = 0;
////////			dog_jump_flag = 0;
////////		}
//////////////¥����,��ǰ��---------------------------
////////		if(rc_rc.s2 == 2)
////////		{
////////			dog_jump_flag_2 = 1;//����ʼ��ʱ��־
////////				if(500>dog_jump_time_2)
////////				{
////////					//�¶�
////////					*X = -50;
////////					//�¶׸߶�
////////					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_2/1000)));
////////				}
////////				if((500<dog_jump_time_2)&&(700>dog_jump_time_2))
////////				{
////////					//����
////////					*X = -200;
////////					*Y = 250;
////////				//dog_jump_flag = 0;//����ʼ��ʱ��־
////////				}
//////////�������?---------------------------------------------------------------------------
////////				if((700<dog_jump_time_2)&&(900>dog_jump_time_2))
////////				{
////////					//ǰ��
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = -200 + 210*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////						//�����¶׸߶�
////////						*Y = 210 - 50*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////					}
////////					//����
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = -200 + 200*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////						//�����¶׸߶�
////////						*Y = 250 - 90*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////					}
////////				}
////////				//�����?��λ
////////				if((1400<dog_jump_time_2)&&(1900>dog_jump_time_2))
////////				{
////////					*X = 10 - 10*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));

////////					*Y = 160 + 40*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
////////				}

////////		}
////////		else//--��������ģʽ����ʱ������
////////		{
////////			dog_jump_time_2 = 0;
////////			dog_jump_flag_2 = 0;
////////		}
//////////̤��--------------------------------------------------------------------------------------
////////////////		if(rc_rc.s2 == 2)
////////////////		{
////////////////			if(move_mode == 1)
////////////////			{
////////////////				if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
////////////////				{
////////////////					*Y = 250;
////////////////				}
////////////////				else
////////////////				{
////////////////					if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.1f))
////////////////					{
////////////////						//�Ľ��ŵ�ʱ��
////////////////						*Y = 250;
////////////////					}
////////////////					else
////////////////					{
////////////////						*Y = 250 - 50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.1f)));
////////////////					}
////////////////				}
////////////////			}
////////////////			if(move_mode == 2)
////////////////			{
////////////////				if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
////////////////				{
////////////////					*Y = 250 - 50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm)));
////////////////				}
////////////////				else
////////////////				{
////////////////					if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.1f))
////////////////					{//�Ľ��ŵ�ʱ��
////////////////						*Y = 250;
////////////////					}
////////////////					else
////////////////					{
////////////////						*Y = 250;
////////////////					}
////////////////				}
////////////////			}
////////////////		}
////////	}
////////	else//--��������ģʽ����ʱ������
////////	{
////////		dog_jump_time = 0;
////////		dog_jump_flag = 0;
////////	}
////////////////////////////////////////////////////////////////////////////////////////
	//����ģʽʱ����λ
		if(flag)
		{
			*Y = 200;
			*X = 0;
		} 
	}
}

void jump_backflip(int backflip_time)
{
//		flag = 0;
//		//��---------------------------
//		if(rc_rc.s2 == 2)
//		{
//			backflip_flag = 1;//����ʼ��ʱ��־
//			if(500>backflip_time)
//			{
//				//�¶�
//				*X = -50;
//				//�¶׸߶�
//				*Y = 250 - 100*(0.5f - 0.5f*cos(6.283f*((float)backflip_time/1000)));
//			}
//			if((500<backflip_time)&&(600>backflip_time))
//			{
//				//����
//				*X = -100;
//				*Y = 300;
//			//dog_jump_flag = 0;//����ʼ��ʱ��־
//			}
//			if((600<backflip_time)&&(900>backflip_time))
//			{
//				*X = -100 + 150*((float)(backflip_time-600)/600 - 0.1591f*sin(6.283f*((float)(backflip_time-600)/600)));
//				//�����¶׸߶�
//				*Y = 300 - 120*(0.5f - 0.5f*cos(6.283f*((float)(backflip_time-600)/600)));
//			}
//			//�����?��λ
//			if((900<backflip_time)&&(1400>backflip_time))
//			{
//				*X = 50 - 50*((float)(backflip_time-900)/500 - 0.1591f*sin(6.283f*((float)(backflip_time-900)/500)));

//				*Y = 180 + 70*((float)(backflip_time-900)/500 - 0.1591f*sin(6.283f*((float)(backflip_time-900)/500)));
//			}
//			
//		}
//		else//--��������ģʽ����ʱ������
//		{
//			backflip_time = 0;
//			backflip_flag = 0;
//		}
}

void Dog_sport_mode(void)
{
	//��ģʽ
	if(rc_rc.s1 == 1)
	{ 
		//����
		if(rc_rc.s2 == 1)
		{
		
		}
		//׼������
		if(rc_rc.s2 == 3)
		{
			
		}
	}
	//����ģʽ
	if(rc_rc.s1 == 1)
	{
		//�첽
		if(rc_rc.s2 == 1)
		{
		
		}
		//����
		if(rc_rc.s2 == 3)
		{
			
		}
		//����
		if(rc_rc.s2 == 2)
		{
			
		}
	}
	//ת��ģʽ
	if(rc_rc.s1 == 1)
	{
		//�첽
		if(rc_rc.s2 == 1)
		{
		
		}
		//����
		if(rc_rc.s2 == 3)
		{
			
		}
		//����
		if(rc_rc.s2 == 2)
		{
			
		}
	}
}



