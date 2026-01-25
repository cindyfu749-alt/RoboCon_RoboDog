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
#include "control_task.h"
MOTOR_send motor_control_clear;
MOTOR_send motor_control_data[10]= {0}; 
MOTOR_recv motor_feedback_data[10] = {0}; 
float Motor_speed_PID_OUT[10];    ///< ËÙ¶È»· PID Êä³ö (1-9)
float Motor_rang_PID_OUT[10];    ///< Î»ÖÃ»· PID Êä³ö (1-9) 
float Motor_set_Pos = 0.0f; 
float step_turn_k = 0.0004f;
int dog_rest_flag = 0;
int leg_high = 200;  //--ÍÈ¸ß±äÁ¿//
int leg_high_tai_tui = 40;
/* µç»úÎ»ÖÃÆ«ÒÆÁ¿ºÍÄ¿±ê½Ç¶È±í£¨ÔËÐÐÊ±³õÊ¼»¯£© */
Motor_Pos motor_pos_offset[10];
/* PID²ÎÊý³õÊ¼»¯ */
void motor_pid_init_all(void)
{ 
	for (int id = 1; id < Mo_Count; id++) {
		PID_init(&Motor_speed_PID[id], PID_POSITION, Motor_speed_PID_data[id], MOTOR_SPEED_PID_MAX_OUT[id], MOTOR_SPEED_PID_MAX_I_OUT[id]);
		PID_init(&Motor_rang_PID[id], PID_POSITION, Motor_angle_PID_data[id], MOTOR_ANGLE_PID_MAX_OUT[id], MOTOR_ANGLE_PID_MAX_I_OUT[id]);
	}
	/* IMU Z axis if present */
	PID_init(&imu_Z_PID, PID_POSITION, IMU_Z_AXIS_PID_data, 0.0f, 0.0f);
}

/**
 * @brief µç»úÎ»ÖÃÆ«ÒÆ±íÔËÐÐÊ±³õÊ¼»¯
 * @details ½« rang__* ±äÁ¿(ÓÉconter_motionÊä³öµÃµ½)Ó³Éäµ½ motor_pos_offset[]
 */
void motor_pos_init(void)
{
	motor_pos_offset[0] = (Motor_Pos){0.0f, 0.0f};             // [0] Î´Ê¹ÓÃ
	motor_pos_offset[1] = (Motor_Pos){0.0f, rang__2};          // [1] Î´Ê¹ÓÃ
	motor_pos_offset[2] = (Motor_Pos){+11.184f, rang__2};      // [2] µç»ú2
	motor_pos_offset[3] = (Motor_Pos){-9.080f, rang__3};       // [3] µç»ú3
	motor_pos_offset[4] = (Motor_Pos){+0.501f, -rang__4};      // [4] µç»ú4
	motor_pos_offset[5] = (Motor_Pos){-18.194f, -rang__5};     // [5] µç»ú5
	motor_pos_offset[6] = (Motor_Pos){-12.201f, -rang__6};     // [6] µç»ú6
	motor_pos_offset[7] = (Motor_Pos){+4.448f, -rang__7};      // [7] µç»ú7
	motor_pos_offset[8] = (Motor_Pos){-10.007f, rang__8};      // [8] µç»ú8
	motor_pos_offset[9] = (Motor_Pos){+6.888f, rang__9};       // [9] µç»ú9
}

//µç»úÄ£Ê½³õÊ¼»¯//
int Motor_Mode_init(void)
{
	/* µç»úÍ£Ö¹½á¹¹Ìå */
	memset(&motor_control_clear, 0, sizeof(motor_control_clear));//Çå¿Õ
	motor_control_clear.id = 15;
	motor_control_clear.mode = 1;
	modify_data(&motor_control_clear);

	/* ³õÊ¼»¯µç»ú¿ØÖÆÊý¾Ý */
	for (int id = 1; id <= Mo_Count; ++id) {
		motor_control_data[id].id = id;
		motor_control_data[id].mode = 1;
		motor_control_data[id].T = 0;  //¿ØÖÆÊ±£¬Ö»¶ÔÅ¤¾Ø¸üÐÂ
		motor_control_data[id].W = 0.0f; 
		motor_control_data[id].Pos = 0.0f;
		motor_control_data[id].K_P = 0;
		motor_control_data[id].K_W = 0;
		modify_data(&motor_control_data[id]);
	}
	return 0;
}

/* PIDÊä³öº¯Êý */
int Motor_pid_compute(int Motor_feedback_ID, float set_pos)
{
	int id = _clamp_motor_id(Motor_feedback_ID);
    Motor_Pos *motor_pos=&motor_pos_offset[id]; //´ÓÊý×éÖÐ»ñµÃÆ«ÒÆÁ¿ºÍÄ¿±ê½Ç¶È
	//½Ç¶ÈPID//
	Motor_rang_PID_OUT[id] = PID_calc(&Motor_rang_PID[id], motor_feedback_data[id].Pos+motor_pos->pos_offset,motor_pos->target_rang);
	//ËÙ¶ÈPID//
	Motor_speed_PID_OUT[id] = PID_calc(&Motor_speed_PID[id], motor_feedback_data[id].W, Motor_rang_PID_OUT[id]);
	return 0;
}

int Motor_data_update(int Motor_feedback_ID)
{
	int id = _clamp_motor_id(Motor_feedback_ID);
	dog_state_t* state = control_task_get_state();
    if((state->jump_time>500)&&(state->jump_time<700)&&(rc_rc.s2 == 1))
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
	for(int i=1;i<Mo_Count;i++)
	{
      Usart_SendStr_length( USART6, 
	  ((uint8_t *)(&motor_control_data[id])),
	   motor_control_data[id].hex_len );
	}
	return 0;
}

//***********ÄæÔË¶¯Ñ§¼ÆËã ½«x£¬y×ø±ê×ª»¯Îªµç»ú½Ç¶È***************//
void counter_motion(float X,float Y,float *range1, float *range2)
{
	float L = 0.0f;//ÍÈ³¤
//	float L1 = 0.0f;//ÍÈ³¤
//	float L2 = 0.0f;//ÍÈ³¤
	float range_Leg = 0.0f;//ÍÈ³¤
	float range_separate = 0.0f;//ÍÈ³¤
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

//ÄæÔË¶¯Ñ§¼ÆËã----Ì§ÍÈÊ±----Ç°Ê±-------×ªÍÈÊ±------×ªÍÈÇ°Ê±-----Ì§ÍÈÖ§ÍÈ-Ä£Ê½//
void foot_track (float *X,float *Y,int Tm,int time_currently,int turn_Tm,int time_turn,int move_mode,int direction_mode)
{
	//--ÆµÂÊÑ¡Ôñ----------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.s2 == 1))
		{
			leg_high = 220;
		}
		else
		{
			leg_high = 200;
		}
  //ÄæÔË¶¯Ñ§Ä£Ê½Ê±¼ÆËãÎ»ÒÆ
	int flag = 1;
	dog_state_t* state = control_task_get_state();
	//¹·³õÊ¼»¯Õ¾Á¢ÆðÀ´2s//
	if(state->dog_init_time < 2000)
	{
		//×ã¶ËÖ±ÏßÏòºó¹ì¼£
		*X = 160 - 160*((float)state->dog_init_time/2000);
		*Y = 100 + 100*((float)state->dog_init_time/2000);
	}
	else
	{
	//-1-//ÐÐ×ßÏÈºó½Å------ÏÈÖ§³Å-------------------------------------------------------------------------------
	if(move_mode == 1)
	{
		//ÏòÇ°×ß
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//Ì§½ÅÊ±¼ä-----Ì§½Å
			if(((state->time_currently == 0)||(state->time_currently > 0))&&((state->time_currently<Tm)||(Tm == state->time_currently)))
			{
				//×ã¶ËÖ±ÏßÏòºó¹ì¼£
				*X = 70 - 140*((float)state->time_currently/Tm) ;
				*Y = leg_high;
			}
			else///¹ýÁËÌ§½ÅÊ±¼äºó£¬×ã¶ËÖ§³ÅÖ±ÏßÏòºó
			{
				//ÎÞËÄ½Å×ÅµØÊ±¼ä
				//×ã¶Ë°ÚÏßÏòÇ°¹ì¼£
				//ÆðÊ¼µã-----²½³¤------------µ±Ç°Ê±¼ä--Ì§½ÅÊ±³¤
				*X =   -70  +   140*((float)state->time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)state->time_currently/Tm - 1.0f)));
				//×ã¶Ë¸ß     Ì§½Å¸ß¶È
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)state->time_currently/Tm - 1.0f)));
			{
				//×ã¶ËÖ±ÏßÏòºó¹ì¼£
				*X = 70 - 140*((float)state->time_currently/Tm) ;
				*Y = leg_high;
			}
			else///¹ýÁËÌ§½ÅÊ±¼äºó£¬×ã¶ËÖ§³ÅÖ±ÏßÏòºó
			{
				//ÎÞËÄ½Å×ÅµØÊ±¼ä
				//×ã¶Ë°ÚÏßÏòÇ°¹ì¼£
				//ÆðÊ¼µã-----²½³¤------------µ±Ç°Ê±¼ä--Ì§½ÅÊ±³¤
				*X =   -70  +   140*((float)state->time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)state->time_currently/Tm - 1.0f)));
				//×ã¶Ë¸ß     Ì§½Å¸ß¶È
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)state->time_currently/Tm - 1.0f)));
////				//ÓÐËÄ½Å×ÅµØÊ±¼ä
////				if(((float)state->time_currently/Tm > 1)&&((float)state->time_currently/Tm <1.5f))
////				{
////					*X = -70;
////					*Y = 200;
////				}
////				else
////				{
////					//×ã¶Ë°ÚÏßÏòÇ°¹ì¼£
////					//ÆðÊ¼µã-----²½³¤------------µ±Ç°Ê±¼ä--Ì§½ÅÊ±³¤
////					*X =   -70  +   140*((float)state->time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)state->time_currently/Tm - 1.5f)));
////					//×ã¶Ë¸ß     Ì§½Å¸ß¶È
////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)state->time_currently/Tm - 1.5f)));
////				}
			}
//---²½·ù×ªÏò----------------------------------------------------------
			if((rc_rc.ch0 > 0)||(state->step_turn_flag))
			{
				state->step_turn_flag = 1;
				//-------ÍÈ2ºÅ-4-5-----------²½·ù×ªÏò±êÖ¾--------------------------------------------------------
				if(((direction_mode ==3))&&state->step_turn_flag)
				{
					//ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½Ïµï¿½ï¿½step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
				//-------ï¿½ï¿½4ï¿½ï¿½-8-9---------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==4))&&state->step_turn_flag)
				{
					*X = (*X)*1.09f ;
				}
			}
			if((rc_rc.ch0 < 0)||(state->step_turn_flag))
			{
				state->step_turn_flag = 1;
				//-------ï¿½ï¿½-4-5-----------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==3))&&state->step_turn_flag)
				{
					*X = *X ;
				}
				//-------ï¿½ï¿½-8-9-----------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==4))&&state->step_turn_flag)
				{
					//ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½Ïµï¿½ï¿½step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*( - rc_rc.ch0));
				}
			}
//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			state->dog_rest_flag = 0;
		}
		//ï¿½ï¿½ï¿½ï¿½ï¿?----------------------------------------------------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//Ì§ï¿½ï¿½Ê±ï¿½ï¿½Ì§ï¿½ï¿½
			if(((state->time_currently == 0)||(state->time_currently > 0))&&((state->time_currently<Tm)||(Tm == state->time_currently)))
			{
				//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
				*X = -70 + 140*((float)state->time_currently/Tm);
				*Y = leg_high;
			}
			else//ï¿½ï¿½ï¿½ï¿½Ì§ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö§ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿?
			{
				//ï¿½ï¿½ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
				//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
				//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
				*X =   70 - 140*((float)state->time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)state->time_currently/Tm - 1.0f)));
				//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)state->time_currently/Tm - 1.0f)));

//////////				//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
//////////				if(((float)state->time_currently/Tm > 1)&&((float)state->time_currently/Tm <1.5f))
//////////				{
//////////					*X = 70;
//////////					*Y = 200;
//////////				}
//////////				else
//////////				{
//////////					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
//////////					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
//////////					*X =   70 - 140*((float)state->time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)state->time_currently/Tm - 1.5f)));
//////////					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
//////////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)state->time_currently/Tm - 1.5f)));
//////////				}
			}
			//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			state->dog_rest_flag = 0;
		}
	}
//-1-//ï¿½ï¿½ï¿½ï¿½ï¿½Èºï¿½ï¿?--ï¿½ï¿½Ì§ï¿½ï¿½-------------------------------------------------------------------------------
	if(move_mode == 2)
	{
		//ï¿½ï¿½Ç°ï¿½ï¿½
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//Ì§ï¿½ï¿½Ê±ï¿½ï¿½-----Ì§ï¿½ï¿½
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
				//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
				*X =   -70 + 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//ï¿½ï¿½ï¿½ï¿½Ì§ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö§ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿?
			{
				//ï¿½ï¿½ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
				//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
				*X = 70 - 140*((float)time_currently/Tm -1.0f);
				*Y = leg_high;
////////				//ï¿½ï¿½ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////////				{
////////					*X = 70;
////////					*Y = 200;
////////				}
////////				else
////////				{
////////					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
////////					*X = 70 - 140*((float)time_currently/Tm -1.5f);
////////					*Y = 200;
////////				}
			}
//---ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½-----------------------------------------------------------
			//ï¿½ï¿½×ª--
			if((rc_rc.ch0 > 0)||(state->step_turn_flag))
			{
				state->step_turn_flag = 1;
				//-------ï¿½ï¿½1ï¿½ï¿½-2-3-----------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==1))&&state->step_turn_flag)
				{
					*X = (*X)*1.09f  ;
				}
				//-------ï¿½ï¿½2ï¿½ï¿½-6-7----------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==2))&&state->step_turn_flag)
				{
					//ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½Ïµï¿½ï¿½step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
			}
			//ï¿½ï¿½×ª--
			if((rc_rc.ch0 < 0)||(state->step_turn_flag))
			{
				state->step_turn_flag = 1;
				//-------ï¿½ï¿½-2-3-----------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==1))&&state->step_turn_flag)
				{
					//ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½Ïµï¿½ï¿½step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*(-rc_rc.ch0));
				}
				//-------ï¿½ï¿½-4-5--------ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ö?--------------------------------------------------------
				if(((direction_mode ==2))&&state->step_turn_flag)
				{
					* X = *X;
				}
			}
//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			state->dog_rest_flag = 0;
		}
		//ï¿½ï¿½ï¿½ï¿½ï¿?
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//Ì§ï¿½ï¿½Ê±ï¿½ï¿½Ì§ï¿½ï¿½
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
				//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
				*X =   70 - 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//ï¿½ï¿½ï¿½ï¿½Ì§ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö§ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿?
			{
				//ï¿½ï¿½ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
				//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
				*X = -70 + 140*((float)time_currently/Tm - 1.0f);
				*Y = leg_high;
////////				//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////////				{
////////					*X = -70;
////////					*Y = 200;
////////				}
////////				else
////////				{
////////					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
////////					*X = -70 + 140*((float)time_currently/Tm - 1.5f);
////////					*Y = 200;
////////				}
			}
			//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			dog_rest_flag = 0;
		}
	}
//-2-//×ªï¿½ï¿½-----------------------------------------------------------------------------
	if(rc_rc.s1 == 2)
	{
		//ï¿½ï¿½ï¿½ï¿½Ç°----- -X -----2,3
		if(direction_mode == 1)
		{
			//ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = 70-140*((float)time_turn/turn_Tm);
					*Y = 200;
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = -70 + 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = 70-140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = -70 + 140*((float)time_turn/turn_Tm - 1);
					*Y = 200;
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
		}
//ï¿½ï¿½ï¿½ï¿½Ç°----- +X    6-7
		if(direction_mode == 2)
		{
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = -70 + 140*((float)time_turn/turn_Tm);
					*Y = leg_high;
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = 70 - 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//ï¿½ï¿½Ë¸ï¿?     
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
			//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			dog_rest_flag = 0;
			}
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = 70-140*((float)time_turn/turn_Tm - 1);
					*Y = 200;
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
		}
		//ï¿½ï¿½Ì§----- -X     4-5
		if(direction_mode == 3)
		{
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = 70 - 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = -70 + 140*((float)time_turn/turn_Tm -1 );
					*Y = 200;
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_currently > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = 70-140*((float)time_turn/turn_Tm  );
					*Y = 200;
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = -70 + 140*((float)time_turn/turn_Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm - 1)));
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
		}
		//ï¿½ï¿½Ì§----- +X    8-9
		if(direction_mode == 4)
		{
/////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 > 330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<turn_Tm)||(turn_Tm == time_turn)))
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = 70 - 140*((float)time_turn/turn_Tm -1);
					*Y = 200;
				}
			//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
			dog_rest_flag = 0;
			}
////////ï¿½ï¿½ï¿½ï¿½×ª----X-ï¿½ï¿½Öµ
			if(rc_rc.ch0 < -330)
			{
				flag = 0;
				//Ä¦ï¿½ï¿½ï¿½ï¿½ï¿½Ø½×¶ï¿½
				if(((time_turn == 0)||(time_turn > 0))&&((time_turn<Tm)||(Tm == time_turn)))
				{
					//ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ì¼?
					*X = -70 + 140*((float)time_turn/Tm  );
					*Y = 200;
				}
				else//Ì§ï¿½Å¸ï¿½Î»ï¿½×¶ï¿½
				{
					//ï¿½ï¿½Ë°ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½ì¼?
					//ï¿½ï¿½Ê¼ï¿½ï¿½-----ï¿½ï¿½ï¿½ï¿½------------ï¿½ï¿½Ç°Ê±ï¿½ï¿½--Ì§ï¿½ï¿½Ê±ï¿½ï¿½
					*X = 70 - 140*((float)time_turn/Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/Tm - 1)));
					//ï¿½ï¿½Ë¸ï¿?     Ì§ï¿½Å¸ß¶ï¿½
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/Tm - 1)));
				}
				//ï¿½ï¿½Ì¬ï¿½ï¿½Î»----------------
				dog_rest_flag = 0;
			}
		}
	}
//-1-rc_rc.s1 == 1--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½---/////////////////////////////////////////////////////////
	if(rc_rc.s1 == 1)
	{
		flag = 0;
		//ï¿½ï¿½---------------------------
		if(rc_rc.s2 == 1)
		{
			dog_jump_flag = 1;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
				if(500>dog_jump_time)
				{
					//ï¿½Â¶ï¿½
					*X = -50;
					//ï¿½Â¶×¸ß¶ï¿½
					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time/1000)));
				}
				if((500<dog_jump_time)&&(700>dog_jump_time))
				{
					//ï¿½ï¿½ï¿½ï¿½
					*X = -220;
					*Y = 350;
				//dog_jump_flag = 0;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
				}
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?---------------------------------------------------------------------------
				if((700<dog_jump_time)&&(900>dog_jump_time))
				{
					//Ç°ï¿½ï¿½
					if((direction_mode == 1)||(direction_mode == 3))
					{
						*X = -220 + 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					}
					//ï¿½ï¿½ï¿½ï¿½
					if((direction_mode == 2)||(direction_mode == 4))
					{
						*X = -220 + 260*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
					}
				}
				//ï¿½ï¿½ï¿½ï¿½ó£?¸ï¿½Î»
				if((1400<dog_jump_time)&&(1900>dog_jump_time))
				{
					*X = -50 + 50*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));

					*Y = 160 + 40*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
				}

		}
		else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		{
			dog_jump_time = 0;
			dog_jump_flag = 0;
		}
//////Â¥ï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½Ç°ï¿½ï¿½---------------------------
		if(rc_rc.s2 == 2)
		{
			dog_jump_flag_2 = 1;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
				if(500>dog_jump_time_2)
				{
					//ï¿½Â¶ï¿½
					*X = -50;
					//ï¿½Â¶×¸ß¶ï¿½
					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_2/1000)));
				}
				if((500<dog_jump_time_2)&&(700>dog_jump_time_2))
				{
					//ï¿½ï¿½ï¿½ï¿½
					*X = -200;
					*Y = 250;
				//dog_jump_flag = 0;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
				}
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?---------------------------------------------------------------------------
				if((700<dog_jump_time_2)&&(900>dog_jump_time_2))
				{
					//Ç°ï¿½ï¿½
					if((direction_mode == 1)||(direction_mode == 3))
					{
						*X = -200 + 210*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
						*Y = 210 - 50*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					}
					//ï¿½ï¿½ï¿½ï¿½
					if((direction_mode == 2)||(direction_mode == 4))
					{
						*X = -200 + 200*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
						*Y = 250 - 90*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
					}
				}
				//ï¿½ï¿½ï¿½ï¿½ó£?¸ï¿½Î»
				if((1400<dog_jump_time_2)&&(1900>dog_jump_time_2))
				{
					*X = 10 - 10*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));

					*Y = 160 + 40*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
				}

		}
		else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		{
			dog_jump_time_2 = 0;
			dog_jump_flag_2 = 0;
		}
//Ì¤ï¿½ï¿½--------------------------------------------------------------------------------------
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
////////						//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
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
////////					{//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
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
//////////-1-rc_rc.s1 == 1--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½---/////////////////////////////////////////////////////////
////////	if(rc_rc.s1 == 1)
////////	{
////////		flag = 0;
////////		//ï¿½ï¿½---------------------------
////////		if(rc_rc.s2 == 1)
////////		{
////////			dog_jump_flag = 1;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
////////				if(500>dog_jump_time)
////////				{
////////					//ï¿½Â¶ï¿½
////////					*X = -50;
////////					//ï¿½Â¶×¸ß¶ï¿½
////////					*Y = 200 - 40*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time/1000)));
////////				}
////////				if((500<dog_jump_time)&&(700>dog_jump_time))
////////				{
////////					//ï¿½ï¿½ï¿½ï¿½
////////					*X = -220;
////////					*Y = 350;
////////				//dog_jump_flag = 0;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
////////				}
//////////ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?---------------------------------------------------------------------------
////////				if((700<dog_jump_time)&&(900>dog_jump_time))
////////				{
////////					//Ç°ï¿½ï¿½
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = -150 + 170*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 350 - 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////					//ï¿½ï¿½ï¿½ï¿½
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = -150 + 210*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 350 - 100*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////				}
////////				//ï¿½ï¿½ï¿½ï¿½ó£?¸ï¿½Î»
////////				if((1400<dog_jump_time)&&(1900>dog_jump_time))
////////				{
////////					//Ç°ï¿½ï¿½
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = 20 - 40*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 160 ;//- 190*((float)(dog_jump_time-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time-700)/200)));
////////					}
////////					//ï¿½ï¿½ï¿½ï¿½
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = 60 ;//+ 50*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 250;//350 - 100*((float)(dog_jump_time-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time-1400)/500)));
////////					}

////////				}

////////		}
////////		else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
////////		{
////////			dog_jump_time = 0;
////////			dog_jump_flag = 0;
////////		}
//////////////Â¥ï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½Ç°ï¿½ï¿½---------------------------
////////		if(rc_rc.s2 == 2)
////////		{
////////			dog_jump_flag_2 = 1;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
////////				if(500>dog_jump_time_2)
////////				{
////////					//ï¿½Â¶ï¿½
////////					*X = -50;
////////					//ï¿½Â¶×¸ß¶ï¿½
////////					*Y = 200 - 50*(0.5f - 0.5f*cos(6.283f*((float)dog_jump_time_2/1000)));
////////				}
////////				if((500<dog_jump_time_2)&&(700>dog_jump_time_2))
////////				{
////////					//ï¿½ï¿½ï¿½ï¿½
////////					*X = -200;
////////					*Y = 250;
////////				//dog_jump_flag = 0;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
////////				}
//////////ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?---------------------------------------------------------------------------
////////				if((700<dog_jump_time_2)&&(900>dog_jump_time_2))
////////				{
////////					//Ç°ï¿½ï¿½
////////					if((direction_mode == 1)||(direction_mode == 3))
////////					{
////////						*X = -200 + 210*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 210 - 50*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////					}
////////					//ï¿½ï¿½ï¿½ï¿½
////////					if((direction_mode == 2)||(direction_mode == 4))
////////					{
////////						*X = -200 + 200*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////						//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
////////						*Y = 250 - 90*((float)(dog_jump_time_2-700)/200 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-700)/200)));
////////					}
////////				}
////////				//ï¿½ï¿½ï¿½ï¿½ó£?¸ï¿½Î»
////////				if((1400<dog_jump_time_2)&&(1900>dog_jump_time_2))
////////				{
////////					*X = 10 - 10*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));

////////					*Y = 160 + 40*((float)(dog_jump_time_2-1400)/500 - 0.1591f*sin(6.283f*((float)(dog_jump_time_2-1400)/500)));
////////				}

////////		}
////////		else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
////////		{
////////			dog_jump_time_2 = 0;
////////			dog_jump_flag_2 = 0;
////////		}
//////////Ì¤ï¿½ï¿½--------------------------------------------------------------------------------------
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
////////////////						//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
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
////////////////					{//ï¿½Ä½ï¿½ï¿½Åµï¿½Ê±ï¿½ï¿½
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
////////	else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
////////	{
////////		dog_jump_time = 0;
////////		dog_jump_flag = 0;
////////	}
////////////////////////////////////////////////////////////////////////////////////////
	//ï¿½ï¿½ï¿½ï¿½Ä£Ê½Ê±ï¿½ï¿½ï¿½ï¿½Î»
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
//		//ï¿½ï¿½---------------------------
//		if(rc_rc.s2 == 2)
//		{
//			backflip_flag = 1;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
//			if(500>backflip_time)
//			{
//				//ï¿½Â¶ï¿½
//				*X = -50;
//				//ï¿½Â¶×¸ß¶ï¿½
//				*Y = 250 - 100*(0.5f - 0.5f*cos(6.283f*((float)backflip_time/1000)));
//			}
//			if((500<backflip_time)&&(600>backflip_time))
//			{
//				//ï¿½ï¿½ï¿½ï¿½
//				*X = -100;
//				*Y = 300;
//			//dog_jump_flag = 0;//ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½Ö¾
//			}
//			if((600<backflip_time)&&(900>backflip_time))
//			{
//				*X = -100 + 150*((float)(backflip_time-600)/600 - 0.1591f*sin(6.283f*((float)(backflip_time-600)/600)));
//				//ï¿½ï¿½ï¿½ï¿½ï¿½Â¶×¸ß¶ï¿½
//				*Y = 300 - 120*(0.5f - 0.5f*cos(6.283f*((float)(backflip_time-600)/600)));
//			}
//			//ï¿½ï¿½ï¿½ï¿½ó£?¸ï¿½Î»
//			if((900<backflip_time)&&(1400>backflip_time))
//			{
//				*X = 50 - 50*((float)(backflip_time-900)/500 - 0.1591f*sin(6.283f*((float)(backflip_time-900)/500)));

//				*Y = 180 + 70*((float)(backflip_time-900)/500 - 0.1591f*sin(6.283f*((float)(backflip_time-900)/500)));
//			}
//			
//		}
//		else//--ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//		{
//			backflip_time = 0;
//			backflip_flag = 0;
//		}
}

void Dog_sport_mode(void)
{
	//ï¿½ï¿½Ä£Ê½
	if(rc_rc.s1 == 1)
	{ 
		//ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 1)
		{
		
		}
		//×¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 3)
		{
			
		}
	}
	//ï¿½ï¿½ï¿½ï¿½Ä£Ê½
	if(rc_rc.s1 == 1)
	{
		//ï¿½ì²½
		if(rc_rc.s2 == 1)
		{
		
		}
		//ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 3)
		{
			
		}
		//ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 2)
		{
			
		}
	}
	//×ªï¿½ï¿½Ä£Ê½
	if(rc_rc.s1 == 1)
	{
		//ï¿½ì²½
		if(rc_rc.s2 == 1)
		{
		
		}
		//ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 3)
		{
			
		}
		//ï¿½ï¿½ï¿½ï¿½
		if(rc_rc.s2 == 2)
		{
			
		}
	}
}



