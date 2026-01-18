#include "bsp_ma_su.h" 
#include "pid.h"
#include "GO-M8010-6.h"
#include "./usart/bsp_debug_usart.h"
#include "./key/bsp_key.h" 
#include "bsp_rc.h"
#include "math.h"
//��ȫ�ֱ�������������ƽṹ��͵�����ݽṹ�壬�����ڹ���ʱͨ��debug�鿴����ֵ
extern	MOTOR_send motor_control_clear; //���ֹͣ
extern	MOTOR_send motor1_control_data;   //���1����
extern	MOTOR_recv motor1_feedback_data;	 //���1����
extern	MOTOR_send motor2_control_data;   //���1����
extern	MOTOR_recv motor2_feedback_data;	 //���1����
extern	MOTOR_send motor3_control_data;   //���1����
extern	MOTOR_recv motor3_feedback_data;	 //���1����
extern	MOTOR_send motor4_control_data;   //���1����
extern	MOTOR_recv motor4_feedback_data;	 //���1����
extern	MOTOR_send motor5_control_data;   //���1����
extern	MOTOR_recv motor5_feedback_data;	 //���1����
extern	MOTOR_send motor6_control_data;   //���1����
extern	MOTOR_recv motor6_feedback_data;	 //���1����
extern	MOTOR_send motor7_control_data;   //���1����
extern	MOTOR_recv motor7_feedback_data;	 //���1����
extern	MOTOR_send motor8_control_data;   //���1����
extern	MOTOR_recv motor8_feedback_data;	 //���1����
extern	MOTOR_send motor9_control_data;   //���1����
extern	MOTOR_recv motor9_feedback_data;	 //���1����
extern float Motor1_speed_PID_OUT;
extern float Motor1_rang_PID_OUT;
extern float Motor2_speed_PID_OUT;
extern float Motor2_rang_PID_OUT;
extern float Motor3_speed_PID_OUT;
extern float Motor3_rang_PID_OUT;
extern float Motor4_speed_PID_OUT;
extern float Motor4_rang_PID_OUT;
extern float Motor5_speed_PID_OUT;
extern float Motor5_rang_PID_OUT;
extern float Motor6_speed_PID_OUT;
extern float Motor6_rang_PID_OUT;
extern float Motor7_speed_PID_OUT;
extern float Motor7_rang_PID_OUT;
extern float Motor8_speed_PID_OUT;
extern float Motor8_rang_PID_OUT;
extern float Motor9_speed_PID_OUT;
extern float Motor9_rang_PID_OUT;
extern const fp32 Motor1_speed_PID_data[3];
extern const fp32 Motor1_rang_PID_data[3];
extern const fp32 Motor2_speed_PID_data[3];
extern const fp32 Motor2_rang_PID_data[3];
extern const fp32 Motor3_speed_PID_data[3];
extern const fp32 Motor3_rang_PID_data[3];
extern const fp32 Motor4_speed_PID_data[3];
extern const fp32 Motor4_rang_PID_data[3];
extern const fp32 Motor5_speed_PID_data[3];
extern const fp32 Motor5_rang_PID_data[3];
extern const fp32 Motor6_speed_PID_data[3];
extern const fp32 Motor6_rang_PID_data[3];
extern const fp32 Motor7_speed_PID_data[3];
extern const fp32 Motor7_rang_PID_data[3];
extern const fp32 Motor8_speed_PID_data[3];
extern const fp32 Motor8_rang_PID_data[3];
extern const fp32 Motor9_speed_PID_data[3];
extern const fp32 Motor9_rang_PID_data[3];
extern float rang__2;
extern float rang__3;
extern float rang__4;
extern float rang__5;
extern float rang__6;
extern float rang__7;
extern float rang__8;
extern float rang__9;
//��˹켣
	//--1������˹켣����
	extern float foot_track_x1 ;
	extern float foot_track_y1 ;
	//--2������˹켣����
	extern float foot_track_x2 ;
	extern float foot_track_y2 ;
	//--3������˹켣����
	extern float foot_track_x3 ;
	extern float foot_track_y3 ;
	//--4������˹켣����
	extern float foot_track_x4 ;
	extern float foot_track_y4 ;
	extern int time_currently;//��̬��ǰʱ��
	extern int time_turn ;//ת��ǰʱ��
float ttl = 0.0f;
int step = 1;//�ж��и�ֵ
//-----Motor_set_Pos-----------------
float Motor_set_Pos = 0.0f;
//---------------------------------------
int Motor_send_ID = 1;//1--��--8�ŵ�����������Ƶ������
//--------------------------------
//����ʼ��վ������ʱ��
extern int Dog_Iinit ;
extern int Dog_flag ;
//--------------------------------
//��̬��λ---��־----------------
 int dog_rest_flag = 0;
//��------------------------------------------------------------------------------------
		//����ʼ��ʱ��־
		extern int dog_jump_flag;
		//--��--ʱ��------------
		extern int dog_jump_time;
		//--��--ʱ��------------
		extern int dog_jump_time_2 ;
		//����ʼ��ʱ��־
		extern int dog_jump_flag_2 ;
		//�շ���ʼ��ʱ��־
		extern int backflip_flag ;
		//�շ���ʱ
		extern int backflip_time ;
//-IMC-������ʹ��-----------------------------------------------------------------
extern float imu_Rx_data[4][4];
//---����ת���־--------------------------------------------------------
extern int step_turn_flag;
float step_turn_k = 0.0004f;

//--Ħ���ȸ�--------------------------
	int leg_high = 200;
	int leg_high_tai_tui = 40;
//----------------------------------
void PID_Init(void)
{
	//-1111111111111111111111------------------------------- pid������
	//speed
	PID_init(&Motor1_speed_PID, PID_POSITION,Motor1_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor1_rang_PID, PID_POSITION,Motor1_rang_PID_data,10000,2);
	//-2222222222222222222222---------------------------------
	//speed
	PID_init(&Motor2_speed_PID, PID_POSITION,Motor2_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor2_rang_PID, PID_POSITION,Motor2_rang_PID_data,10000,2);
	//-3333333333333333333333---------------------------------
	//speed
	PID_init(&Motor3_speed_PID, PID_POSITION,Motor3_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor3_rang_PID, PID_POSITION,Motor3_rang_PID_data,10000,2);
	//-4444444444444444444444------------------------------- pid������
	//speed
	PID_init(&Motor4_speed_PID, PID_POSITION,Motor4_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor4_rang_PID, PID_POSITION,Motor4_rang_PID_data,10000,2);
	//-5555555555555555555555---------------------------------
	//speed
	PID_init(&Motor5_speed_PID, PID_POSITION,Motor5_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor5_rang_PID, PID_POSITION,Motor5_rang_PID_data,10000,2);
	//-6666666666666666666666666---------------------------------
	//speed
	PID_init(&Motor6_speed_PID, PID_POSITION,Motor6_speed_PID_data, 3.50, 0.2);
	//rang
	PID_init(&Motor6_rang_PID, PID_POSITION,Motor6_rang_PID_data,10000,2);
	//-77777777777777777777777------------------------------- pid������
	//speed
	PID_init(&Motor7_speed_PID, PID_POSITION,Motor7_speed_PID_data,  3.50,0.2);
	//rang
	PID_init(&Motor7_rang_PID, PID_POSITION,Motor7_rang_PID_data,10000,2);
	//-88888888888888888888888---------------------------------
	//speed
	PID_init(&Motor8_speed_PID, PID_POSITION,Motor8_speed_PID_data, 3.50,0.2);
	//rang
	PID_init(&Motor8_rang_PID, PID_POSITION,Motor8_rang_PID_data,10000,2);
	//-99999999999999999999999---------------------------------
	//speed
	PID_init(&Motor9_speed_PID, PID_POSITION,Motor9_speed_PID_data,  3.50, 0.2);
	//rang
	PID_init(&Motor9_rang_PID, PID_POSITION,Motor9_rang_PID_data,10000,2);
}
//�����ģʽ�ȳ�ʼ��
int Motor_Mode_Init(void)
{
	//----���ֹͣ�ṹ��----------------------------------------------------
	motor_control_clear.id=15; 			
	motor_control_clear.mode=1;
	motor_control_clear.T=0;
	motor_control_clear.W=0.0;
	motor_control_clear.Pos=0.0;
	motor_control_clear.K_P=0.0;
	motor_control_clear.K_W=0.0;
	//��ͣ������д�뷢�ͽṹ�����ݸ�ֵ
	modify_data(&motor_control_clear);
	
	//-------------------------------------------------------------
		motor1_control_data.id=1;
		motor1_control_data.mode=1;
		motor1_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor1_control_data.W=0.0;
		motor1_control_data.Pos=0.0;
		motor1_control_data.K_P=0.0;
		motor1_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor1_control_data);
		
	//------------------------------------
		motor2_control_data.id=2;
		motor2_control_data.mode=1;
		motor2_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor2_control_data.W=0.0;
		motor2_control_data.Pos=0.0;
		motor2_control_data.K_P=0.0;
		motor2_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor2_control_data);
	//------------------------------------
	
		motor3_control_data.id=3;
		motor3_control_data.mode=1;
		motor3_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor3_control_data.W=0.0;
		motor3_control_data.Pos=0.0;
		motor3_control_data.K_P=0.0;
		motor3_control_data.K_W=0.0;
		//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor3_control_data);
		//------------------------------------
		
		//-------------------------------------------------------------
		motor4_control_data.id=4;
		motor4_control_data.mode=1;
		motor4_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor4_control_data.W=0.0;
		motor4_control_data.Pos=0.0;
		motor4_control_data.K_P=0.0;
		motor4_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor4_control_data);
		
	//------------------------------------
		motor5_control_data.id=5;
		motor5_control_data.mode=1;
		motor5_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor5_control_data.W=0.0;
		motor5_control_data.Pos=0.0;
		motor5_control_data.K_P=0.0;
		motor5_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor5_control_data);
	//------------------------------------
	
		motor6_control_data.id=6;
		motor6_control_data.mode=1;
		motor6_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor6_control_data.W=0.0;
		motor6_control_data.Pos=0.0;
		motor6_control_data.K_P=0.0;
		motor6_control_data.K_W=0.0;
		//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor6_control_data);
		//------------------------------------
		
			//-------------------------------------------------------------
		motor7_control_data.id=7;
		motor7_control_data.mode=1;
		motor7_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor7_control_data.W=0.0;
		motor7_control_data.Pos=0.0;
		motor7_control_data.K_P=0.0;
		motor7_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor7_control_data);
		
	//------------------------------------
		motor8_control_data.id=8;
		motor8_control_data.mode=1;
		motor8_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor8_control_data.W=0.0;
		motor8_control_data.Pos=0.0;
		motor8_control_data.K_P=0.0;
		motor8_control_data.K_W=0.0;
	//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor8_control_data);
	//------------------------------------
	
		motor9_control_data.id=9;
		motor9_control_data.mode=1;
		motor9_control_data.T=0;//����ʱ��ֻ��ת�ظ���
		motor9_control_data.W=0.0;
		motor9_control_data.Pos=0.0;
		motor9_control_data.K_P=0.0;
		motor9_control_data.K_W=0.0;
		//�ѵ������д�뷢�ͽṹ�����ݸ�ֵ
		modify_data(&motor9_control_data);
		//------------------------------------
	return 0;
}

////���pid����
int Motor_pid_count(int Motor_feedback_ID,float set_pos)
{
	
		switch(Motor_feedback_ID)
	{
//		case 1 :  
////					//����ٶ�pid-1-����-----------------------------����--------Ŀ��ֵ---
////					//rang
////					Motor1_rang_PID_OUT = PID_calc(&Motor1_rang_PID, motor1_feedback_data.Pos,rang__2);
////					//speed
////					Motor1_speed_PID_OUT = PID_calc(&Motor1_speed_PID, motor1_feedback_data.W ,Motor1_rang_PID_OUT);
////				
//					break;
		case 2 :  
					//����ٶ�pid-1-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor2_rang_PID_OUT = PID_calc(&Motor2_rang_PID, motor2_feedback_data.Pos + 11.148f,rang__2);
					//speed
					Motor2_speed_PID_OUT = PID_calc(&Motor2_speed_PID, motor2_feedback_data.W ,Motor2_rang_PID_OUT);
					break;
		case 3 :  
					//����ٶ�pid-3-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor3_rang_PID_OUT = PID_calc(&Motor3_rang_PID,motor3_feedback_data.Pos - 9.080f,rang__3);
					//speed
					Motor3_speed_PID_OUT = PID_calc(&Motor3_speed_PID, motor3_feedback_data.W ,Motor3_rang_PID_OUT);
					break;
		case 4 :  
					//����ٶ�pid-4-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor4_rang_PID_OUT = PID_calc(&Motor4_rang_PID,motor4_feedback_data.Pos + 0.501f,-rang__4);
					//speed
					Motor4_speed_PID_OUT = PID_calc(&Motor4_speed_PID, motor4_feedback_data.W ,Motor4_rang_PID_OUT);
						break;		
		case 5 :  
					//����ٶ�pid-5-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor5_rang_PID_OUT = PID_calc(&Motor5_rang_PID,motor5_feedback_data.Pos - 18.194f,-rang__5);
					//speed
					Motor5_speed_PID_OUT = PID_calc(&Motor5_speed_PID, motor5_feedback_data.W ,Motor5_rang_PID_OUT);
						break;
		case 6 :  
					//����ٶ�pid-6-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor6_rang_PID_OUT = PID_calc(&Motor6_rang_PID,motor6_feedback_data.Pos - 12.201f ,-rang__6);
					//speed
					Motor6_speed_PID_OUT = PID_calc(&Motor6_speed_PID, motor6_feedback_data.W ,Motor6_rang_PID_OUT);
						break;
		case 7 :  
					//����ٶ�pid-7-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor7_rang_PID_OUT = PID_calc(&Motor7_rang_PID,motor7_feedback_data.Pos + 4.448f,-rang__7);
					//speed
					Motor7_speed_PID_OUT = PID_calc(&Motor7_speed_PID, motor7_feedback_data.W ,Motor7_rang_PID_OUT);
						break;
		case 8 :  
					//����ٶ�pid-8-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor8_rang_PID_OUT = PID_calc(&Motor8_rang_PID,motor8_feedback_data.Pos - 10.007f,rang__8);
					//speed
					Motor8_speed_PID_OUT = PID_calc(&Motor8_speed_PID, motor8_feedback_data.W ,Motor8_rang_PID_OUT);
					break;
		case 9 :  
					//����ٶ�pid-9-����-----------------------------����--------Ŀ��ֵ---
					//rang
					Motor9_rang_PID_OUT = PID_calc(&Motor9_rang_PID,motor9_feedback_data.Pos + 6.888f,rang__9);
					//speed
					Motor9_speed_PID_OUT = PID_calc(&Motor9_speed_PID, motor9_feedback_data.W ,Motor9_rang_PID_OUT);
					break;
	}


 return 0;
}

//������ݸ���
int Motor_data_update(int Motor_feedback_ID)
{
	
//	//��--����
//	if((dog_jump_time>500)&&(dog_jump_time<700)&&(rc_rc.s2 == 1))
//	{
//		Motor2_speed_PID_OUT = Motor2_speed_PID_OUT*1.50f;
//		Motor3_speed_PID_OUT = Motor3_speed_PID_OUT*1.50f;
//		Motor4_speed_PID_OUT = Motor4_speed_PID_OUT*1.50f;
//		Motor5_speed_PID_OUT = Motor5_speed_PID_OUT*1.50f;
//		Motor6_speed_PID_OUT = Motor6_speed_PID_OUT*1.50f;
//		Motor7_speed_PID_OUT = Motor7_speed_PID_OUT*1.50f;
//		Motor8_speed_PID_OUT = Motor8_speed_PID_OUT*1.50f;
//		Motor9_speed_PID_OUT = Motor9_speed_PID_OUT*1.50f;
//	}
	//���������ָ��ṹ�帳ֵ---------------------------------------------------------------------------
		switch(Motor_feedback_ID)
	{
		case 1 :  
						//���1��ֵ--ת�����--------------------
//						motor1_control_data.T=0;
						//������д�뷢�ͽṹ�����ݸ�ֵ
//						modify_data(&motor1_control_data);
						//�ж�CRC������д����սṹ�����ݸ�ֵ
						//extract_data(&motor1_feedback_data);
						break;
		case 2 :  
						motor2_control_data.T = Motor2_speed_PID_OUT;
						modify_data(&motor2_control_data);
						//extract_data(&motor2_feedback_data);
						break;
		case 3 :  
						motor3_control_data.T = Motor3_speed_PID_OUT;
						modify_data(&motor3_control_data);
						break;
		case 4 :  
						motor4_control_data.T = Motor4_speed_PID_OUT;
						modify_data(&motor4_control_data);
						break;	
		case 5 : 
						motor5_control_data.T = Motor5_speed_PID_OUT;
						modify_data(&motor5_control_data);
						break;  
		case 6 :  
						motor6_control_data.T = Motor6_speed_PID_OUT;
						modify_data(&motor6_control_data);
						break;
		case 7 :  
						motor7_control_data.T = Motor7_speed_PID_OUT;
						modify_data(&motor7_control_data);
						break;
		case 8 :  
						motor8_control_data.T= Motor8_speed_PID_OUT;
						modify_data(&motor8_control_data);
		break;
		case 9 :  
						motor9_control_data.T= Motor9_speed_PID_OUT;
						modify_data(&motor9_control_data);
		break;
	}

	return 0;
}
//1--��--8�ŵ�����������Ƶ�����ݵ�����ݷ���
int Motor_date_send(int*Motor_send_ID)
{
	switch(*Motor_send_ID)
	{
		case 1 :
//						Usart_SendStr_length( USART6, ((uint8_t *)(&motor1_control_data)),motor1_control_data.hex_len );
						break;
		case 2 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor2_control_data)),motor2_control_data.hex_len );
						break;
		case 3 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor3_control_data)),motor3_control_data.hex_len );
						break;
		case 4 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor4_control_data)),motor4_control_data.hex_len );
						break;
		case 5 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor5_control_data)),motor5_control_data.hex_len );
						break;
		case 6 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor6_control_data)),motor6_control_data.hex_len );
						break;
		case 7 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor7_control_data)),motor7_control_data.hex_len );
						break;
		case 8 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor8_control_data)),motor8_control_data.hex_len );
						break; 
		case 9 :
						Usart_SendStr_length( USART6, ((uint8_t *)(&motor9_control_data)),motor9_control_data.hex_len );
						break; 
	}
	//1--��--8�ŵ�����������Ƶ������
	if(*Motor_send_ID == 9) (*Motor_send_ID) = 1;
	*Motor_send_ID = (*Motor_send_ID) +1;
	return 0;
}

void Motor_Key (void)
{
	/* ��ѯ����״̬��������������ֹͣ���л�ʼ���� */    
	if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
	{
    //step = 0;
		//pid����
		PID_clear(&Motor1_speed_PID);
		PID_clear(&Motor1_rang_PID);
		PID_clear(&Motor2_speed_PID);
		PID_clear(&Motor2_rang_PID);
		PID_clear(&Motor3_speed_PID);
		PID_clear(&Motor3_rang_PID);
		PID_clear(&Motor4_speed_PID);
		PID_clear(&Motor4_rang_PID);
		PID_clear(&Motor5_speed_PID);
		PID_clear(&Motor5_rang_PID);
		PID_clear(&Motor6_speed_PID);
		PID_clear(&Motor6_rang_PID);
		PID_clear(&Motor7_speed_PID);
		PID_clear(&Motor7_rang_PID);
		PID_clear(&Motor8_speed_PID);
		PID_clear(&Motor8_rang_PID);
		PID_clear(&Motor9_speed_PID);
		PID_clear(&Motor9_rang_PID);
		while(1)
		{
			Usart_SendStr_length( USART6, ((uint8_t *)(&motor_control_clear)),motor_control_clear.hex_len );
		}
	} 
}
//�ȹؽ��˶����
void counter_motion(float X,float Y,float *range1, float *range2)
{
	float L = 0.0f;//���ȳ�
//	float L1 = 0.0f;//���ȳ�
//	float L2 = 0.0f;//���ȳ�
	float range_Leg = 0.0f;//�Ƚ�
	float range_separate = 0.0f;//�����
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
//��˹켣--------------------------̧��ʱ��----��ǰʱ��------ת�������--ת��ǰʱ��-----������֧��-ģʽ------
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
		//���ֱ�����켣
		*X = 160 - 160*((float)Dog_Iinit/2000);
		*Y = 100 + 100*((float)Dog_Iinit/2000);
	}
	else
	{
	//-1-//�����Ⱥ��------��֧��-------------------------------------------------------------------------------
	if(move_mode == 1)
	{
		//��ǰ��
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//̧��ʱ��-----̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//���ֱ�����켣
				*X = 70 - 140*((float)time_currently/Tm) ;
				*Y = leg_high;
			}
			else//����̧��ʱ������֧��ֱ�����
			{
				//���Ľ��ŵ�ʱ��
				//��˰�����ǰ�켣
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   -70  +   140*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//��˸�     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));
////				//���Ľ��ŵ�ʱ��
////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
////				{
////					*X = -70;
////					*Y = 200;
////				}
////				else
////				{
////					//��˰�����ǰ�켣
////					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
////					*X =   -70  +   140*((float)time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.5f)));
////					//��˸�     ̧�Ÿ߶�
////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.5f)));
////				}
			}
//---����ת��-----------------------------------------------------------
			if((rc_rc.ch0 > 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��2��-4-5-----------����ת���־--------------------------------------------------------
				if(((direction_mode ==3))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 0.91f - step_turn_k*rc_rc.ch0);
				}
				//-------��4��-8-9---------����ת���־--------------------------------------------------------
				if(((direction_mode ==4))&&step_turn_flag)
				{
					*X = (*X)*1.09f ;
				}
			}
			if((rc_rc.ch0 < 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��-4-5-----------����ת���־--------------------------------------------------------
				if(((direction_mode ==3))&&step_turn_flag)
				{
					*X = *X ;
				}
				//-------��-8-9-----------����ת���־--------------------------------------------------------
				if(((direction_mode ==4))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*( - rc_rc.ch0));
				}
			}
//��̬��λ----------------
			dog_rest_flag = 0;
		}
		//�����----------------------------------------------------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//̧��ʱ��̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//���ֱ�����켣
				*X = -70 + 140*((float)time_currently/Tm);
				*Y = leg_high;
			}
			else//����̧��ʱ������֧��ֱ�����
			{
				//���Ľ��ŵ�ʱ��
				//��˰�����ǰ�켣
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   70 - 140*((float)time_currently/Tm-1.0f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.0f)));
				//��˸�     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.0f)));

//////////				//�Ľ��ŵ�ʱ��
//////////				if(((float)time_currently/Tm > 1)&&((float)time_currently/Tm <1.5f))
//////////				{
//////////					*X = 70;
//////////					*Y = 200;
//////////				}
//////////				else
//////////				{
//////////					//��˰�����ǰ�켣
//////////					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
//////////					*X =   70 - 140*((float)time_currently/Tm-1.5f - 0.1591f*sin(6.283f*((float)time_currently/Tm - 1.5f)));
//////////					//��˸�     ̧�Ÿ߶�
//////////					*Y =   200  -   50*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm - 1.5f)));
//////////				}
			}
			//��̬��λ----------------
			dog_rest_flag = 0;
		}
	}
//-1-//�����Ⱥ��--��̧��-------------------------------------------------------------------------------
	if(move_mode == 2)
	{
		//��ǰ��
		if((rc_rc.s1 == 3)&&(rc_rc.ch3>330))
		{
			flag = 0;
			//̧��ʱ��-----̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//��˰�����ǰ�켣
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   -70 + 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//��˸�     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//����̧��ʱ������֧��ֱ�����
			{
				//���Ľ��ŵ�ʱ��
				//���ֱ�����켣
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
////////					//���ֱ�����켣
////////					*X = 70 - 140*((float)time_currently/Tm -1.5f);
////////					*Y = 200;
////////				}
			}
//---����ת��-----------------------------------------------------------
			//��ת--
			if((rc_rc.ch0 > 0)||(step_turn_flag))
			{
				step_turn_flag = 1;
				//-------��1��-2-3-----------����ת���־--------------------------------------------------------
				if(((direction_mode ==1))&&step_turn_flag)
				{
					*X = (*X)*1.09f  ;
				}
				//-------��2��-6-7----------����ת���־--------------------------------------------------------
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
				//-------��-2-3-----------����ת���־--------------------------------------------------------
				if(((direction_mode ==1))&&step_turn_flag)
				{
					//����ת��ϵ��step_turn_k*rc_rc.ch0  0---0.5
					*X = (*X)*( 1.0f - step_turn_k*(-rc_rc.ch0));
				}
				//-------��-4-5--------����ת���־--------------------------------------------------------
				if(((direction_mode ==2))&&step_turn_flag)
				{
					* X = *X;
				}
			}
//��̬��λ----------------
			dog_rest_flag = 0;
		}
		//�����
		if((rc_rc.s1 == 3)&&(rc_rc.ch3 < -330))
		{
			flag = 0;
			//̧��ʱ��̧��
			if(((time_currently == 0)||(time_currently > 0))&&((time_currently<Tm)||(Tm == time_currently)))
			{
				//��˰�����ǰ�켣
				//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
				*X =   70 - 140*((float)time_currently/Tm - 0.1591f*sin(6.283f*((float)time_currently/Tm)));
				//��˸�     ̧�Ÿ߶�
				*Y =   200  -   leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_currently/Tm )));
			}
			else//����̧��ʱ������֧��ֱ�����
			{
				//���Ľ��ŵ�ʱ��
				//���ֱ�����켣
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
////////					//���ֱ�����켣
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
					//���ֱ�����켣
					*X = 70-140*((float)time_turn/turn_Tm);
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�     ̧�Ÿ߶�
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
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70-140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ�����켣
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
					//���ֱ�����켣
					*X = -70 + 140*((float)time_turn/turn_Tm);
					*Y = leg_high;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/turn_Tm-1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�     
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
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ�����켣
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
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ�����켣
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
					//���ֱ�����켣
					*X = 70-140*((float)time_turn/turn_Tm  );
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm - 1)));
					//��˸�     ̧�Ÿ߶�
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
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = -70 + 140*((float)time_turn/turn_Tm - 0.1591f*sin(6.283f*((float)time_turn/turn_Tm )));
					//��˸�     ̧�Ÿ߶�
					*Y = 200 - leg_high_tai_tui*(0.5f - 0.5f*cos(6.283f*((float)time_turn/turn_Tm )));
				}
				else//̧�Ÿ�λ�׶�
				{
					//���ֱ�����켣
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
					//���ֱ�����켣
					*X = -70 + 140*((float)time_turn/Tm  );
					*Y = 200;
				}
				else//̧�Ÿ�λ�׶�
				{
					//��˰�����ǰ�켣
					//��ʼ��-----����------------��ǰʱ��--̧��ʱ��
					*X = 70 - 140*((float)time_turn/Tm -1 - 0.1591f*sin(6.283f*((float)time_turn/Tm - 1)));
					//��˸�     ̧�Ÿ߶�
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
//�������---------------------------------------------------------------------------
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
				//����󣬸�λ
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
//�������---------------------------------------------------------------------------
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
				//����󣬸�λ
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
//////////�������---------------------------------------------------------------------------
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
////////				//����󣬸�λ
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
//////////�������---------------------------------------------------------------------------
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
////////				//����󣬸�λ
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
//			//����󣬸�λ
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



