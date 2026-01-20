//加入pid

#include "stm32f4xx.h"//寄存器映射头文件
#include "bsp_clkconfig.h"
#include "./usart/bsp_debug_usart.h"
#include "GO-M8010-6.h"
#include "bsp_usart_dma.h"
#include "pid.h"
#include "./tim/bsp_general_tim.h"
#include "./key/bsp_key.h" 
#include "bsp_ma_su.h" 
#include "bsp_rc.h"
#include "./can/bsp_can.h"
#include "bsp_imu.h"
int Key_flag =0;
///////////////////////////////////////////////////////////////////////////////
//以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_send motor_control_clear; //电机停止
// MDA 存储数据后，在中断中用于电机 ID 判断（临时缓冲）
MOTOR_recv motor_feedback_data;
// 全局数组：motor_control_data[1..Mo_Count], motor_feedback_data_arr[1..Mo_Count]
MOTOR_send motor_control_data[Mo_Count + 1];
MOTOR_recv motor_feedback_data_arr[Mo_Count + 1];
uint8_t Motor_Rxflag = 0;
//电机接收的数据
uint8_t Motor_Rx_date = 0;
//pid-1--------------------------------0.009f-0.0005------0.021-
const fp32 Motor1_speed_PID_data[3] = {0.009f,0.0006f,0.05f};//P,I,D值
//-----------------------------------49---35
const fp32 Motor1_rang_PID_data[3] = {100.0f,0.1f,0.0f};//P,I,D值

////////pid-9--------------------------------0.015f,0.0005f,0.03f-
//////const fp32 Motor9_speed_PID_data[3] = {0.009f,0.0006f,0.015f};//P,I,D值
////////-----------------------------------39.90f
//////const fp32 Motor9_rang_PID_data[3] = {24.90f,0.00f,0.0f};//P,I,D值
////////////////////////pid-2--------------------------------0.015f,0.0005f,0.03f-
//////////////////////const fp32 Motor2_speed_PID_data[3] = {0.01f,0.0006f,0.015f};//P,I,D值
////////////////////////-----------------------------------39.90f
//////////////////////const fp32 Motor2_rang_PID_data[3] = {50.90f,0.0f,50.0f};//P,I,D
//------------------------------------------------------------------------------
//pid-IMU-陀螺仪-X-方向---------------------------------------------------------
const fp32 imu_Z_PID_data[3] = {0.0f,0.0f,0.0f};//P,I,D值
//------------------------------------------------------------------------------
// PID 输出数组（1-based 索引）
float Motor_speed_PID_OUT[Mo_Count + 1] = {0};
float Motor_rang_PID_OUT[Mo_Count + 1] = {0};
//pid-IMU-陀螺仪-X-方向
float imu_Z_PID_OUT = 0;
////////////////////////////////////////////////////////////////////////////////

int tim_t = 1;////启动延时等待误差结束
int time_currently = 0;//步态当前时间

int time_turn = 0;//转向当前时间
extern uint8_t RC[18];
//-------------------------------
extern float Motor_set_Pos;
//------------------------------------
extern int Motor_send_ID;	//1--到--8号电机轮流发控制电机数据
extern int Motor_feedback_ID ;//反馈ID
extern int step ;
//----计时用------------------
extern  int Dog_Iinit;
////-----------------------------------------------
extern int printf_flag ;//可打印标志---2023-6-22
//-IMC-陀螺仪使用-----------------------------------------------------------------
extern float imu_Rx_data[4][4];
//--------------------------
//计算完得到的角度
float rang__2 = 0.0f;
float rang__3 = 0.0f;
float rang__4 = 0.0f;
float rang__5 = 0.0f;
float rang__6 = 0.0f;
float rang__7 = 0.0f;
float rang__8 = 0.0f;
float rang__9 = 0.0f;
//足端轨迹
  //--1号腿足端轨迹坐标
	float foot_track_x1 = 0.0f;
	float foot_track_y1 = 0.0f;
  //--2号腿足端轨迹坐标
	float foot_track_x2 = 0.0f;
	float foot_track_y2 = 0.0f;
  //--3号腿足端轨迹坐标
	float foot_track_x3 = 0.0f;
	float foot_track_y3 = 0.0f;
  //--4号腿足端轨迹坐标
	float foot_track_x4 = 0.0f;
	float foot_track_y4 = 0.0f;
	
	int nnn = 0;
////--CAN-----------------------------------------
uint32_t mail_box;//邮箱号
extern CanTxMsg TxMessage;//发送缓冲区

extern int printf_flag_A ;//步态当前时间
//---步幅转向标志--------------------------------------------------------
extern int step_turn_flag;
int step_rate = 0;
int main(void)
{
	/*初始化USART1*/
	RC_USART1_Config();
	/*初始化USART6*/
	Debug_USART_Config();
	/*初始化USART3*/
	MOTOR_CONTROL_USART_Config();
	/*USART6的DMA*/
	USART6_DMA_Config();
	/*USART1的DMA*/
	USART1_DMA_Config();
	/*USART1--RC--的DMA接收中断*/
	RC_DMA_NVIC_Configuration();
	/*USART6的DMA电机数据接收中断中断*/
	Motor_DMA_NVIC_Configuration();
	/*USART6--电机数据接收中断---不用了---有DMA中断了*/
	//	NVIC_Configuration();
	/* USART1 向 DMA发出RX请求 */
	USART_DMACmd(rc_DEBUG_USART , USART_DMAReq_Rx, ENABLE);
	/* USART6 向 DMA发出RX请求 */
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);
//	/*初始化can,在中断接收CAN数据包*/
//	CAN_Config();
	//系统时钟
  HSE_SetSysClock(12,336, 2, 7);

	  /*初始化按键*/
  Key_GPIO_Config();

	//PID初始化--与--限幅-----------------------------------
	PID_Init();
	//给电机模式等初始化
	Motor_Mode_Init();
	
	//电机位置偏移表运行时初始化
	motor_pos_init();
	
	//-----上电电机角度初始化---------------------------------------------
	//启动误差等待
	while(tim_t<100000)
	{
		tim_t++;
	}
	/* 初始化通用定时器2定时，1ms产生一次中断 */
  TIMx_Configuration();
//////////	//pid-IMU-陀螺仪-X-方向--2023-6-22
//////////	IMU_PID_Init();
       
	while(1)
	{	
		//--------------------------------------------------
		rc_rc_date();
		/*按下时电机停止，若按键按下则停止运行或开始运行 */  
		Motor_Key ();
		//pid-IMU-陀螺仪-X-方向--2023-6-22
		IMU_pid_count(0.0f);
		//--------------------2023-6-22-end
		if(step)
		{
			//--步频选择----------------------------------------------
			if((rc_rc.s1 == 3)&&(rc_rc.s2 == 1))
			{
				step_rate = 150;
			}
			if((rc_rc.s1 == 3)&&(rc_rc.s2 == 3))
			{
				step_rate = 200;
			}
			//--------------------------------------
			if((step == 2)||(step == 3))
			{
				//足端轨迹计算----得X---------得Y------抬脚时长---当前时间-模式
				foot_track(&foot_track_x1,&foot_track_y1,step_rate,time_currently,200,time_turn,2,1);
				//腿关节运动逆解---目标X--------目标Y---得出角度--2电机--3电机
				counter_motion(foot_track_x1,foot_track_y1, &rang__2, &rang__3);
			}
			if((step == 4)||(step == 5))
			{
				foot_track(&foot_track_x2,&foot_track_y2,step_rate,time_currently,200,time_turn,1,3);
				counter_motion(foot_track_x2,foot_track_y2, &rang__5, &rang__4);
			}
			if((step == 6)||(step == 7))
			{
				foot_track(&foot_track_x3,&foot_track_y3,step_rate,time_currently,200,time_turn,2,2);
				counter_motion(foot_track_x3,foot_track_y3, &rang__6, &rang__7);
			}
			if((step == 8)||(step == 9))
			{
				foot_track(&foot_track_x4,&foot_track_y4,step_rate,time_currently,200,time_turn,1,4);
				counter_motion(foot_track_x4,foot_track_y4, &rang__9, &rang__8);
			}
			Motor_pid_count(step,rc_rc.ch0*0.015f);
			//电机数据更新--------ID
			Motor_data_update(step);
			step = 0;
		}
		//2--到--9号电机轮流发控制电机数据
		Motor_date_send(&Motor_send_ID);
//		printf("2 == %f,3 == %f,4 == %f,5 == %f,6 == %f,7 == %f, 8 == %f,9 == %f,Z == %f\n", motor2_feedback_data.Pos,motor3_feedback_data.Pos\
//		, motor4_feedback_data.Pos, motor5_feedback_data.Pos, motor6_feedback_data.Pos, motor7_feedback_data.Pos, motor8_feedback_data.Pos\
//		, motor9_feedback_data.Pos,imu_Rx_data[2][3]);
//		printf("2 == %f,3 == %f,4 == %f,5 == %f,6 == %f,7 == %f, 8 == %f,9 == %f,Z == %f\n", Motor2_speed_PID_OUT,Motor3_speed_PID_OUT\
//		, Motor4_speed_PID_OUT, Motor5_speed_PID_OUT,Motor6_speed_PID_OUT, Motor7_speed_PID_OUT, Motor8_speed_PID_OUT\
//		, Motor9_speed_PID_OUT,imu_Rx_data[2][3]);
//		printf("%f,,,,%f\n", imu_Rx_data[2][3],imu_Rx_data[3][3]);


//				//设置发送数据
//		CAN_SetMsg(&TxMessage);
//		//把报文发送到邮箱，邮箱自动发送
//		mail_box = CAN_Transmit(CAN_x, &TxMessage);
//		//等数据发送到收发邮箱
//		if(mail_box != CAN_TxStatus_NoMailBox)
//		{
//			printf("\r\nCAN接收到数据：\r\n");	
//			//等待发送完毕
//			while(CAN_TransmitStatus(CAN_x,mail_box) != CAN_TxStatus_Ok);
//			printf("\r\nCAN接收到数据：\r\n");	
//		}	

//		if(printf_flag)
//		{
//			printf_flag = 0;
//			printf("%f\n",motor6_feedback_data.Pos);
//		}
		while(nnn<5000)
		{
			nnn++;
		}
		nnn = 0;
	}
}



