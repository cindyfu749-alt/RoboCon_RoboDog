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
#include "control_task.h"
#include <string.h>
//以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_send motor_control_clear; //电机停止
// MDA 存储数据后，在中断中用于电机 ID 判断（临时缓冲）
MOTOR_recv motor_feedback_date_buffer;

// 全局数组：motor_control_data[1..Mo_Count], motor_feedback_data_arr[1..Mo_Count]
MOTOR_send motor_control_data[Mo_Count];
MOTOR_recv motor_feedback_data[Mo_Count];
uint8_t Motor_Rxflag = 0;
//电机接收的数据
uint8_t Motor_Rx_date = 0;
////--CAN-----------------------------------------
uint32_t mail_box;//邮箱号
int main(void)
{
	//-----上电电机角度初始化---------------------------------------------
	control_task_init();
	//启动误差等待
	dog_state_t* state = control_task_get_state();
	while(state->tim_t<100000)
	{
		state->tim_t++;
	}
	/* 初始化通用定时器2定时，1ms产生一次中断 */
  TIMx_Configuration();
//////////	//pid-IMU-陀螺仪-X-方向--2023-6-22
//////////	IMU_PID_Init();
	while(1)
	{	
		//控制任务执行函数
	    control_task_execute();
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
	}
}



