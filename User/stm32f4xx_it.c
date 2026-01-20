/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "./usart/bsp_debug_usart.h"
#include "./tim/bsp_general_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "pid.h"
#include "GO-M8010-6.h"
#include "bsp_ma_su.h" 
#include "bsp_usart_dma.h"
#include "bsp_rc.h"
#include "./can/bsp_can.h"
#include "bsp_imu.h"

/* Legacy per-motor feedback data symbols (for compatibility; backed by motor_feedback_data[id]) */
MOTOR_recv motor1_feedback_data;
MOTOR_recv motor2_feedback_data;
MOTOR_recv motor3_feedback_data;
MOTOR_recv motor4_feedback_data;
MOTOR_recv motor5_feedback_data;
MOTOR_recv motor6_feedback_data;
MOTOR_recv motor7_feedback_data;
MOTOR_recv motor8_feedback_data;
MOTOR_recv motor9_feedback_data;

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

//uint8_t USART_receiveData  = 0;
//void RS232_USART_IRQHandler(void)
//{
//	//if(USART_GetITStatus( DEBUG_USART, USART_IT_RXNE ) != RESET)
//	//{		
//	  
//		//USART_receiveData = USART_ReceiveData( DEBUG_USART );
//		//USART_SendData(DEBUG_USART,USART_receiveData);
//	//} 
//	 
//}	
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
extern	MOTOR_recv motor_feedback_data;
extern uint8_t Motor_Rxflag;
extern uint8_t Motor_Rx_date;
void RS485_USART_IRQHandler(void)
{
//	if(USART_GetITStatus( DEBUG_USART, USART_IT_RXNE ) != RESET)
//	{		
//	  Motor_Rxflag = 1;		
//		
//	} 
	 
}	

//------------------------------------------------------------------
int pp = 0;
int Motor_feedback_ID = 1;
int tim_flag_aa = 0;
int RC_RX_flag = 0;
//狗初始化站立起来时间
int Dog_Iinit = 0;
int Dog_flag = 1;


//--CAN使用------------------------------------------------------------------------------
 __IO uint32_t flag ;		 //用于标志是否接收到数据，在中断函数中赋值
extern CanRxMsg RxMessage;		//接收缓冲区
//--------------------------------------------------------------------------------
//-IMC-陀螺仪使用-----------------------------------------------------------------
extern float imu_Rx_data[4][4];
	//位移使用
	float imu_x = 0.0f;
	float imu_x1 = 0.0f;
	float imu_x2 = 0.0f;
	float imu_t1 = 0;
	int   imu_t2 = 0;
	float imu_v1 = 0.0f;
	float imu_v2 = 0.0f;
//--------------------------------------------------------------------------------
int printf_flag = 0;//可打印标志
//-步态-复位标志---
extern int dog_rest_flag;
//跳----------------------------------------------------------------------
	//--跳--时间------------
	int dog_jump_time = 0;
	//跳开始计时标志
	int dog_jump_flag = 0;
	//--跳--时间------------
	int dog_jump_time_2 = 0;
	//跳开始计时标志
	int dog_jump_flag_2 = 0;
	//空翻开始计时标志
	int backflip_flag = 0;
	//空翻计时
	int backflip_time = 0;
//------------------------------------------------------------------------
//---步幅转向标志--------------------------------------------------------
int step_turn_flag = 1;
//-----------------------------------------------------------------------
//---步频--------------------------------------------------------------------
		//跷跷板步速
		int step_speed_high = 300;
		//双木桥步速
		int step_speed_middle = 400;
		int time_speed = 300;
//----1-毫秒中断
void GENERAL_TIM_IRQHandler (void)
{
	//--步频选择----------------------------------------------
		if((rc_rc.s1 == 3)&&(rc_rc.s2 == 3))
		{
			time_speed = step_speed_middle;
		}
		if((rc_rc.s1 == 3)&&(rc_rc.s2 == 1))
		{
			if(time_currently>299)time_currently = 0;
			time_speed = step_speed_high;
		}

	//行走步态时间-------------
	time_currently++;
	if(time_currently == time_speed)
	{
		time_currently = 0;
		Dog_flag = 0;
		printf_flag = 1;
		if(dog_rest_flag <1)dog_rest_flag++;
		//---步幅转向标志--步幅转向周期------------------------------------------------------
    step_turn_flag = 0;
	}
	time_turn++;//转向时间
	if(time_turn == 400)
	{
		time_turn = 0;
	}
	
	if(dog_jump_flag)dog_jump_time++;//跳--时间
	if(dog_jump_flag_2)dog_jump_time_2++;//跳--时间
//	if(dog_jump_time == 1000)
//	{
//		dog_jump_time = 0;
//	}
	//狗初始化站立起来时间
	if(Dog_Iinit <2001)
	{
		Dog_Iinit++;
	}
  //清除中断标志
	TIM_ClearITPendingBit(GENERAL_TIM , TIM_IT_Update);  		 
}

/*********DMA中断处理**********/
//电机数据的ID辨别
void Motor_DMA_IRQHandler (void)
{
		
	//判断CRC把数据写入接收结构体数据赋值
	extract_data(&motor_feedback_data);
	Motor_feedback_ID = motor_feedback_data.motor_id;
	

//	printf("%d\n",motor_feedback_data.motor_id);
//	printf("%d\n",motor_feedback_data.motor_recv_data.mode.id);
	switch(Motor_feedback_ID)
	 {
		case 1 :  
			step = 1;
			motor1_feedback_data.T = motor_feedback_data.T;
			motor1_feedback_data.W = motor_feedback_data.W;
			motor1_feedback_data.Pos = motor_feedback_data.Pos;
			break;
		case 2 :  
			step = 2;
			motor2_feedback_data.T = motor_feedback_data.T;
			motor2_feedback_data.W = motor_feedback_data.W;
			motor2_feedback_data.Pos = motor_feedback_data.Pos;	
			break;
		case 3 :
			step = 3;			
			motor3_feedback_data.T = motor_feedback_data.T;
			motor3_feedback_data.W = motor_feedback_data.W;
			motor3_feedback_data.Pos = motor_feedback_data.Pos;	
			break;
		case 4 :  
			step = 4;			
			motor4_feedback_data.T = motor_feedback_data.T;
			motor4_feedback_data.W = motor_feedback_data.W;
			motor4_feedback_data.Pos = motor_feedback_data.Pos;
			break;		
		case 5 :  
			step = 5;			
			motor5_feedback_data.T = motor_feedback_data.T;
			motor5_feedback_data.W = motor_feedback_data.W;
			motor5_feedback_data.Pos = motor_feedback_data.Pos;	
		break;
		case 6 :  
			step = 6;			
			motor6_feedback_data.T = motor_feedback_data.T;
			motor6_feedback_data.W = motor_feedback_data.W;
			motor6_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
		case 7 :  
			step = 7;			
			motor7_feedback_data.T = motor_feedback_data.T;
			motor7_feedback_data.W = motor_feedback_data.W;
			motor7_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
		case 8 :  
			step = 8;			
			motor8_feedback_data.T = motor_feedback_data.T;
			motor8_feedback_data.W = motor_feedback_data.W;
			motor8_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
		case 9 :  
			step = 9;			
			motor9_feedback_data.T = motor_feedback_data.T;
			motor9_feedback_data.W = motor_feedback_data.W;
			motor9_feedback_data.Pos = motor_feedback_data.Pos;		
		break;
	 }
	DMA_ClearITPendingBit(DEBUG_USART_DMA_STREAM , DMA_IT_TCIF1); 
}

void RC_DMA_IRQHandler(void)
{
	RC_RX_flag = 1;
	DMA_ClearITPendingBit(RC_USART_DMA_STREAM , DMA_IT_TCIF2); 
}

//imu陀螺仪CAN接收中断----------------------------------------------------
void CAN_RX_IRQHandler(void)
{
//	uint16_t imu_Rx_data[3][3];
	/*从邮箱中读出报文*/
	CAN_Receive(CAN_x, CAN_FIFO0, &RxMessage);
	/* 比较ID*/ 
	if((RxMessage.StdId==0x50) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
	{
		switch(RxMessage.Data[1])
		{
//			case 0x51 ://加速度
//				imu_Rx_data[0][1] = 0.009f+ ((int16_t)(RxMessage.Data[3]<<8)|(int16_t)RxMessage.Data[2])/32768.0f * 16.0f;
//				imu_Rx_data[0][2] = ((int16_t)(RxMessage.Data[5]<<8)|(int16_t)RxMessage.Data[4])/32768.0f * 16.0f;
//				imu_Rx_data[0][3] = ((int16_t)(RxMessage.Data[7]<<8)|(int16_t)RxMessage.Data[6])/32768.0f * 16.0f;
//				break;
//			case 0x52 :
//				imu_Rx_data[1][1] = ((int16_t)(RxMessage.Data[3]<<8)|(int16_t)RxMessage.Data[2])/ 32768.0f * 2000.0f;
//				imu_Rx_data[1][2] = ((int16_t)(RxMessage.Data[5]<<8)|(int16_t)RxMessage.Data[4])/ 32768.0f * 2000.0f;
//				imu_Rx_data[1][3] = ((int16_t)(RxMessage.Data[7]<<8)|(int16_t)RxMessage.Data[6])/ 32768.0f * 2000.0f;
//				break;
//			case 0x53 :
//				//----X轴
//				imu_Rx_data[2][1] = ((int16_t)(RxMessage.Data[3]<<8)|RxMessage.Data[2])/32768.0f*180.0f;
//			  //----Y轴
//				imu_Rx_data[2][2] = ((int16_t)(RxMessage.Data[5]<<8)|RxMessage.Data[4])/32768.0f*180.0f;
//				//----Z轴
//				imu_Rx_data[2][3] = ((int16_t)(RxMessage.Data[7]<<8)|RxMessage.Data[6])/32768.0f*180.0f;
//				break;
//			case 0x54 :
//				imu_Rx_data[3][1] = ((int16_t)(RxMessage.Data[3]<<8)|RxMessage.Data[2]);
//				imu_Rx_data[3][2] = ((int16_t)(RxMessage.Data[5]<<8)|RxMessage.Data[4]);
//				imu_Rx_data[3][3] = ((int16_t)(RxMessage.Data[7]<<8)|RxMessage.Data[6])/120;
//				break;
		}
//		if(RxMessage.Data[1] == 0x53)
//		{
//			printf("角度 x = %f,y = %f,z = %f\n",imu_Rx_data[2][1],imu_Rx_data[2][2],imu_Rx_data[2][3]);
//			flag++ ;//接收成功  
//		}
	}

}




void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



