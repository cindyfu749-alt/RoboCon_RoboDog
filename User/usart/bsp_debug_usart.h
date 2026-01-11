#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>



//引脚定义
/*******************************************************/
#define DEBUG_USART                             USART6 
#define DEBUG_USART_CLK                         RCC_APB2Periph_USART6 

#define DEBUG_USART_RX_GPIO_PORT                GPIOG 
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOG 
#define DEBUG_USART_RX_PIN                      GPIO_Pin_9
#define DEBUG_USART_RX_AF                       GPIO_AF_USART6
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource9

#define DEBUG_USART_TX_GPIO_PORT                GPIOG
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOG
#define DEBUG_USART_TX_PIN                      GPIO_Pin_14
#define DEBUG_USART_TX_AF                       GPIO_AF_USART6
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource14
/************************************************************/
//电机控制USART转485-----------------------------------------//
#define MOTOR_CONTROL_USART                             USART3
#define MOTOR_CONTROL_USART_CLK                         RCC_APB1Periph_USART3

#define MOTOR_CONTROL_USART_RX_GPIO_PORT                GPIOD
#define MOTOR_CONTROL_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD 
#define MOTOR_CONTROL_USART_RX_PIN                      GPIO_Pin_9
#define MOTOR_CONTROL_USART_RX_AF                       GPIO_AF_USART3
#define MOTOR_CONTROL_USART_RX_SOURCE                   GPIO_PinSource9

#define MOTOR_CONTROL_USART_TX_GPIO_PORT                GPIOD
#define MOTOR_CONTROL_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define MOTOR_CONTROL_USART_TX_PIN                      GPIO_Pin_8
#define MOTOR_CONTROL_USART_TX_AF                       GPIO_AF_USART3
#define MOTOR_CONTROL_USART_TX_SOURCE                   GPIO_PinSource8
//RC USART1
#define rc_DEBUG_USART                       USART1
#define rc_DEBUG_USART_CLK                   RCC_APB2Periph_USART1

#define rc_DEBUG_USART_RX_GPIO_PORT          GPIOB
#define rc_DEBUG_USART_RX_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define rc_DEBUG_USART_RX_PIN                GPIO_Pin_7
#define rc_DEBUG_USART_RX_AF                 GPIO_AF_USART1
#define rc_DEBUG_USART_RX_SOURCE             GPIO_PinSource7

#define rc_DEBUG_USART_TX_GPIO_PORT          GPIOB
#define rc_DEBUG_USART_TX_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define rc_DEBUG_USART_TX_PIN                GPIO_Pin_6
#define rc_DEBUG_USART_TX_AF                 GPIO_AF_USART1
#define rc_DEBUG_USART_TX_SOURCE             GPIO_PinSource6

#define rc_DEBUG_USART_BAUDRATE              100000

//-------------------------------------------------------------//
//中断函数名
#define RS485_USART_IRQHandler                   USART6_IRQHandler
#define RS485_USART_IRQ                 					USART6_IRQn
/************************************************************/
//中断函数名
#define RC_USART_IRQHandler                   USART1_IRQHandler
#define RC_USART_IRQ                 					USART1_IRQn




//串口波特率
#define DEBUG_USART_BAUDRATE                    4000000
#define MOTOR_CONTROL_USART_BAUDRATE            115200 
void NVIC_Configuration(void);
void MOTOR_CONTROL_USART_Config(void);
void Debug_USART_Config(void);
void RC_USART1_Config(void);
	
static void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch );
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
void Usart_SendString( USART_TypeDef * pUSARTx, uint8_t *str);
//int fputc(int ch, FILE *f);

#endif /* __USART1_H */
