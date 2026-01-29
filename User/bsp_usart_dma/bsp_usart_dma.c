#include "bsp_usart_dma.h"
#include "GO-M8010-6.h"

uint8_t RC[18];


/**
  * @brief  USART1 TX DMA 配置，内存到外设(USART1->DR)
  * @param  无
  * @retval 无
  */
void USART6_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /*开启DMA时钟*/
  RCC_AHB1PeriphClockCmd(DEBUG_USART_DMA_CLK, ENABLE);
  
  /* 复位初始化DMA数据流 */
  DMA_DeInit(DEBUG_USART_DMA_STREAM);

  /* 确保DMA数据流复位完成 */
  while (DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != DISABLE)  
	{
		
  }

  /*usart6 Rx对应dma2，通道5，数据流1*/	
  DMA_InitStructure.DMA_Channel = DEBUG_USART_DMA_CHANNEL;  
  /*设置DMA源：串口数据寄存器地址*/
  DMA_InitStructure.DMA_PeripheralBaseAddr = DEBUG_USART_DR_BASE;	 
  /*内存地址(要传输的变量的指针)*/
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(&motor_feedback_date_buffer);
  /*方向：从外设到内存*/		
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
  /*传输大小DMA_BufferSize=SENDBUFF_SIZE  数据的个数*/	
  DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
  /*外设地址不增*/	    
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  /*内存地址自增*/
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
  /*外设数据单位*/	
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  /*内存数据单位 8bit*/
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
  /*DMA模式：传输一次*/
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 
  /*优先级：VeryHigh*/	
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;      
  /*禁用FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;    
  /*单次模式*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
  /*单次模式*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
  /*配置DMA1的数据流1*/		   
  DMA_Init(DEBUG_USART_DMA_STREAM, &DMA_InitStructure);
  
  /*使能DMA*/
  DMA_Cmd(DEBUG_USART_DMA_STREAM, ENABLE);
  
  /* 等待DMA数据流有效*/
  while(DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != ENABLE)
  {
  }   
}
void Motor_DMA_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = Motor_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
//		/*配置串口接收中断*/
	DMA_ITConfig(DEBUG_USART_DMA_STREAM, DMA_IT_TC, ENABLE);
}
void USART1_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /*开启DMA时钟*/
  RCC_AHB1PeriphClockCmd(RC_USART_DMA_CLK, ENABLE);
  
  /* 复位初始化DMA数据流 */
  DMA_DeInit(RC_USART_DMA_STREAM);

  /* 确保DMA数据流复位完成 */
  while (DMA_GetCmdStatus(RC_USART_DMA_STREAM) != DISABLE)  
	{
		
  }

  /*usart3 Rx对应dma1，通道4，数据流1*/	
  DMA_InitStructure.DMA_Channel = RC_USART_DMA_CHANNEL;  
  /*设置DMA源：串口数据寄存器地址*/
  DMA_InitStructure.DMA_PeripheralBaseAddr = RC_USART_DR_BASE;	 
  /*内存地址(要传输的变量的指针)*/
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(RC);
  /*方向：从外设到内存*/		
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
  /*传输大小DMA_BufferSize=SENDBUFF_SIZE  数据的个数*/	
  DMA_InitStructure.DMA_BufferSize = RC_SENDBUFF_SIZE;
  /*外设地址不增*/	    
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  /*内存地址自增*/
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
  /*外设数据单位*/	
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  /*内存数据单位 8bit*/
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
  /*DMA模式：传输一次*/
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 
  /*优先级：VeryHigh*/	
  DMA_InitStructure.DMA_Priority = DMA_Priority_High  ;      
  /*禁用FIFO*/
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;    
  /*存储器突发传输 16个节拍*/
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
  /*外设突发传输 1个节拍*/
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
  /*配置DMA1的数据流3*/		   
  DMA_Init(RC_USART_DMA_STREAM, &DMA_InitStructure);
  
  /*使能DMA*/
  DMA_Cmd(RC_USART_DMA_STREAM, ENABLE);
  
  /* 等待DMA数据流有效*/
  while(DMA_GetCmdStatus(RC_USART_DMA_STREAM) != ENABLE)
  {
  }   
}
void RC_DMA_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = RC_USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
//		/*配置串口接收中断*/
	DMA_ITConfig(RC_USART_DMA_STREAM, DMA_IT_TC, ENABLE);
}


/*********************************************END OF FILE**********************/
