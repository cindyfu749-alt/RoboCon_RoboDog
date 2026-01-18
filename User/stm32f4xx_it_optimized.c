/**
 * @file stm32f4xx_it_optimized.c
 * @brief 中断处理程序（优化版本）
 * @details
 * - 简化的中断处理
 * - 使用 control_task 模块处理定时器中断
 * - 保持中断处理时间 < 0.5ms
 */

#include "stm32f4xx_it.h"
#include "./tim/bsp_general_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "control_task.h"
#include "bsp_ma_su.h"

/* ==================== 1ms 定时器中断 ==================== */

/**
 * @brief 通用定时器中断处理函数
 * @details 1ms 周期，处理步态时间更新和标志设置
 * @note 轻量级 - 只做整数计数，< 0.5ms
 */
void GENERAL_TIM_IRQHandler(void)//定时器2
{
    // 调用 control_task 的定时器处理
    control_task_timer_handler();
    
    // 清除中断标志
    TIM_ClearITPendingBit(GENERAL_TIM, TIM_IT_Update);
}

/* ==================== DMA 中断处理 ==================== */

/**
 * @brief 电机 DMA 接收中断处理
 * @details 处理 USART6 的 DMA 接收数据
 */
void Motor_DMA_IRQHandler(void)
{
    // 已由 bsp_usart_dma.c 的底层驱动处理
    // 此处预留用于高级逻辑处理
}

/**
 * @brief RC 遥控 DMA 接收中断处理
 * @details 处理 USART1 的 DMA 接收数据
 */
void RC_DMA_IRQHandler(void)
{
    // 已由 bsp_usart_dma.c 的底层驱动处理
    // 此处预留用于高级逻辑处理
}

/* ==================== 保留的系统中断 ==================== */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {
        // 硬件故障处理
    }
}

void MemManage_Handler(void)
{
    while (1) {
        // 内存管理故障处理
    }
}

void BusFault_Handler(void)
{
    while (1) {
        // 总线故障处理
    }
}

void UsageFault_Handler(void)
{
    while (1) {
        // 使用故障处理
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}
