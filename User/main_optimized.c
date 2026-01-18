/**
 * @file main_optimized.c
 * @brief 主程序（优化版本）
 * @details
 * - 清晰的初始化流程
 * - 精简的主循环
 * - 使用 control_task 模块集中管理逻辑
 * - 减少全局变量声明
 */

#include "stm32f4xx.h"
#include "bsp_clkconfig.h"
#include "./usart/bsp_debug_usart.h"
#include "bsp_usart_dma.h"
#include "bsp_rc.h"
#include "./can/bsp_can.h"
#include "./key/bsp_key.h"
#include "./tim/bsp_general_tim.h"
#include "bsp_imu.h"
#include "control_task.h"

/* ==================== 系统配置常数 ==================== */

/** 启动等待计数 */
#define STARTUP_WAIT_COUNT 100000

/** 主循环延迟计数 */
#define MAIN_LOOP_DELAY    5000

/* ==================== 系统初始化函数 ==================== */

/**
 * @brief 初始化所有硬件接口
 * @details 串口、DMA、时钟等
 */
static void _init_hardware(void)
{
    // 串口初始化
    RC_USART1_Config();              // RC 遥控
    Debug_USART_Config();            // 调试串口 USART6
    MOTOR_CONTROL_USART_Config();    // 电机控制 USART3
    
    // DMA 配置
    USART6_DMA_Config();
    USART1_DMA_Config();
    
    // DMA 中断配置
    RC_DMA_NVIC_Configuration();
    Motor_DMA_NVIC_Configuration();
    
    // 启动 DMA 接收请求
    USART_DMACmd(rc_DEBUG_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);
}

/**
 * @brief 初始化时钟系统
 * @details 配置系统时钟为 168 MHz
 */
static void _init_clock(void)
{
    // 系统时钟：168 MHz
    HSE_SetSysClock(12, 336, 2, 7);
}

/**
 * @brief 初始化控制系统
 * @details PID、电机模式、按键等
 */
static void _init_control(void)
{
    // PID 初始化
    motor_pid_init_all();
    
    // 电机模式初始化
    Motor_Mode_Init();
    
    // 按键初始化
    Key_GPIO_Config();
    
    // 控制任务初始化
    control_task_init();
}

/**
 * @brief 启动等待
 * @details 等待硬件稳定和初始化完成
 */
static void _startup_wait(void)
{
    volatile int wait_count = 0;
    while (wait_count < STARTUP_WAIT_COUNT) {
        wait_count++;
    }
}

/**
 * @brief 主程序初始化
 * @details 按顺序初始化系统的各个部分
 */
static void _system_init(void)
{
    _init_hardware();
    _init_clock();
    _init_control();
    _startup_wait();
    
    // 启动定时器（产生 1ms 中断）
    TIMx_Configuration();
}

/* ==================== 主循环 ==================== */

/**
 * @brief 主程序入口
 */
int main(void)
{
    // 系统初始化
    _system_init();
    
    // 主循环
    while (1) {
        // 执行控制任务
        control_task_execute();
        
        // 简单延迟（防止循环过快）
        volatile int delay = 0;
        while (delay < MAIN_LOOP_DELAY) {
            delay++;
        }
    }
    
    return 0;
}

/* ==================== 调试函数 ==================== */

/**
 * @brief 打印系统状态
 */
void system_print_status(void)
{
    dog_state_t *state = control_task_get_state();
    
    if (!state->print_flag) return;
    
    printf("=== System Status ===\n");
    printf("Step: %d, Rate: %d\n", state->step, state->step_rate);
    printf("Time: %d, Turn: %d\n", state->time_currently, state->time_turn);
    printf("Init: %d/2000\n", state->dog_init_time);
}
