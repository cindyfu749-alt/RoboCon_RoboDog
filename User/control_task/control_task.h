/**
 * @file control_task.h
 * @brief 控制任务模块 - 集中管理所有控制逻辑
 * @details 避免在 main.c 中堆积代码，提供清晰的控制流程
 */

#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "stm32f4xx.h"
#include "struct_typedef.h"

/* ==================== 系统状态定义 ==================== */
typedef struct {
    // 运动控制
    int step;                    // 步态计数器
    int step_rate;               // 步频
    int time_currently;          // 步态内当前时间
    int time_turn;               // 转向内当前时间
    int tim_t;               // 延时等待
    int step_turn_flag;          // 步幅/转向切换标志

    // 电机控制
    int motor_send_id;           // 正在发送的电机 ID
    int motor_feedback_id;       // 最近接收反馈的电机 ID

    //看門狗
    int watch_self;
    uint8_t motor_stop_flag;     // 电机停止请求标志（从中断驱动，在主循环处理）
    uint8_t init_done_flag;      // 初始化完成标志（防止重复初始化）
    
    // 状态标志
    uint8_t is_running;          // 运动中标志
    int dog_init_time;           // 初始化计时
    int Dog_Iinit;          // 狗初始化站立时间
    
    // 跳跃相关
    int jump_flag;               // 跳跃标志
    int jump_time;               // 跳跃计时
    int jump_flag2;               // 跳跃标志
    int jump_time2;               // 跳跃计时
    int backflip_flag;           // 空翻标志
    int backflip_time;           // 空翻计时
    int dog_rest_flag;            // 跳跃计时
    
    // 调试标志
    uint8_t printf_flag;          // 打印调试信息标志
    
} dog_state_t;


/* ==================== 公共接口 ==================== */

/**
 * @brief 初始化控制系统
 * @return 0=成功, 其他=失败
 */
int control_task_init(void);

/**
 * @brief 1ms 定时器中断处理函数
 * @details 必须在 GENERAL_TIM_IRQHandler 中调用
 * @return 无
 */
void control_task_timer_handler(void);

/**
 * @brief 主控制循环执行
 * @details 应在主循环中调用，或与定时器组合使用
 * @return 无
 */
void control_task_execute(void);

/**
 * @brief 获取全局狗状态对象
 * @return 指向 dog_state_t 的指针
 */
dog_state_t* control_task_get_state(void);

/**
 * @brief 启动/停止运动
 * @param enable 1=启动, 0=停止
 * @return 无
 */
void control_task_set_running(uint8_t enable);

/**
 * @brief 设置步频
 * @param rate 步频值 (150-200)
 * @return 无
 */
void control_task_set_step_rate(int rate);

/**
 * @brief 电机看门狗控制函数（在1ms定时器中调用）
 * @details 在中断中设置标志位，避免阻塞操作
 * @return 无
 */
void Motor_Key(void);

#endif /* __CONTROL_TASK_H */
