/**
 * @file control_task.c
 * @brief 控制任务实现 - 集中管理所有控制逻辑
 * @details 
 * - 避免在 main.c 中堆积代码
 * - 减少全局变量声明，改用结构体
 * - 提高代码可维护性
 */

#include "control_task.h"
#include "bsp_ma_su.h"
#include "bsp_rc.h"
#include "bsp_imu.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include "bsp_key.h"

/* ==================== 全局变量 ==================== */

/** 全局狗状态对象 */
static dog_state_t dog_state = {0};

/** 目标关节角度 - 在 system_data.h 中声明的全局变量
 * 映射关系：
 *  - rang__2, rang__3 = 腿1 (电机2,3)
 *  - rang__4, rang__5 = 腿2 (电机4,5)
 *  - rang__6, rang__7 = 腿3 (电机6,7)
 *  - rang__8, rang__9 = 腿4 (电机8,9)
 */

/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化全局狗状态
 */
static void _init_dog_state(void)
{
    dog_state.step = 1;
    dog_state.step_rate = 200;
    dog_state.time_currently = 0;
    dog_state.time_turn = 0;
    dog_state.motor_send_id = 1;
    dog_state.is_running = 0;
    dog_state.dog_init_time = 0;
    dog_state.printf_flag = 0;
    dog_state.tim_t=1;
    dog_state.watch_self = 0;        // 看门狗计数器
    dog_state.motor_stop_flag = 0;   // 电机停止标志
    dog_state.init_done_flag = 0;    // 初始化标志
}

/**
 * @brief 初始化控制系统
 */
int control_task_init(void)
{
    	//系统时钟
   HSE_SetSysClock(12,336, 2, 7);
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
	  /*初始化按键*/
   Key_GPIO_Config();
	//PID初始化--与--限幅-----------------------------------
	PID_Init();
	//给电机模式等初始化
	Motor_Mode_Init();
	//电机位置偏移表运行时初始化
	motor_pos_init();
    _init_dog_state();
    return 0;
}

/* ==================== 定时器中断处理 ==================== */

/**
 * @brief 1ms 定时器中断处理函数
 * @details 只做计数和标志设置，避免重计算
 */
void control_task_timer_handler(void)
{
    dog_state_t *state = &dog_state;
    
    // 步频选择
    if ((rc_rc.s1 == 3) && (rc_rc.s2 == 1)) {
        state->step_rate = 150;
    } else if ((rc_rc.s1 == 3) && (rc_rc.s2 == 3)) {
        state->step_rate = 200;
    }
    
    // 更新步态时间
    state->time_currently++;
    if (state->time_currently >= state->step_rate) {
        state->time_currently = 0;
        state->step = 0;  // 触发主循环执行步态计算
    }
    
    // 转向时间计数
    state->time_turn++;
    if (state->time_turn >= 400) {
        state->time_turn = 0;
    }
    
    // 初始化计时
    if (state->dog_init_time < 2001) {
        state->dog_init_time++;
    }
    
    // 跳跃计时
    if (state->jump_flag) {
        state->jump_time++;
    }
    if (state->backflip_flag) {
        state->backflip_time++;
    }
}

/* ==================== 步态计算 ==================== */

/**
 * @brief 计算腿的足端轨迹
 * @param leg_idx 腿的索引 (0-3)
 * @details 调用 counter_motion() 计算关节角度，结果写入各电机目标角度全局变量
 */
static void _calculate_leg_wave(int leg_idx)
{
    if (leg_idx < 0 || leg_idx >= 4) return;
    
    dog_state_t *state = &dog_state;
    foot_track_t *foot = &g_foot_track[leg_idx];
    
    // 根据腿号映射到对应的步态和角度输出
    switch (leg_idx) {
        case 0:  // 腿1 (电机2,3)
            if ((state->step == 2) || (state->step == 3)) {
                foot_track(&foot->x, &foot->y, state->step_rate, 
                          state->time_currently, 200, state->time_turn, 2, 1);
                counter_motion(foot->x, foot->y, &rang__2, &rang__3);
            }
            break;
        case 1:  // 腿2 (电机4,5)
            if ((state->step == 4) || (state->step == 5)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 1, 3);
                counter_motion(foot->x, foot->y, &rang__5, &rang__4);
            }
            break;
        case 2:  // 腿3 (电机6,7)
            if ((state->step == 6) || (state->step == 7)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 2, 2);
                counter_motion(foot->x, foot->y, &rang__6, &rang__7);
            }
            break;
        case 3:  // 腿4 (电机8,9)
            if ((state->step == 8) || (state->step == 9)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 1, 4);
                counter_motion(foot->x, foot->y, &rang__9, &rang__8);
            }
            break;
    }
}

/* ==================== 主控制循环 ==================== */

/**
 * @brief 主控制循环执行函数
 * @details 应在主循环中调用，处理所有控制逻辑和非时间敏感的串口操作
 */
void control_task_execute(void)
{
    dog_state_t *state = &dog_state;
    // 1. RC 遥控解析
    rc_rc_date();
    // 2. 按键控制（用于其他功能）
    Motor_Key();
    // 3. IMU 陀螺仪控制
    IMU_pid_count(0.0f);
    // 4. 步态更新触发
    if (state->step == 0) {
        // 计算所有腿的足端轨迹和关节角度
        for (int i = 0; i < 4; i++) {
            _calculate_leg_trajectory(i);
        }
        
        // 执行 PID 控制
        Motor_pid_count(state->step, rc_rc.ch0 * 0.015f);
        
        // 更新电机数据
        Motor_data_update(state->step);
    }
    
    // 5. 发送电机命令
    Motor_date_send(&state->motor_send_id);
}

/* ==================== 访问器函数 ==================== */

/**
 * @brief 获取全局狗状态对象(對外的狀態接口)
 */ 
dog_state_t* control_task_get_state(void)
{
    return &dog_state;
}


/**
 * @brief 用看门狗/定时器替代按键控制电机启停（中断上下文版本）
 * @note 在1ms 定时器中调用（ GENERAL_TIM_IRQHandler 中调用）
 * @details 
 * - 仅在中断中进行计数和标志设置，处理时间 < 0.5ms
 * - 实际的 PID 清除和电机停止指令发送在 control_task_execute() 主循环中处理
 * - 这样避免了中断中的串口阻塞操作（while 等待 TXE/TC 标志）
 */
void Motor_Key(void)
{
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
    {
       // 清除所有电机 PID（索引 1 开始，符合 1-based 约定）K
        for (int id = 1; id < Mo_Count; id++) {
            PID_clear(&Motor_speed_PID[id]);
            PID_clear(&Motor_rang_PID[id]);
        }
        
        // 检查串口是否空闲，避免冲突（在主循环中执行，不阻塞中断）
        if (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == SET) {
            // 发送电机停止指令
            Usart_SendStr_length(USART6,
                (uint8_t *)(&motor_control_clear), 
                motor_control_clear.hex_len);
        }
    }
}


/**
 * @brief 设置步频
 */
void control_task_set_step_rate(int rate)
{
    if (rate >= 150 && rate <= 250) {
            dog_state.step_rate = rate;
    }
}

/* ==================== 调试函数 ==================== */

/**
 * @brief 打印调试信息
 */
void control_task_print_debug(void)
{
    if (!dog_state.printf_flag) return;
    
    dog_state.printf_flag = 0;
    
    printf("State: step=%d, rate=%d, time=%d, turn=%d\n",
           dog_state.step,
           dog_state.step_rate,
           dog_state.time_currently,
           dog_state.time_turn);
    
    printf("Angles: j2=%f, j3=%f, j4=%f, j5=%f\n",
           joint_angles[0], joint_angles[1],
           joint_angles[2], joint_angles[3]);
}
