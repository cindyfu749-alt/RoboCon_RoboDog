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

/* ==================== 全局变量 ==================== */

/** 全局狗状态对象 */
static dog_state_t g_dog_state = {0};

/** 足端轨迹缓冲 */
typedef struct {
    float x;
    float y;
} foot_pos_t;

static foot_pos_t g_foot_track[4] = {0};  // 4条腿的足端坐标

/** 目标关节角度 */
static float g_joint_angles[9] = {0};     // rang__2 至 rang__9

/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化全局狗状态
 */
static void _init_dog_state(void)
{
    g_dog_state.step = 1;
    g_dog_state.step_rate = 200;
    g_dog_state.time_currently = 0;
    g_dog_state.time_turn = 0;
    g_dog_state.motor_send_id = 1;
    g_dog_state.is_running = 0;
    g_dog_state.dog_init_time = 0;
    g_dog_state.print_flag = 0;
}

/**
 * @brief 初始化控制系统
 */
int control_task_init(void)
{
    _init_dog_state();
    return 0;
}

/* ==================== 定时器中断处理 ==================== */

/**
 * @brief 1ms 定时器中断处理函数
 * @details 轻量级 - 只做计数和标志设置，避免重计算
 */
void control_task_timer_handler(void)
{
    dog_state_t *state = &g_dog_state;
    
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
 * @return 无
 */
static void _calculate_leg_trajectory(int leg_idx)
{
    if (leg_idx < 0 || leg_idx >= 4) return;
    
    dog_state_t *state = &g_dog_state;
    foot_pos_t *foot = &g_foot_track[leg_idx];
    float *angle_out;
    
    // 根据腿号映射到对应的步态和角度输出
    switch (leg_idx) {
        case 0:  // 腿1
            if ((state->step == 2) || (state->step == 3)) {
                foot_track(&foot->x, &foot->y, state->step_rate, 
                          state->time_currently, 200, state->time_turn, 2, 1);
                angle_out = &g_joint_angles[0];  // ran__2, ran__3
                counter_motion(foot->x, foot->y, angle_out, &angle_out[1]);
            }
            break;
        case 1:  // 腿2
            if ((state->step == 4) || (state->step == 5)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 1, 3);
                angle_out = &g_joint_angles[3];  // ran__5, ran__4
                counter_motion(foot->x, foot->y, angle_out, &angle_out[1]);
            }
            break;
        case 2:  // 腿3
            if ((state->step == 6) || (state->step == 7)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 2, 2);
                angle_out = &g_joint_angles[4];  // ran__6, ran__7
                counter_motion(foot->x, foot->y, angle_out, &angle_out[1]);
            }
            break;
        case 3:  // 腿4
            if ((state->step == 8) || (state->step == 9)) {
                foot_track(&foot->x, &foot->y, state->step_rate,
                          state->time_currently, 200, state->time_turn, 1, 4);
                angle_out = &g_joint_angles[7];  // ran__9, ran__8
                counter_motion(foot->x, foot->y, angle_out, &angle_out[1]);
            }
            break;
    }
}

/* ==================== 主控制循环 ==================== */

/**
 * @brief 主控制循环执行函数
 * @details 应在主循环中调用，处理所有控制逻辑
 */
void control_task_execute(void)
{
    dog_state_t *state = &g_dog_state;
    
    // 1. RC 解析
    rc_rc_date();
    
    // 2. 按键控制
    Motor_Key();
    
    // 3. IMU 陀螺仪控制
    IMU_pid_count(0.0f);
    
    // 4. 如果触发步态更新
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
 * @brief 获取全局狗状态对象
 */
dog_state_t* control_task_get_state(void)
{
    return &g_dog_state;
}

/**
 * @brief 启动/停止运动
 */
void control_task_set_running(uint8_t enable)
{
    g_dog_state.is_running = enable;
    if (!enable) {
        Motor_Key();  // 触发停止
    }
}

/**
 * @brief 设置步频
 */
void control_task_set_step_rate(int rate)
{
    if (rate >= 150 && rate <= 250) {
        g_dog_state.step_rate = rate;
    }
}

/* ==================== 调试函数 ==================== */

/**
 * @brief 打印调试信息
 */
void control_task_print_debug(void)
{
    if (!g_dog_state.print_flag) return;
    
    g_dog_state.print_flag = 0;
    
    printf("State: step=%d, rate=%d, time=%d, turn=%d\n",
           g_dog_state.step,
           g_dog_state.step_rate,
           g_dog_state.time_currently,
           g_dog_state.time_turn);
    
    printf("Angles: j2=%f, j3=%f, j4=%f, j5=%f\n",
           g_joint_angles[0], g_joint_angles[1],
           g_joint_angles[2], g_joint_angles[3]);
}
