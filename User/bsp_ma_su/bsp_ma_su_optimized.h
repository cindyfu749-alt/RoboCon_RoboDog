/**
 * @file bsp_ma_su.h
 * @brief 电机和步态控制模块（优化版本）
 * @details 
 * - 减少 extern 声明，改用数组索引
 * - PID 参数表驱动
 * - 提高代码复用性
 */

#ifndef _MA_SU_H
#define _MA_SU_H

#include "stm32f4xx.h"
#include "struct_typedef.h"

/* ==================== PID 参数表 ==================== */

/** PID 参数表：[速度Kp, 速度Ki, 速度Kd] */
extern const fp32 g_motor_speed_pid_params[10][3];

/** PID 参数表：[位置Kp, 位置Ki, 位置Kd] */
extern const fp32 g_motor_angle_pid_params[10][3];

/** 电机控制和反馈数据数组 */
extern MOTOR_send g_motor_ctrl[10];
extern MOTOR_recv g_motor_fdbk[10];

/** PID 输出数组 */
extern float g_motor_speed_pid_out[10];
extern float g_motor_angle_pid_out[10];

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化电机系统
 * @return 0=成功
 */
int motor_system_init(void);

/**
 * @brief 初始化 PID 控制器
 * @return 无
 */
void motor_pid_init_all(void);

/**
 * @brief 计算单个电机的 PID 控制输出
 * @param motor_id 电机 ID (1-9)
 * @param target_pos 目标位置
 * @param feedback_pos 反馈位置
 * @return 0=成功
 */
int motor_pid_calculate(int motor_id, float target_pos, float feedback_pos);

/**
 * @brief 更新电机控制数据
 * @param motor_id 电机 ID (1-9)
 * @return 0=成功
 */
int motor_data_update_by_id(int motor_id);

/**
 * @brief 发送所有待发送的电机命令
 * @return 无
 */
void motor_send_all_commands(void);

/**
 * @brief 按键控制电机启停
 * @return 无
 */
void motor_key_control(void);

/* ==================== 步态计算函数 ==================== */

/**
 * @brief 计算足端轨迹
 * @param x 输出 X 坐标
 * @param y 输出 Y 坐标
 * @param cycle_time 步态周期
 * @param current_time 当前时间
 * @param lift_time 抬起时间
 * @param turn_time 转向时间
 * @param move_mode 运动模式
 * @param direction_mode 方向模式
 * @return 无
 */
void gait_foot_track(float *x, float *y, 
                     int cycle_time, int current_time,
                     int lift_time, int turn_time,
                     int move_mode, int direction_mode);

/**
 * @brief 逆运动学计算
 * @param x 足端 X 坐标
 * @param y 足端 Y 坐标
 * @param angle1 输出角度1
 * @param angle2 输出角度2
 * @return 0=成功
 */
int gait_inverse_kinematics(float x, float y, float *angle1, float *angle2);

/**
 * @brief 位置速度解耦控制
 * @param feedback_pos 反馈位置
 * @param target_pos 目标位置
 * @param speed_limit 速度限制
 * @return 0=成功
 */
int motor_pos_speed_control(float feedback_pos, float target_pos, float speed_limit);

#endif /* _MA_SU_H */
