/**
 * @file motor_pid_params.h
 * @brief 电机 PID 参数集中管理
 * @details
 * - 所有 PID 参数在一处定义
 * - 使用表驱动设计
 * - 易于调试和修改
 */

#ifndef __MOTOR_PID_PARAMS_H
#define __MOTOR_PID_PARAMS_H

#include "struct_typedef.h"

/* ==================== PID 参数表 ==================== */

/**
 * @brief 电机速度环 PID 参数
 * @details [Kp, Ki, Kd] - 按电机 ID (1-9) 索引
 * @note 索引 0 未使用，从索引 1 开始
 */
const fp32 MOTOR_SPEED_PID_PARAMS[10][3] = {
    {0.0f,   0.0f,    0.0f},     // [0] 未使用
    {0.009f, 0.0006f, 0.05f},    // [1] 电机1
    {0.01f,  0.0006f, 0.015f},   // [2] 电机2
    {0.01f,  0.0006f, 0.015f},   // [3] 电机3
    {0.01f,  0.0006f, 0.015f},   // [4] 电机4
    {0.01f,  0.0006f, 0.015f},   // [5] 电机5
    {0.01f,  0.0006f, 0.015f},   // [6] 电机6
    {0.01f,  0.0006f, 0.015f},   // [7] 电机7
    {0.01f,  0.0006f, 0.015f},   // [8] 电机8
    {0.01f,  0.0006f, 0.015f},   // [9] 电机9
};

/**
 * @brief 电机位置环 PID 参数
 * @details [Kp, Ki, Kd] - 按电机 ID (1-9) 索引
 * @note 索引 0 未使用，从索引 1 开始
 */
const fp32 MOTOR_ANGLE_PID_PARAMS[10][3] = {
    {0.0f,   0.0f,  0.0f},       // [0] 未使用
    {100.0f, 0.1f,  0.0f},       // [1] 电机1
    {40.90f, 0.0f,  40.0f},      // [2] 电机2
    {40.90f, 0.0f,  40.0f},      // [3] 电机3
    {40.90f, 0.0f,  40.0f},      // [4] 电机4
    {40.90f, 0.0f,  40.0f},      // [5] 电机5
    {40.90f, 0.0f,  40.0f},      // [6] 电机6
    {40.90f, 0.0f,  40.0f},      // [7] 电机7
    {40.90f, 0.0f,  40.0f},      // [8] 电机8
    {40.90f, 0.0f,  40.0f},      // [9] 电机9
};

/**
 * @brief 陀螺仪 Z 轴 PID 参数
 * @details [Kp, Ki, Kd]
 */
const fp32 IMU_Z_AXIS_PID_PARAMS[3] = {
    0.0f, 0.0f, 0.0f
};

/* ==================== PID 限幅参数 ==================== */

/**
 * @brief 电机速度环的最大输出（限幅）
 * @details 按电机 ID 索引
 */
const float MOTOR_SPEED_PID_MAX_OUT[10] = {
    0.0f,   // [0] 未使用
    3.5f,   // [1] 电机1
    3.5f,   // [2] 电机2
    3.5f,   // [3] 电机3
    3.5f,   // [4] 电机4
    3.5f,   // [5] 电机5
    3.5f,   // [6] 电机6
    3.5f,   // [7] 电机7
    3.5f,   // [8] 电机8
    3.5f,   // [9] 电机9
};

/**
 * @brief 电机速度环的最大积分输出（限幅）
 * @details 按电机 ID 索引
 */
const float MOTOR_SPEED_PID_MAX_I_OUT[10] = {
    0.0f,   // [0] 未使用
    0.2f,   // [1] 电机1
    0.2f,   // [2] 电机2
    0.2f,   // [3] 电机3
    0.2f,   // [4] 电机4
    0.2f,   // [5] 电机5
    0.2f,   // [6] 电机6
    0.2f,   // [7] 电机7
    0.2f,   // [8] 电机8
    0.2f,   // [9] 电机9
};

/**
 * @brief 电机位置环的最大输出（限幅）
 * @details 按电机 ID 索引
 */
const float MOTOR_ANGLE_PID_MAX_OUT[10] = {
    0.0f,      // [0] 未使用
    10000.0f,  // [1] 电机1
    10000.0f,  // [2] 电机2
    10000.0f,  // [3] 电机3
    10000.0f,  // [4] 电机4
    10000.0f,  // [5] 电机5
    10000.0f,  // [6] 电机6
    10000.0f,  // [7] 电机7
    10000.0f,  // [8] 电机8
    10000.0f,  // [9] 电机9
};

/**
 * @brief 电机位置环的最大积分输出（限幅）
 * @details 按电机 ID 索引
 */
const float MOTOR_ANGLE_PID_MAX_I_OUT[10] = {
    0.0f,   // [0] 未使用
    2.0f,   // [1] 电机1
    2.0f,   // [2] 电机2
    2.0f,   // [3] 电机3
    2.0f,   // [4] 电机4
    2.0f,   // [5] 电机5
    2.0f,   // [6] 电机6
    2.0f,   // [7] 电机7
    2.0f,   // [8] 电机8
    2.0f,   // [9] 电机9
};

/* ==================== 辅助宏 ==================== */

/**
 * @brief 获取指定电机的速度 PID 参数
 */
#define GET_SPEED_PID_PARAMS(motor_id) (MOTOR_SPEED_PID_PARAMS[(motor_id)])

/**
 * @brief 获取指定电机的角度 PID 参数
 */
#define GET_ANGLE_PID_PARAMS(motor_id) (MOTOR_ANGLE_PID_PARAMS[(motor_id)])

/**
 * @brief 获取指定电机的速度限幅
 */
#define GET_SPEED_PID_LIMIT(motor_id) (MOTOR_SPEED_PID_MAX_OUT[(motor_id)])

/**
 * @brief 获取指定电机的角度限幅
 */
#define GET_ANGLE_PID_LIMIT(motor_id) (MOTOR_ANGLE_PID_MAX_OUT[(motor_id)])

#endif /* __MOTOR_PID_PARAMS_H */
