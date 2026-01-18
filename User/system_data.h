/**
 * @file system_data.h
 * @brief 系统数据结构定义（优化版本）
 * @details
 * - 集中定义所有全局数据结构
 * - 统一的命名约定
 * - 便于查找和修改
 */

#ifndef __SYSTEM_DATA_H
#define __SYSTEM_DATA_H

#include "stm32f4xx.h"
#include "struct_typedef.h"
#include "GO-M8010-6.h"

/* ==================== 电机数据结构 ==================== */

/** 
 * @brief 全局电机控制和反馈数据
 * @note 索引 0 未使用，1-9 对应电机 1-9
 */
extern MOTOR_send g_motor_ctrl[10];        ///< 电机控制命令
extern MOTOR_recv g_motor_fdbk[10];        ///< 电机反馈数据
extern MOTOR_send g_motor_ctrl_clear;      ///< 电机停止命令

/* ==================== PID 输出数据 ==================== */

extern float g_motor_speed_pid_out[10];    ///< 速度环 PID 输出 (1-9)
extern float g_motor_angle_pid_out[10];    ///< 位置环 PID 输出 (1-9)
extern float g_imu_z_pid_out;              ///< IMU Z 轴 PID 输出

/* ==================== 步态数据结构 ==================== */

/**
 * @brief 足端轨迹数据
 */
typedef struct {
    float x;                    ///< 足端 X 坐标
    float y;                    ///< 足端 Y 坐标
} foot_track_t;

/** 全局足端轨迹（4 条腿） */
extern foot_track_t g_foot_track[4];

/**
 * @brief 关节角度数据
 * @note 索引对应关系：
 *  - [0,1] = 腿1 (电机2,3)
 *  - [2,3] = 腿2 (电机5,4)
 *  - [4,5] = 腿3 (电机6,7)
 *  - [6,7] = 腿4 (电机9,8)
 */
extern float g_joint_angles[8];

/* ==================== 时间计数 ==================== */

extern int g_time_currently;               ///< 步态内当前时间
extern int g_time_turn;                    ///< 转向内当前时间
extern int g_dog_init_time;                ///< 初始化计时

/* ==================== 电机通信 ==================== */

extern int g_motor_send_id;                ///< 正在发送的电机 ID
extern int g_motor_feedback_id;            ///< 最近接收的电机反馈 ID
extern uint8_t g_motor_rx_flag;            ///< 电机接收标志

/* ==================== RC 遥控 ==================== */

extern uint8_t g_rc_raw_data[18];          ///< RC 原始数据缓冲
extern rc rc_rc;                           ///< 解析后的 RC 数据

/* ==================== 标志位 ==================== */

extern uint8_t g_print_flag;               ///< 打印标志
extern int g_step;                         ///< 步态计数器
extern int g_step_turn_flag;               ///< 步幅/转向标志
extern int g_dog_jump_flag;                ///< 跳跃标志
extern int g_dog_jump_time;                ///< 跳跃计时
extern int g_backflip_flag;                ///< 空翻标志
extern int g_backflip_time;                ///< 空翻计时

/* ==================== 传感器数据 ==================== */

extern float g_imu_data[4][4];             ///< IMU 数据（3x3 矩阵 + 温度）

/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化所有全局数据结构
 */
void system_data_init(void);

/**
 * @brief 获取指定电机的控制数据
 * @param motor_id 电机 ID (1-9)
 * @return 指向 MOTOR_send 的指针，失败返回 NULL
 */
MOTOR_send* system_data_get_motor_ctrl(int motor_id);

/**
 * @brief 获取指定电机的反馈数据
 * @param motor_id 电机 ID (1-9)
 * @return 指向 MOTOR_recv 的指针，失败返回 NULL
 */
MOTOR_recv* system_data_get_motor_fdbk(int motor_id);

/**
 * @brief 获取指定腿的足端轨迹
 * @param leg_id 腿 ID (0-3)
 * @return 指向 foot_track_t 的指针，失败返回 NULL
 */
foot_track_t* system_data_get_foot_track(int leg_id);

/**
 * @brief 获取指定腿的关节角度
 * @param leg_id 腿 ID (0-3)
 * @param angle1 输出第一关节角度指针
 * @param angle2 输出第二关节角度指针
 * @return 0=成功, -1=失败
 */
int system_data_get_leg_angles(int leg_id, float **angle1, float **angle2);

#endif /* __SYSTEM_DATA_H */
