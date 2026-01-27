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
#include "bsp_can.h"
#define Mo_Count 10 
/* ==================== 电机数据结构 ==================== */

/** 
 * @brief 全局电机控制和反馈数据
 * @note 索引 0 未使用，1-9 对应电机 1-9
 */
extern MOTOR_send motor_control_data[10];        ///< 电机控制命令
extern MOTOR_recv motor_feedback_data[10];        ///< 电机反馈数据
extern MOTOR_send motor_control_clear;      ///< 电机停止命令

/* ==================== PID 输出数据 ==================== */

extern float motor_speed_pid_out[10];    ///< 速度环 PID 输出 (1-9)
extern float motor_angle_pid_out[10];    ///< 位置环 PID 输出 (1-9)
extern float imu_z_pid_out;              ///< IMU Z 轴 PID 输出

/* ==================== PID 结构体 ==================== */
extern pid_type_def Motor_speed_PID[Mo_Count]; //电机速度PID
extern pid_type_def Motor_rang_PID[Mo_Count];  //电机角度PID
extern pid_type_def imu_Z_PID;

/* ==================== KP Ki KD=========== */
/** PID 参数表：[速度Kp, 速度Ki, 速度Kd] */
extern const fp32 motor_speed_pid_data[10][3];
/** PID 参数表：[位置Kp, 位置Ki, 位置Kd] */
extern const fp32 motor_rang_pid_data[10][3];

/* ==================== 步态数据结构 ==================== */
extern float rang__2, rang__3, rang__4, rang__5, rang__6, rang__7, rang__8, rang__9;  //各电机位置
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
extern float joint_angles[8];

/* ==================== 时间计数 ==================== */

extern int time_currently;               ///< 步态内当前时间
extern int time_turn;                    ///< 转向内当前时间
extern int dog_init_time;                ///< 初始化计时

/* ==================== 电机通信 ==================== */

extern int Motor_Send_ID;                ///< 正在发送的电机 ID
extern int Motor_feedback_ID;            ///< 最近接收的电机反馈 ID
extern uint8_t motor_rx_flag;            ///< 电机接收标志

/* ==================== CAN_TX/RX_Msg ==================== */
extern CanTxMsg TxMessage;       //发送缓冲区
extern CanRxMsg RxMessage;		//接收缓冲区

/* ==================== RC 遥控 ==================== */

extern uint8_t RC[18];                     ///< RC 原始数据缓冲
extern rc rc_rc;                           ///< 解析后的 RC 数据

/* ==================== 标志位 ==================== */      
extern float ttl;
extern int Dog_Iinit ;
extern int Dog_flag ;
extern float Motor_set_Pos;                ///< 目标位置

/* ==================== 传感器数据 ==================== */

extern float imu_Rx_data[4][4];             ///< IMU 数据（3x3 矩阵 + 温度）

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
