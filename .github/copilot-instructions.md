# Copilot 使用说明（四足机器狗项目）

## 项目概览
STM32F4 嵌入式四足机器狗控制系统。使用 ST Standard Peripheral Library、Keil MDK (UV5)、串口/DMA 通信、双环级联 PID 控制、逆运动学步态算法。

**CMake 配置**: 项目包含 CMakeLists.txt，支持在 VS Code 中完整的 IntelliSense 和代码导航。

## 构建与调试

### 使用 Keil MDK（推荐用于编译/下载）
- **IDE**: Keil MDK UV5 → [project/RVMDK(V5)/MOD_F427IIH.uvprojx](project/RVMDK(V5)/MOD_F427IIH.uvprojx)
- **目标芯片**: STM32F427IIH（Flash: 2MB, SRAM: 256KB）
- **启动与下载**: Build → 连接 ST-Link 调试器 → Download + Debug

### VS Code IntelliSense 配置（用于代码浏览和导航）
1. 安装扩展: **C/C++ (Microsoft)**, **CMake**, **CMake Tools**
2. 打开项目，CMake 会自动配置（见 `.vscode/settings.json` 中 `cmake.configureOnOpen: true`）
3. 编译时运行 CMake: `Ctrl+Shift+P` → `CMake: Configure`
4. 功能:
   - **Go to Definition**: `Ctrl+Click` 或 `F12` 跳转到定义
   - **Go to References**: `Shift+F12` 查看所有引用
   - **Find All Definitions**: `Ctrl+Shift+O` 符号搜索

## 核心架构

### 初始化顺序（在 User/main.c 中）
1. **串口 & DMA 配置**: USART1 (RC遥控) + USART6 (电机命令) + DMA 双缓冲接收
2. **系统时钟**: `HSE_SetSysClock(12, 336, 2, 7)` → 168 MHz
3. **上电等待**: `while(tim_t < 100000)` 确保硬件稳定
4. **PID 初始化**: `PID_Init()` 为 9 个电机初始化双环 PID (速度 + 位置)
5. **定时器启动**: `TIMx_Configuration()` 产生 1ms 周期中断
6. **主循环**: 1ms 中断触发步态更新，主循环轮询发送数据（无阻塞等待）

### 1ms 中断的关键职责 (GENERAL_TIM_IRQHandler)
位置: [User/stm32f4xx_it.c](User/stm32f4xx_it.c#L174)
- 更新步态时间计数 (`time_currently`, `time_turn`)
- 判断步态切换点 (`Dog_flag = 0` 触发主循环的步态计算)
- 更新跳跃/空翻计时
- **必须轻量级**：只做整数计数，不涉及浮点运算或串口操作

### 主循环的执行流
[User/main.c](User/main.c#L165)
```
while(1):
  ├─ rc_rc_date()              # RC 通道解析
  ├─ Motor_Key()               # 按键控制电机启停
  ├─ IMU_pid_count(0.0f)       # IMU 陀螺仪 PID（Z轴）
  └─ if(step):                 # step 由中断 set（初值 0）
      ├─ foot_track()          # 足端轨迹（贝塞尔曲线）
      ├─ counter_motion()      # 逆运动学计算目标关节角度
      ├─ Motor_pid_count()     # 速度+位置 PID 控制
      ├─ Motor_data_update()   # 将 PID 输出写入电机命令结构体
      └─ step = 0              # 清除标志等待下一个中断周期
  └─ Motor_date_send()         # 轮流发送电机命令到 USART6（含小延迟防溢出）
```

## 关键全局数据结构（1-based 索引）

### 电机数据（9 个关节，例如电机 2~9）
[User/main.c](User/main.c#L19-L23) 定义
```c
MOTOR_send   motor2_control_data, motor3_control_data, ..., motor9_control_data;
MOTOR_recv   motor2_feedback_data, motor3_feedback_data, ..., motor9_feedback_data;
```
- **MOTOR_send**: 位置目标、速度、力矩限制（由 PID 输出赋值）
- **MOTOR_recv**: 当前位置、速度、温度、错误码（通过 USART6 DMA 接收）
- 详见: [User/GO-M8010-6/GO-M8010-6.h](User/GO-M8010-6/GO-M8010-6.h)

### PID 参数与输出
[User/main.c](User/main.c#L26-L79) 定义
```c
const fp32 Motor2_speed_PID_data[3] = {0.01f, 0.0006f, 0.015f};  // [Kp, Ki, Kd]
const fp32 Motor2_rang_PID_data[3] = {40.90f, 0.0f, 40.0f};
float Motor2_speed_PID_OUT, Motor2_rang_PID_OUT;
```
**两级级联**: 位置环 PID → 速度指令 → 速度环 PID → 力矩输出

### 足端轨迹与转向
```c
float foot_track_x1, foot_track_y1;  // 4 条腿的足端坐标 (x,y)
float foot_track_x2, foot_track_y2;
// ...
float rang__2, rang__3, ..., rang__9;  // 9 个关节的目标角度
int time_currently;   // 步态内时间 [0, time_speed)
int time_turn;        // 转向内时间 [0, 400)
int step_turn_flag;   // 步幅/转向切换标志
```

### RC 接收（遥控数据）
```c
extern uint8_t RC[18];           // USART1 DMA 接收缓冲
extern struct {...} rc_rc;       // 解析后的通道值、开关状态
```

## 关键文件导览

| 文件 | 职责 |
|------|------|
| [User/main.c](User/main.c) | 启动、主循环、步态分支控制、PID 参数定义 |
| [User/bsp_ma_su/bsp_ma_su.c](User/bsp_ma_su/bsp_ma_su.c) | 步态计算、PID 初始化、电机数据更新发送 |
| [User/stm32f4xx_it.c](User/stm32f4xx_it.c) | GENERAL_TIM_IRQHandler（1ms 中断）、DMA 中断处理 |
| [User/GO-M8010-6/GO-M8010-6.c](User/GO-M8010-6/GO-M8010-6.c) | 电机协议编码/解码、CRC 校验 |
| [User/pid/pid.c](User/pid/pid.c) | PID 计算核心（位置/速度模式） |
| [User/bsp_rc/bsp_rc.c](User/bsp_rc/bsp_rc.c) | RC 通道解析 |
| [User/bsp_imu/bsp_imu.c](User/bsp_imu/bsp_imu.c) | IMU 陀螺仪数据读取与 Z 轴稳定控制 |
| [User/tim/bsp_general_tim.c](User/tim/bsp_general_tim.c) | 定时器配置（1ms 周期）|

## 通信协议

### 电机通信 (GO-M8010-6 驱动)
- **接口**: USART6 (波特率: 115200 baud，见 [User/bsp_usart_dma/bsp_usart_dma.c](User/bsp_usart_dma/bsp_usart_dma.c))
- **数据流**: 主控 → 电机（命令）；电机 → 主控（反馈，通过 DMA 中断接收）
- **数据包格式**: 见 [User/GO-M8010-6/GO-M8010-6.h](User/GO-M8010-6/GO-M8010-6.h) 中 `MOTOR_send` / `MOTOR_recv` 结构体（pragma pack(1)）
- **重要**: `modify_data()` / `extract_data()` 处理字节序和 CRC；修改协议前务必同步校验

### RC 遥控
- **接口**: USART1 (DMA 接收)
- **格式**: 18 字节帧，通过 `rc_rc_date()` 解析为通道值 (`rc_rc.ch0` ~ `rc_rc.ch3`) 和开关 (`rc_rc.s1`, `rc_rc.s2`)

## 常见编码模式

### 1. 修改 PID 参数
参考: [User/main.c](User/main.c#L26-L28)
```c
// User/main.c 中定义
const fp32 Motor2_speed_PID_data[3] = {Kp, Ki, Kd};

// bsp_ma_su.c 中初始化
PID_struct[id] = PID_Init(..., Motor2_speed_PID_data, ...);

// 修改时：需同时更新定义、初始化点 和所有引用点
// 搜索: "Motor2_speed_PID_data" 确保一致性
```

### 2. 添加新的步态模式
参考: [User/main.c](User/main.c#L169-L192)
- 在条件分支中添加新 `if(step == N)` 判断
- 调用 `foot_track()` 计算足端轨迹
- 调用 `counter_motion()` 计算逆运动学
- 确保 `foot_track_xN`, `foot_track_yN` 遵循设定的轨迹周期

### 3. 调整中断优先级
见: [User/stm32f4xx_it.c](User/stm32f4xx_it.c) 和 [User/tim/bsp_general_tim.c](User/tim/bsp_general_tim.c)
- 不要修改 GENERAL_TIM_IRQHandler 的时间复杂度（保证 < 0.5ms）
- 若需添加新中断，在 stm32f4xx_it.c 中新增并在 NVIC 配置中设置优先级 (TIM > USART6_DMA > RC_DMA)

## 调试技巧

- **监视变量**: `motor2_feedback_data.Pos`, `Motor2_speed_PID_OUT`, `time_currently`
- **日志输出**: 通过 [User/usart/bsp_debug_usart.h](User/usart/bsp_debug_usart.h) 的 `printf()` 到调试串口
- **DMA 接收失败排查**: 检查 USART6 波特率、DMA 通道配置、CRC 校验（见 [User/GO-M8010-6/GO-M8010-6.c](User/GO-M8010-6/GO-M8010-6.c)）

## 修改前的快速检查清单

1. **影响 1ms 中断?** 若是，需保证代码在 < 0.5ms 内完成
2. **涉及电机命令格式?** 同步检查 [User/GO-M8010-6/GO-M8010-6.c](User/GO-M8010-6/GO-M8010-6.c) 中 CRC 和字节序
3. **修改全局电机/PID 数据?** grep 搜索所有引用点并一次更新定义、初始化、使用点
4. **新增中断处理?** 确认优先级和与 TIM 的不冲突
5. **改主循环流程?** 保留 `step = 0` 以及 `Motor_date_send()` 顺序不变
